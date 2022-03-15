#include <global_planner/PathPlanTheta.h>
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(global_planner::PathPlanTheta, nav_core::BaseGlobalPlanner)

namespace global_planner
{
using namespace std;
PathPlanTheta::PathPlanTheta(): costmap_(NULL),costmap_ros_(NULL), initialized_(false), costarr(NULL) {}

PathPlanTheta::~PathPlanTheta()
{
  delete costarr;
}

PathPlanTheta::PathPlanTheta(std::string name, costmap_2d::Costmap2DROS *costmap_ros): costmap_(NULL),costmap_ros_(NULL), initialized_(false), costarr(NULL)
{

    initialize(name, costmap_ros);
}

PathPlanTheta::PathPlanTheta(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame): costmap_(NULL),costmap_ros_(NULL), initialized_(false), costarr(NULL)
{
    initialize(name, costmap, global_frame);
} 

void PathPlanTheta::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    std::cout<< "costmap_ros_ assigned " << std::endl;
    costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void PathPlanTheta::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);

        unsigned int xs = costmap->getSizeInCellsX();
        unsigned int ys = costmap->getSizeInCellsY();
        this->map_res = costmap->getResolution();
        setup_map(xs, ys);
        costmap_ = costmap;
      
        private_nh.param("num_search", num_search, 500000);
        private_nh.param("allotted_time", allotted, 5.0);
        private_nh.param("allow_unknown", allow_unknown, false);
        private_nh.param("obs_cost_ratio",obs_cost_ratio, 1.0);
        // nh.param("Theta/step", step, 3);
        private_nh.param("Theta/cheap_cost", cheap_cost, 150);

        init();
        setCostmap(costmap_ros_); //must be called after setup_map and init()
        initialized_ = true;
    }
}

bool PathPlanTheta::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_)
    {
        ROS_ERROR("Initialize before calling makePlan");
        return false;
    }

    plan.clear();

    //MODULE 2: Checked start and goal positions are within bounds
    double start_x_w = start.pose.position.x; 
    double start_y_w = start.pose.position.y;
    double goal_x_w = goal.pose.position.x; 
    double goal_y_w = goal.pose.position.y;
    unsigned int start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(start_x_w, start_y_w, start_x, start_y)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if (!costmap_->worldToMap(goal_x_w, goal_y_w, goal_x, goal_y)) {
        ROS_WARN_THROTTLE(1.0,"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    // setCostmap(costmap_ros_);

    State start_state = State({float(start_x), float(start_y), float(0)});
    State goal_state = State({float(goal_x), float(goal_y), float(0)});

    setStart(start_state);
    setGoal(goal_state);

    auto path_states = calcPath();

    // extract the plan
    ros::Time plan_time = ros::Time::now();
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    for (const auto &s : path_states)
    {
        double world_x, world_y;
        costmap_->mapToWorld(s[0], s[1], world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        // tf::quaternionTFToMsg(tf::createQuaternionFromYaw(s[2]), pose.pose.orientation);
        plan.push_back(pose);
    }

    if (!plan.empty())
    {
        plan.push_back(goal);
    }

    return !plan.empty();

}

IState PathPlanTheta::stateToIState(const State& s) {
    if (s.size() == 2) {
        return IState(s[0], s[1]);
    }
    if (s.size() == 3) {
        return IState(s[0], s[1], s[2]);
    }
    return IState();
}

State PathPlanTheta::IStateToState(const IState& is) {
    State s = {float(is.x), float(is.y), float(is.z)};
    return s;
}

void PathPlanTheta::setGoal(const State &goal)
{
  IState igoal = stateToIState(goal);
  ROS_INFO("Set goal as: %d %d, cell cost: %d", igoal.x, igoal.y, costState(igoal));
  this->goal = igoal;
  init();
}

void PathPlanTheta::setStart(const State &start)
{
  IState istart = stateToIState(start);
  ROS_INFO("Set start as: %d %d %.3f, cell cost: %d", istart.x, istart.y, istart.z, costState(istart));
  this->start = istart;
}

void PathPlanTheta::setup_map(int nx, int ny)
{
  std::cout<< "map is set up " << std::endl;
    this->nx = nx;
    this->ny = ny;
    int ns = nx * ny;
    costarr = new COSTTYPE[ns]; // cost array, 2d config space
    memset(costarr, 0, ns * sizeof(COSTTYPE));
}

void PathPlanTheta::setCostmap(costmap_2d::Costmap2DROS *costmap_ros)
{
  std::cout << "costmap is set" << std::endl;
  COSTTYPE *cm = costarr;
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
  COSTTYPE *cmap = costmap_ros->getCostmap()->getCharMap();

  int cyc = 0;
  for (int i = 0; i < ny; i++)
  {
    int k = i * nx;
    for (int j = 0; j < nx; j++, k++, cmap++, cm++) //go through each array simultaneously
    {
      *cm = COST_OBS; // assign forbidden (unknown)
      int v = *cmap; // v = value of costmap

      if (cyc++ > 50000 && cyc < 100000)
        std::cout << v << " ";
        
      if (v < COST_OBS_ROS) //if costmap is not obstacle
      {
        v = COST_NEUTRAL + COST_FACTOR * v;  //assign neutral cost to current cell
        if (v >= COST_OBS)                   //if > forbidden region (254)
          v = COST_OBS - 1;                  //make it an obstacle (253) or > forbidden
        *cm = v;                             //assign to local cost array
        //*cm = COST_NEUTRAL;
      }
      else if (v == COST_UNKNOWN_ROS && allow_unknown) //if unknown and unknowns are allowed
      {
        v = COST_UNKNOWN_ROS;                          //assign unknown cost (255)
        *cm = v;                          
      }
    }
  }
  std::cout << "cycles " << cyc;
}

void PathPlanTheta::reset()
{
  openhash.clear();
  from.clear();
  cost.clear();
  visited.clear();
  allow_obs_plan_through = false;
  while (!open.empty())
    open.pop();
}

void PathPlanTheta::init()
{
  std::cout << "init method called "<< std::endl;
  reset();
  from[start] = start;
  cost[start] = 0;
  open.push(TQPair(heuristic(start), start));
  openhash.insert(start);
}

std::vector<State> PathPlanTheta::calcPath()
{
  bool found_solution = CalculatePath();
  if (!found_solution && !allow_obs_plan_through)
  {
    ROS_WARN("No path found with no dynamic obstacle plan-through");
    ROS_WARN("Reattempting planning with dynamic obstacle plan-through allowed");
    init();
    allow_obs_plan_through = true;
    found_solution = CalculatePath(); 
  }
  std::vector<State> sol;
  if (found_solution)
  {
    sol = extractPath();
    ROS_INFO("Found solution of length %lu", sol.size());
  }
  return sol;
}

std::vector<State> PathPlanTheta::extractPath()
{
  vector<State> sol;
  IState c = goal;
  while (c != start)
  {
    sol.push_back(IStateToState(c));
    c = from[c];
  }
  sol.push_back(IStateToState(start));
  reverse(sol.begin(), sol.end());

  return sol;
}

bool PathPlanTheta::CalculatePath()
{
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  bool found_solution = false;
  int num_loop = 0;
  float time_left = allotted;

  while (!open.empty())
  {
    IState p; // coordinates (x,y,z)
    TQPair qp; //fcost and coordinates (x,y,z)

    num_loop++;

    // check limits on loop and time taken
    if (num_loop > num_search)
    {
      ROS_WARN("No solution found after searching %d loops", num_loop);
      break;
    }
    if (time_left < 0)
    {
      ROS_WARN("No solution found after searching for %.2f s", allotted);
      break;
    }

    qp = open.top(); //get cheapest element
    open.pop();
    p = qp.second; //coordinate of current element

    if (isGoalReached(p))
    {
      found_solution = true;
      break;
    }

    if (!isValid(qp)) // only execute expansion if not inside openhash
      continue;

    StateSet::iterator cur = openhash.find(p);
    openhash.erase(p);

    visited.insert(p); 
    for (IState t : neighbors(p))
    {
      if (visited.count(t)) 
        continue;

      if (openhash.find(t) == openhash.end()) //if this neighbor is not in openhash
        cost[t] = INFINITY;
      UpdateVertex(p, t); 
    }

    std::chrono::steady_clock::time_point inside = std::chrono::steady_clock::now();
    double spent = std::chrono::duration_cast<std::chrono::duration<double>>(inside - t1).count();
    time_left = allotted - spent;
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  ROS_INFO("Time taken: %.5f searching: %d", ttrack, num_loop);
  return found_solution;
}

void PathPlanTheta::UpdateVertex(IState p, IState t)
{
  float old_cost = cost[t];
  ComputeCost(p, t);
  if (cost[t] < old_cost)
  {
    if (openhash.find(t) != openhash.end()) //if t is in openhash, then remove t
      remove(t);
    insert(cost[t] + heuristic(t), t); //add to open and openhash
  }
}

/** check if inside openhash, Returns false if not inside openhash **/
bool PathPlanTheta::isValid(TQPair qp)
{
  StateSet::iterator i = openhash.find(qp.second);
  if (i == openhash.end())
    return false;
  return true;
}

void PathPlanTheta::remove(IState s)
{
  auto c = openhash.find(s);
  if (c == openhash.end())
    return;
  openhash.erase(c);
}

void PathPlanTheta::insert(float total, IState s)
{
  TQPair q(total, s);
  open.push(q);
  openhash.insert(s);
}

/** checks line of sight in addition to computing cost * **/
void PathPlanTheta::ComputeCost(IState p, IState t)
{
  float c;
  if (lineOfSight(from[p], t))
  {
    c = MoveCost(from[p], t);
    if (cost[from[p]] + c < cost[t])
    {
      from[t] = from[p];
      cost[t] = cost[from[p]] + c;
    }
  }
  else
  {
    c = MoveCost(p, t);
    if (cost[p] + c < cost[t])
    {
      from[t] = p;
      cost[t] = cost[p] + c;
    }
  }
}

bool PathPlanTheta::isCheap(int x, int y) 
{ 
  return isCheaper(x, y, cheap_cost);
}

bool PathPlanTheta::isCheaper(int x, int y, float thres)
{
    if (outOfBound(x, y))
        return false;
    float cost = costarr[y * nx + x];
    return (cost < thres);
}

bool PathPlanTheta::outOfBound(int x, int y)
{
return ( !(0 < x && x < nx && 0 < y && y < ny) ) ;
}

float PathPlanTheta::sqr(float x) 
{ 
    return x * x; 
}

float PathPlanTheta::TravelCost(IState p, IState t)
{
  return DistState(p, t) * costState(t);
}

float PathPlanTheta::MoveCost(IState p, IState t)
{
  return DistState(p, t) * costState(t);
}

/** adds neighbors in octile directions **/
vector<IState> PathPlanTheta::neighbors(IState s)
{
  vector<IState> n;
  if (!isFree(s))
    return n;
  if (isFree(s.x + 1, s.y))
    n.push_back(IState(s.x + 1, s.y));
  if (isFree(s.x, s.y + 1))
    n.push_back(IState(s.x, s.y + 1));
  if (isFree(s.x - 1, s.y))
    n.push_back(IState(s.x - 1, s.y));
  if (isFree(s.x, s.y - 1))
    n.push_back(IState(s.x, s.y - 1));
  if (isFree(s.x + 1, s.y + 1))
    n.push_back(IState(s.x + 1, s.y + 1));
  if (isFree(s.x - 1, s.y - 1))
    n.push_back(IState(s.x - 1, s.y - 1));
  if (isFree(s.x + 1, s.y - 1))
    n.push_back(IState(s.x + 1, s.y - 1));
  if (isFree(s.x - 1, s.y + 1))
    n.push_back(IState(s.x - 1, s.y + 1));
  return n;
}

float PathPlanTheta::distToGoal(const IState &s)
{
  float dist = sqrt(sqr(goal.x - s.x) + sqr(goal.y - s.y));
  return dist;
}

int PathPlanTheta::costState(const IState &s)
{
  if (outOfBound(s.x, s.y))
    return COST_OBS + 1; //mark as forbidden
  int cost = costarr[s.y * nx + s.x]; 
  return cost;
}

float PathPlanTheta::DistState(IState p, IState t) 
{
    float dist = sqrt(sqr(p.x-t.x) + sqr(p.y-t.y));
    return dist;
}    

bool PathPlanTheta::isFree(int x, int y)
{
  return isFree(IState(x, y));
}

bool PathPlanTheta::isFree(const IState &s)
{
  int cost = costState(s);
  int obs_cost = COST_OBS_ROS * obs_cost_ratio; 
  obs_cost = obs_cost * COST_FACTOR + COST_NEUTRAL;
  bool check = allow_obs_plan_through ? (cost < COST_OBS_ROS) : (cost < obs_cost); //check if cost is free (< COST_OBS_ROS) or unknown (< obs_cost)
  check = allow_unknown ? (check || cost == COST_UNKNOWN_ROS) : check; 

  return check;
}

float PathPlanTheta::heuristic(const IState &s)
{
  float d = distToGoal(s);
  //int step = 1;
  //float discount = 0.9;
  //float nstep = d / float(step);
  //float h_dist = (1 - pow(discount, nstep)) / (1 - discount) * COST_NEUTRAL;
  return d * COST_NEUTRAL;
  //return h_dist;
}

bool PathPlanTheta::isGoalReached(const IState &s)
{
  return (s == goal);
}

bool PathPlanTheta::lineOfSight(const IState &s, const IState &n)
{
  int x0, y0, x1, y1, dx, dy, p, sx, sy;
  x0 = int(s.x);
  y0 = int(s.y);
  x1 = int(n.x);
  y1 = int(n.y);
  dx = x1 - x0;
  dy = y1 - y0;
  p = 0;
  if (dy < 0)
  {
    dy = -dy;
    sy = -1;
  }
  else
    sy = 1;

  if (dx < 0)
  {
    dx = -dx;
    sx = -1;
  }
  else
    sx = 1;

  if (dx >= dy)
  {
    while (x0 != x1)
    {
      p += dy;
      if (p >= dx)
      {
        if (!isCheap(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2))
          return false;
        y0 += sy;
        p -= dx;
      }

      if (p != 0 && !isCheap(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2))
        return false;
      if (dy == 0 && !isCheap(x0 + ((sx - 1) / 2), y0) && !isCheap(x0 + ((sx - 1) / 2), y0 - 1))
        return false;
      x0 += sx;
    }
  }
  else
  {
    while (y0 != y1)
    {
      p += dx;
      if (p >= dy)
      {
        if (!isCheap(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2))
          return false;
        x0 += sx;
        p -= dy;
      }

      if (p != 0 && !isCheap(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2))
        return false;
      if (dx == 0 && !isCheap(x0, ((sy - 1) / 2) + y0) && !isCheap(x0 - 1, ((sy - 1) / 2) + y0))
        return false;
      y0 += sy;
    }
  }
  return true;
}



} // namespace global_planner
