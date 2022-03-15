#include <global_planner/ThetaStarBasic1.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::ThetaStarBasic1, nav_core::BaseGlobalPlanner)

namespace global_planner
{
using namespace std;
ThetaStarBasic1::ThetaStarBasic1() {}

ThetaStarBasic1::ThetaStarBasic1(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

ThetaStarBasic1::ThetaStarBasic1(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
    initialize(name, costmap, global_frame);
} 

void ThetaStarBasic1::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void ThetaStarBasic1::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);

        unsigned int xs = costmap->getSizeInCellsX();
        unsigned int ys = costmap->getSizeInCellsY();
        this->map_res = costmap->getResolution();
        global_frame_ = global_frame;
        setup_map(xs, ys);
        costmap_ = costmap;

        private_nh.param("num_search", num_search, 500000);
        private_nh.param("allotted_time", allotted, 5.0);
        private_nh.param("allow_unknown", allow_unknown, false);
        private_nh.param("obs_cost_ratio",obs_cost_ratio, 1.0);
        private_nh.param("cost_neutral",COST_NEUTRAL, 50);
        private_nh.param("cost_factor",COST_FACTOR, 0.8);

        private_nh.param("publish_plan",publish_plan_, true);
        private_nh.param("publish_potential",publish_potential_, true);
        private_nh.param("publish_scale", publish_scale_, 100);
        // nh.param("Theta/step", step, 3);
        private_nh.param("Theta/cheap_cost", cheap_cost, 150);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        make_plan_srv_ = private_nh.advertiseService("make_plan", &ThetaStarBasic1::makePlanService, this);

        max_nodes = 0;
        max_time = 0;

        init();
        initialized_ = true;
    }
}

bool ThetaStarBasic1::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
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

    setCostmap(costmap_ros_);

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

    publishPlan(plan);

    if(publish_potential_){
        publishPotential();
    }

    MaxNodes();

    return !plan.empty();

}

void ThetaStarBasic1::MaxNodes()
{
    int nodes = cost.size();
    
    if (max_nodes < nodes)
        max_nodes = nodes;
    std::cout << "Maximum nodes expanded: " << max_nodes << std::endl;
}

IState ThetaStarBasic1::stateToIState(const State& s) {
    if (s.size() == 2) {
        return IState(s[0], s[1]);
    }
    if (s.size() == 3) {
        return IState(s[0], s[1], s[2]);
    }
    return IState();
}

State ThetaStarBasic1::IStateToState(const IState& is) {
    State s = {float(is.x), float(is.y), float(is.z)};
    return s;
}

void ThetaStarBasic1::setGoal(const State &goal)
{
  IState igoal = stateToIState(goal);
  // ROS_INFO("Set goal as: %d %d, cell cost: %d", igoal.x, igoal.y, costState(igoal));
  this->goal = igoal;
  init();
}

void ThetaStarBasic1::setStart(const State &start)
{
  IState istart = stateToIState(start);
  // ROS_INFO("Set start as: %d %d %.3f, cell cost: %d", istart.x, istart.y, istart.z, costState(istart));
  this->start = istart;
}

void ThetaStarBasic1::setup_map(int nx, int ny)
{
    this->nx_ = nx;
    this->ny_ = ny;
    this->ns_ = nx * ny;
    costarr = new COSTTYPE[ns_]; // cost array, 2d config space
    memset(costarr, 0, ns_ * sizeof(COSTTYPE));
}

void ThetaStarBasic1::setCostmap(costmap_2d::Costmap2DROS *costmap_ros)
{
  COSTTYPE *cm = costarr;
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
  COSTTYPE *cmap = costmap_ros->getCostmap()->getCharMap();

  for (int i = 0; i < ny_; i++)
  {
    int k = i * nx_;
    for (int j = 0; j < nx_; j++, k++, cmap++, cm++) //go through each array simultaneously
    {
      *cm = COST_OBS; // assign forbidden (unknown)
      int v = *cmap; 
      if (v < COST_OBS_ROS) //if not obstacle
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
}

void ThetaStarBasic1::init()
{
  reset();
  from[start] = start;
  cost[start] = 0;
  open.push(TQPair(heuristic(start), start));
  openhash.insert(start);
}

void ThetaStarBasic1::reset()
{
  openhash.clear();
  from.clear();
  cost.clear();
  visited.clear();
  allow_obs_plan_through = false;
  while (!open.empty())
    open.pop();
}

std::vector<State> ThetaStarBasic1::calcPath()
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
    // ROS_INFO("Found solution of length %lu", sol.size());
  }
  return sol;
}

std::vector<State> ThetaStarBasic1::extractPath()
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

bool ThetaStarBasic1::CalculatePath()
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

    // only expand if inside openhash
    if (!isValid(qp)) 
      continue;

    StateSet::iterator cur = openhash.find(p);
    openhash.erase(p);

    visited.insert(p); 
    for (IState t : neighbors(p))
    {
      if (visited.count(t)) 
        continue;

      if (openhash.find(t) == openhash.end()) //if this neighbor is not in openhash, set cost to infinity
        cost[t] = INFINITY;
      UpdateVertex(p, t); 
    }

    std::chrono::steady_clock::time_point inside = std::chrono::steady_clock::now();
    double spent = std::chrono::duration_cast<std::chrono::duration<double>>(inside - t1).count();
    time_left = allotted - spent;
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  ROS_INFO("Take: %.5f searching: %d", ttrack, num_loop);

  MaxTime(ttrack);

  return found_solution;
}

void ThetaStarBasic1::MaxTime(double ttrack)
{   
    if (ttrack > max_time)
    {
        max_time = ttrack;
    }
    ROS_INFO("Max time taken: %f", max_time);
}

void ThetaStarBasic1::UpdateVertex(IState p, IState t)
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

/** FALSE if not inside openhash **/
/** TRUE if inside!!!!! **/
bool ThetaStarBasic1::isValid(TQPair qp)
{
  StateSet::iterator i = openhash.find(qp.second);
  if (i == openhash.end())
    return false;
  return true;
}

void ThetaStarBasic1::remove(IState s)
{
  auto c = openhash.find(s);
  if (c == openhash.end())
    return;
  openhash.erase(c);
}

void ThetaStarBasic1::insert(float total, IState s)
{
  TQPair q(total, s);
  open.push(q);
  openhash.insert(s);
}

void ThetaStarBasic1::ComputeCost(IState p, IState t)
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


bool ThetaStarBasic1::isCheaper(int x, int y, float thres)
{
    if (outOfBound(x, y))
        return false;
    float cost = costarr[y * nx_ + x];
    return (cost < thres);
}

bool ThetaStarBasic1::outOfBound(int x, int y)
{
return (!(0 < x && x < nx_ && 0 < y && y < ny_));
}

float ThetaStarBasic1::sqr(float x) 
{ 
    return x * x; 
}

float ThetaStarBasic1::MoveCost(IState p, IState t)
{
  return DistState(p, t) * costState(t);
}

/** adds neighbors in octile directions **/
vector<IState> ThetaStarBasic1::neighbors(IState s)
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

float ThetaStarBasic1::distToGoal(const IState &s)
{
  float dist = sqrt(sqr(goal.x - s.x) + sqr(goal.y - s.y));
  return dist;
}

int ThetaStarBasic1::costState(const IState &s)
{
  if (outOfBound(s.x, s.y))
    return COST_OBS + 1; //if out of bounds, mark as forbidden
  int cost = costarr[s.y * nx_ + s.x]; 
  return cost;
}

float ThetaStarBasic1::DistState(IState p, IState t) 
{
    float dist = sqrt(sqr(p.x-t.x) + sqr(p.y-t.y));
    return dist;
}    

bool ThetaStarBasic1::isFree(int x, int y)
{
  return isFree(IState(x, y));
}

bool ThetaStarBasic1::isFree(const IState &s)
{
  int cost = costState(s);
  int obs_cost = COST_OBS_ROS * obs_cost_ratio; 
  obs_cost = obs_cost * COST_FACTOR + COST_NEUTRAL;
  bool check = allow_obs_plan_through ? (cost < COST_OBS_ROS) : (cost < obs_cost); //check if cost is free (< COST_OBS_ROS) or unknown (< obs_cost)
  check = allow_unknown ? (check || cost == COST_UNKNOWN_ROS) : check; 

  return check;
}

float ThetaStarBasic1::heuristic(const IState &s)
{
  float d = distToGoal(s);
  //int step = 1;
  //float discount = 0.9;
  //float nstep = d / float(step);
  //float h_dist = (1 - pow(discount, nstep)) / (1 - discount) * COST_NEUTRAL;
  return d * COST_NEUTRAL;
  //return h_dist;
}

bool ThetaStarBasic1::isGoalReached(const IState &s)
{
  return (s == goal);
}

bool ThetaStarBasic1::lineOfSight(const IState &s, const IState &n)
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
        if (!isCheaper(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2, cheap_cost ))
          return false;
        y0 += sy;
        p -= dx;
      }

      if (p != 0 && !isCheaper(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2, cheap_cost ))
        return false;
      if (dy == 0 && !isCheaper(x0 + ((sx - 1) / 2), y0, cheap_cost ) && !isCheaper(x0 + ((sx - 1) / 2), y0 - 1, cheap_cost ))
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
        if (!isCheaper(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2, cheap_cost ))
          return false;
        x0 += sx;
        p -= dy;
      }

      if (p != 0 && !isCheaper(x0 + ((sx - 1) / 2), y0 + (sy - 1) / 2, cheap_cost ))
        return false;
      if (dx == 0 && !isCheaper(x0, ((sy - 1) / 2) + y0, cheap_cost ) && !isCheaper(x0 - 1, ((sy - 1) / 2) + y0, cheap_cost ))
        return false;
      y0 += sy;
    }
  }
  return true;
}

void ThetaStarBasic1::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;  
    }

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(path.empty()) //check for empty path
    {
        gui_path.header.frame_id = global_frame_; //still set a valid frame so visualization won't hit transform issues
        gui_path.header.stamp = ros::Time::now();
    }
    else {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }    

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i=0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i]; 
    }

    plan_pub_.publish(gui_path);
}

void ThetaStarBasic1::publishPotential()
{
    nav_msgs::OccupancyGrid grid;

    // Publish Whole Grid
    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = map_res;

    grid.info.width = nx_;
    grid.info.height = ny_;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - map_res / 2;
    grid.info.origin.position.y = wy - map_res / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx_ * ny_);

    for (unsigned int i = 0; i < grid.data.size(); i++) 
    {
      grid.data[i] = -1;
    }

    float max = 0.0;
    auto it = cost.begin();
    while (it!= cost.end())
    {
      float potential = it->second;
      if (potential < MAX_COST) 
      {
          if (potential > max) 
          {
              max = potential;
          }
      }
      it++;
    }

    it = cost.begin();
    while (it!= cost.end())
    {
      IState pos = it-> first;
      
      int idx = pos.x + pos.y * nx_;
      grid.data[idx] =  max * publish_scale_ / max;
      it++;
    }

    potential_pub_.publish(grid);

}

bool ThetaStarBasic1::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);

  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = global_frame_;

  return true;
}




} // namespace global_planner
