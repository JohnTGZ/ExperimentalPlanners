#include <global_planner/dsl.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::PathPlanDS, nav_core::BaseGlobalPlanner)

namespace global_planner
{

using namespace std;

PathPlanDS::PathPlanDS() {}

PathPlanDS::PathPlanDS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
}

PathPlanDS::PathPlanDS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
    initialize(name, costmap, global_frame);
} 

void PathPlanDS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void PathPlanDS::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);
    global_frame_ = global_frame;
    costmap_ = costmap;    
    saveCostmapProperties();
    setCostmap(); //only set it at initialization
    
    private_nh.param("num_search", num_search, 500000);
    private_nh.param("allotted_time", allotted, 5.0);
    private_nh.param("allow_unknown", allow_unknown, false);
    private_nh.param("obs_cost_ratio",obs_cost_ratio, 1.0);
    private_nh.param("DS/validate_first", validate_first, true);
    private_nh.param("DS/step", step, 3);
    private_nh.param("DS/allow_smooth", allow_smooth, false);
    private_nh.param("DS/cheap_cost", cheap_cost, 150);
    // private_nh.param("neutral_cost", COST_NEUTRAL, 50);
    // private_nh.param("cost_factor", COST_FACTOR, 0.8);
    private_nh.param("publish_scale", publish_scale_, 100);
    private_nh.param("publish_potential",publish_potential_, true);
    private_nh.param("publish_plan",publish_plan_, true);

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

    max_nodes = 0;
    max_time = 0;
    
    firsttime_ = true;
    recurring = false;
    replan_ = false;

    initialized_ = true;
  }

}

bool PathPlanDS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(!initialized_)
  {
      ROS_ERROR("Initialize before calling makePlan");
      return false;
  }

  plan.clear();

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
  
  State start_state = State({float(start_x), float(start_y), float(0)});
  State goal_state = State({float(goal_x), float(goal_y), float(0)});
  setStart(start_state);
  setGoal(goal_state); //if goal has changed, recurring = false
  
  if (firsttime_ || !recurring)
  {
    this->last_start = this->start;
    init(); //resets all data structures and set the goal
    firsttime_ = false;
  }
    
  auto path_states = calcPath();

  // extract the plan
  ros::Time plan_time = ros::Time::now();
  for (const auto &s : path_states)
  {
      double world_x, world_y;
      costmap_->mapToWorld(s[0], s[1], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
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

  if(publish_plan_){
    publishPlan(plan);
  }

  if(publish_potential_){
      publishPotential();
  }

    MaxNodes();

  return !plan.empty();

}


void PathPlanDS::MaxNodes()
{
    int nodes = rhs.size();
    
    if (max_nodes < nodes)
        max_nodes = nodes;
    std::cout << "Maximum nodes expanded: " << max_nodes << std::endl;
}

void PathPlanDS::saveCostmapProperties()
{
    nx = costmap_->getSizeInCellsX();
    ny = costmap_->getSizeInCellsY();
    // ox_ = costmap->getOriginX(); 
    // oy_ = costmap->getOriginY(); 
    map_res = costmap_->getResolution();
    ns = nx * ny;    
    cmlocal.resize(ns);
}

void PathPlanDS::reset()
{
  ulisthash.clear();
  from.clear();
  gval.clear();
  rhs.clear();
  km_ = 0;
  allow_obs_plan_through = false;
  while (!ulist.empty())
  {
    ulist.pop();
  }
}

void PathPlanDS::init()
{
  if(firsttime_)
  {
    reset();
  }
  else 
  {
    ulisthash.clear();
    from.clear();
    km_ = 0;
    allow_obs_plan_through = false;
    while (!ulist.empty())
    {
      ulist.pop();
    }
  }
  from[goal] = goal;
  rhs[goal] = 0;
  ulist.push(DQPair(calcKey(goal), goal));
  ulisthash.insert(goal);
}




void PathPlanDS::setCostmap()
{
  unsigned char *cmap = costmap_ros_->getCostmap()->getCharMap();

  for (int i = 0; i < ny; i++)
  {
    int k = i * nx;
    for (int j = 0; j < nx; j++,  cmap++)
    {
      cmlocal[k + j] = costmap_2d::LETHAL_OBSTACLE;
      int v = *cmap;
      if (v < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        v = COST_NEUTRAL + COST_FACTOR * v;
        if (v >= costmap_2d::LETHAL_OBSTACLE)
          v = costmap_2d::LETHAL_OBSTACLE - 1;
        cmlocal[k + j] = v;
      }
      else if (v == costmap_2d::NO_INFORMATION && allow_unknown ) //
      {
        v = costmap_2d::NO_INFORMATION;
        cmlocal[k + j] = v;
      }
    }
  }
}


void PathPlanDS::setGoal(const State &goal)
{
  IState igoal = stateToIState(goal);
  recurring = igoal == this->goal; //check if current and previous goal are the same
  this->goal = igoal;
  // ROS_INFO("Set goal as: %d %d, cell cost: %d", igoal.x, igoal.y, costState(igoal));
}

void PathPlanDS::setStart(const State &start)
{
  this->start = stateToIState(start);
  // ROS_INFO("Set start as: %d %d %.3f, cell cost: %d", istart.x, istart.y, istart.z, costState(istart));
}

void PathPlanDS::MaxTime(double ttrack)
{   
    if (ttrack > max_time)
    {
        max_time = ttrack;
    }
    ROS_INFO("Max time taken: %f", max_time);
}

std::vector<State> PathPlanDS::calcPath()
{
  if(replan_ && recurring)
  {
    std::cout<<"replanning"<<std::endl;
    replan();
    std::vector<State> sol;
    sol = extractPath_min();
    // ROS_INFO("Found solution of length %lu", sol.size());
    return sol;
  }

  bool found_solution = computeShortestPath();

  if (!found_solution && !allow_obs_plan_through)
  {
    ROS_ERROR("No path found with no dynamic obstacle plan-through");
    ROS_ERROR("Reattempting planning with dynamic obstacle plan-through allowed");
    init();
    allow_obs_plan_through = true;
    found_solution = computeShortestPath();
    allow_obs_plan_through = false;
  }
  
  std::vector<State> sol;
  if (found_solution)
  {
    sol = extractPath_min();
    // ROS_INFO("Found solution of length %lu", sol.size());
  }
  replan_ = true;
  return sol;
}

bool PathPlanDS::computeShortestPath()
{
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  bool found_solution = false;
  int num_loop = 0;
  float time_left = allotted;
  // removed (!ulist.empty()) &&
  while (((ulist.top().first < calcKey(start)) ||
          (different(getRhs(start), getg(start)))))
  {
    IState u;
    DQPair qp;

    num_loop++;

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

    qp = ulist.top(); //retrieve smallest priority
    ulist.pop();
    if (!isValid(qp)) //if not in ulisthash, skip to next itr
      continue;
    ValuePair key_old = qp.first; 
    u = qp.second;

    auto c = ulisthash.find(u);
    ulisthash.erase(c);
    // ulisthash.erase(u);
    ValuePair key_new = calcKey(u);

    // if (num_loop < 5)
    // { 
    //   int idx = u.x + u.y * nx;
    //   std::cout << "ulist size: " << ulist.size() << std::endl;
    //   std::cout << "current cell idx: " << idx << " key: " << key_old.first << ", " << key_old.second << std::endl; 
    //   std::cout << "new key: " << key_new.first << ", " << key_new.second << std::endl;
    // }

    if (key_old < key_new) //compare smallest priority with the priority it should have had
    {
      insertUList(key_new, u);
    }
    else if (getg(u) > getRhs(u)) //locally overconsistent
    {
      gval[u] = rhs[u];
      for (IState s : neighbors(u)) 
      {
        UpdateVertex(s);
      }
    }
    else //locally underconsistent
    {
      gval[u] = MAX_COST;
      for (IState s : neighbors(u))  
      {
        UpdateVertex(s);
      }
      UpdateVertex(u);
    }
    std::chrono::steady_clock::time_point inside = std::chrono::steady_clock::now();
    double spent = std::chrono::duration_cast<std::chrono::duration<double>>(inside - t1).count();
    time_left = allotted - spent;
  }
  found_solution = (num_loop <= num_search && time_left >= 0 && !(getg(start)>=MAX_COST)); 
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

  double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  ROS_INFO("Take: %.5fs searching: %d", ttrack, num_loop);
  MaxTime(ttrack);

  return found_solution;
}



ValuePair PathPlanDS::calcKey(const IState &s)
{
  float sv = smaller(getg(s), getRhs(s));
  return ValuePair(sv + calcHCost(s) + km_, sv);
}

void PathPlanDS::UpdateVertex(IState u)
{
  if (u != goal)
  {
    float min = MAX_COST;
    IState f;
    for (IState s_ : neighbors(u)) 
    {
      float nc = getg(s_) + calcDist(u, s_);
      if (min > nc)
      {
        min = nc;
        f = s_;
      }
    }
    rhs[u] = min; //unordered map
    from[u] = f; //set parents
  }
  if (inUList(u)) 
  {
    removeUList(u);
  }
  if (different(getg(u), getRhs(u))) //not locally consistent
  {
    insertUList(calcKey(u), u);
  }
}

vector<IState> PathPlanDS::simpleNeighbors(IState s)
{
  vector<IState> n;
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

vector<IState> PathPlanDS::neighbors(IState s)
{
  vector<IState> n;
  if (!isFree(s))
    return n;
  return simpleNeighbors(s);
}

int PathPlanDS::equivalent(int v)
{
  if (v < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    v = COST_NEUTRAL + COST_FACTOR * v;
    if (v >= costmap_2d::LETHAL_OBSTACLE)
      v = costmap_2d::LETHAL_OBSTACLE - 1;
    return v;
  }
  else if (v == costmap_2d::NO_INFORMATION && allow_unknown ) 
  {
    v = costmap_2d::NO_INFORMATION;
    return v;
  }
  else
    return costmap_2d::LETHAL_OBSTACLE;
}


void PathPlanDS::replan()
{
  bool replan_required = false;
  unsigned char *cmap = costmap_ros_->getCostmap()->getCharMap();

  int x0 = 0;
  int y0 = 0;
  unsigned int w = this->nx;
  unsigned int h = this->ny;

  std::vector<IState> changed;
  if (x0 < 0 || x0 + int(w) > nx) 
  {
    ROS_WARN("x update out of bound");
    return;
  }
  if (y0 < 0 || y0 + int(h) > ny)
  {
    ROS_WARN("y update out of bound");
    return;
  }
  
  for (unsigned int y = 0; y < ny; y++)
  {
    int k = y * nx;
    for (unsigned int x = 0; x < nx; x++, cmap++) //k++,
    {
      if (outOfBound(x, y)) 
      {
        continue;
      }
      int c = equivalent(*cmap);
      if (costState(x, y) != c ) //check for changes in costmap costs
      {
        changed.push_back(IState(x, y)); 
        cmlocal[k+x] = c;
      }
    }
  }

  if (!changed.empty()) //if there are any changes
  {
    km_ = km_ + calcHCost(last_start);
    this->last_start = this->start;
    // ROS_INFO("Changed: %lu", changed.size());
    for (unsigned int i = 0; i < changed.size(); i++)
    {
      UpdateVertex(changed[i]);
    }
    replan_required = true; 
  }

  if (replan_required)
  {
    computeShortestPath();
  }
}

bool PathPlanDS::inUList(IState s)
{
  return ulisthash.find(s) != ulisthash.end();
}

void PathPlanDS::removeUList(IState s)
{
  auto c = ulisthash.find(s);
  if (c == ulisthash.end())
    return;
  ulisthash.erase(c);
}

void PathPlanDS::insertUList(ValuePair total, IState s)
{
  DQPair q(total, s);
  ulist.push(q);
  ulisthash.insert(s);
}

float PathPlanDS::getRhs(const IState &s)
{
  // if (s == goal)
  //   return 0;
  if (rhs.find(s) == rhs.end())
    return MAX_COST;
  return rhs[s];
}

float PathPlanDS::getg(const IState &s)
{
  if (gval.find(s) == gval.end())
    return MAX_COST;
  return gval[s];
}

int PathPlanDS::costState(const IState &s)
{
  if (outOfBound(s.x, s.y))
    return costmap_2d::NO_INFORMATION;
  return cmlocal[s.y * nx + s.x];
}

int PathPlanDS::costState(unsigned int x, unsigned int y)
{
  if (outOfBound(x, y))
    return costmap_2d::NO_INFORMATION;
  return cmlocal[y * nx + x];
}

float PathPlanDS::distToStart(const IState &s)
{
  float dist = sqrt((start.x - s.x)*(start.x - s.x) + (start.y - s.y)*(start.y - s.y));
  return dist;
}

float PathPlanDS::calcHCost(const IState &s)
{
  float d = distToStart(s);
  return d * COST_NEUTRAL;
}

float PathPlanDS::DistState(IState p, IState t) {
    return sqrt((p.x-t.x)*(p.x-t.x) + (p.y-t.y)*(p.y-t.y));
}

float PathPlanDS::calcDist(IState p, IState t)
{
  return DistState(p, t) * costState(t);
}

float PathPlanDS::DistConnNeighbour(IState p, IState t) {
  int dx = abs(p.x - t.x);
  int dy = abs(p.y - t.y);
  if (dx + dy >= 2) return 1.4;
  return 1;
}

float PathPlanDS::smaller(float x, float y)
{
  if (x < y)
    return x;
  return y;
}

bool PathPlanDS::different(float x, float y)
{
  if (isinf(x) && isinf(y))
    return false;
  return (fabs(x - y) > TOL);
}

bool PathPlanDS::isFree(const IState &s)
{
  int cost = costState(s);
  int obs_cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE * obs_cost_ratio;
  obs_cost = obs_cost * COST_FACTOR + COST_NEUTRAL;
  bool check = allow_obs_plan_through ? (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) : (cost < obs_cost);
  check = allow_unknown ? (check || cost == costmap_2d::NO_INFORMATION) : check;

  return check;
}

bool PathPlanDS::isFree(int x, int y)
{
  return isFree(IState(x, y));
}

bool PathPlanDS::isCheap(int x, int y)
{
  return isCheaper(x, y, cheap_cost);
}

bool PathPlanDS::isCheaper(int x, int y, float thres)
{
  if (outOfBound(x, y))
    return false;
  float cost = cmlocal[y * nx + x];
  return (cost < thres);
}

bool PathPlanDS::outOfBound(int x, int y)
{
  return(x <0 || x > nx || y <0 || y > ny);
}

bool PathPlanDS::isValid(DQPair qp)
{
  StateSet::iterator i = ulisthash.find(qp.second);
  if (i == ulisthash.end())
    return false;
  return true;
}

bool PathPlanDS::lineOfSight(const IState &s, const IState &n)
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


std::vector<State> PathPlanDS::extractPath_parents()
{
  IState current = this->start;
  std::vector<State> sol;
  // IState c = goal;
  while (current != goal)
  {
    sol.push_back(IStateToState(current));
    current = from[current];
  }
  sol.push_back(IStateToState(goal));
  reverse(sol.begin(), sol.end());

  return sol;
}

/** @brief Traces the path of min g-cost from goal to start
 */
std::vector<State> PathPlanDS::extractPath_min()
{
  IState current = this->start;
  std::vector<IState> sol;

  IState min_nb_cell;
  while (current != goal)
  {
    sol.push_back(current);

    std::vector<IState> n = neighbors(current);
    float min_nb_cost = MAX_COST;
    for (IState s : n) //Go to the cheapest neighbor of current
    {
      float cs = calcDist(current, s) + getg(s);
      if (cs < min_nb_cost)
      {
        min_nb_cost = cs;
        min_nb_cell = s;
      }
    }
    current = min_nb_cell;
  }

  if (sol.size() > 0)
  {
    sol.push_back(goal);
  }

  std::vector<State> sols;

  if (allow_smooth) //smooth using LOS, like Post processed A*
  {
    std::vector<IState> smoothen;
    smoothen = smooth(sol);
    for (IState s : smoothen)
    {
      sols.push_back(IStateToState(s));
    }
    return sols;
  }

  for (IState s : sol)
  {
    sols.push_back(IStateToState(s));
  }

  return sols;
}



vector<IState> PathPlanDS::smooth(vector<IState> orig)
{
  vector<IState> filter;
  unsigned int i = 1;
  int p = 0;

  if (orig.size() > 0)
    filter.push_back(orig[0]);

  while (i < orig.size())
  {
    if (!(/*DistState(orig[i], orig[p]) < step &&*/ lineOfSight(orig[i], orig[p])))
    {
      filter.push_back(orig[i]);
      p = i;
    }
    i++;
  }
  if (orig.size() > 0)
  {
    filter.push_back(orig[orig.size() - 1]);
  }
  return filter;
}


void PathPlanDS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
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


void PathPlanDS::publishPotential()
{
    nav_msgs::OccupancyGrid grid;

    // Publish Whole Grid
    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = map_res;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - map_res / 2;
    grid.info.origin.position.y = wy - map_res / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    for (unsigned int i = 0; i < grid.data.size(); i++) 
    {
      grid.data[i] = -1;
    }

    float max = 0.0;
    auto it = rhs.begin();
    while (it!= rhs.end())
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

    it = rhs.begin();
    while (it!= rhs.end())
    {
      IState pos = it-> first;
      
      int idx = pos.x + pos.y * nx;
      grid.data[idx] =  max * publish_scale_ / max;
      it++;
    }

    potential_pub_.publish(grid);

}

} // namespace global_planner

