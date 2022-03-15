#include "global_planner/AStar2.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::AStar2, nav_core::BaseGlobalPlanner) //register this planner as a BaseGlobalPlanner plugin
 namespace global_planner
{
AStar2::AStar2(): costmap_(NULL), initialized_(false), allow_unknown_(true) {}

AStar2::~AStar2() {

    // if (planner_)
    //     delete planner_;
    if (cost_calc_)
        delete cost_calc_; 
    if (path_traceback_)
        delete path_traceback_;
}

AStar2::AStar2(std::string name, costmap_2d::Costmap2DROS *costmap_ros): costmap_(NULL), initialized_(false), allow_unknown_(true)
{
    initialize(name, costmap_ros);
}

AStar2::AStar2(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame): costmap_(NULL), initialized_(false), allow_unknown_(true)
{
    initialize(name, costmap, global_frame);
} 

void AStar2::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void AStar2::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);

        global_frame_ = global_frame;
        costmap_ = costmap;      
        saveCostmapProperties(costmap_);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        make_plan_srv_ = private_nh.advertiseService("make_plan", &AStar2::makePlanService, this);

        private_nh.param("allow_unknown", allow_unknown_, true);
        private_nh.param("neutral_cost", neutral_cost_, 50);
        private_nh.param("cost_factor", cost_factor_, 0.8);
        private_nh.param("obstacle_cost_ratio",obs_cost_ratio, 1.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0); 
        
        private_nh.param("con8", con8_, true); //true: 8-connected search, false: 4-connected
        private_nh.param("distance_metric", distance_metric_, 0); //0: L1, 1: L2, 2: Octile, 3: Chebyshev
        private_nh.param("get_path_traceback", get_path_traceback_, 2); //0: parent traceback, 1: min-cost traceback, 2: gradient descent
        
        private_nh.param("debug", debug_, true);
        private_nh.param("publish_plan", publish_plan_, true);
        private_nh.param("publish_potential", publish_potential_, true);
        private_nh.param("publish_scale", publish_scale_, 100);

        neighbors.clear();
        
        if (con8_)
        {
            neighbors.clear();
            std::vector<int> nb_8con{nx_, nx_+1, 1, -nx_+1, -nx_, -nx_-1, -1, nx_-1};
            for (int nb: nb_8con)
                neighbors.push_back(nb);
        }
        else
        {
            neighbors.clear();
            std::vector<int> nb_4con{nx_, 1, -nx_, -1};
            for (int nb: nb_4con)
                neighbors.push_back(nb);
        }
        
        switch(distance_metric_){
            case 0:{
                cost_calc_ = new L1CostCalculator(nx_, ny_, ns_, neutral_cost_);
                break;
            }
            case 1:{
                cost_calc_ = new L2CostCalculator(nx_, ny_, ns_, neutral_cost_);
                break;
            }
            case 2:{
                cost_calc_ = new OctileCostCalculator(nx_, ny_, ns_, neutral_cost_);
                break;
            }
            case 3:{
                cost_calc_ = new ChebyshevCostCalculator(nx_, ny_, ns_, neutral_cost_);
                break;
            }
        }

        switch(get_path_traceback_){
            case 0:{
                path_traceback_ = new ParentPathTraceback();
                
                break;
            }
            case 1:{
                path_traceback_ = new MinCostPathTraceback();
                break;
            }
            case 2:{
                path_traceback_ = new GradientPathTraceback();
                break;
            }
        }

        path_traceback_-> setSize(nx_, ny_);

        bool allow_obs_plan_through = false;

        max_time = 0;
        max_nodes = 0;
        max_pathlen = 0;

        initialized_ = true;
    }
    else
    {
        ROS_WARN("This planner is already initialized... passing");
    }
}

bool AStar2::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

bool AStar2::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_)
    {
        ROS_ERROR("Initialize before calling makePlan");
        return false;
    }

    plan.clear(); 
    
    //reset all data structures
    reset();
    //initialize f-cost, g-cost and parents
    g_costs.resize(ns_);
    parents.resize(ns_);
    f_costs.resize(ns_);
    std::fill(f_costs.begin(), f_costs.end(), MAX_COST);
    std::fill(g_costs.begin(), g_costs.end(), MAX_COST); //initialize every g-cost with highest possible cost

    //Check frames are equal: until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame_) {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }
    if (start.header.frame_id != global_frame_) {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
    }

    //Check start and goal positions are within bounds
    double start_x_w = start.pose.position.x; double start_y_w = start.pose.position.y;
    double goal_x_w = goal.pose.position.x; double goal_y_w = goal.pose.position.y;
    unsigned int start_x_, start_y_, goal_x_, goal_y_;
    if (!costmap_->worldToMap(start_x_w, start_y_w, start_x_, start_y_)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if (!costmap_->worldToMap(goal_x_w, goal_y_w, goal_x_, goal_y_)) {
        ROS_WARN_THROTTLE(1.0,"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    start_x = start_x_; start_y = start_y_;
    goal_x = goal_x_; goal_y = goal_y_;

    start_idx = mapToIndex(start_x, start_y);
    goal_idx = mapToIndex(goal_x, goal_y);

    cost_calc_->setStartGoal(start_x, start_y, goal_x, goal_y);

    clearRobotCell(start_x, start_y); //clear the starting cell within the costmap because we know it can't be an obstacle
    setCostmap(costmap_ros_);

    g_costs[start_idx] = 0;    //set gcost of start cell as 0
    openList.push_back(Cell(start_idx, 0)); 
    std::fill(parents.begin(), parents.end(), -1); //parent of start cell and all unexplored cells is -1

    f_costs[start_idx] = g_costs[start_idx] + cost_calc_->calcHCost_g(start_idx); 
    
    int cycles = 0;
    int max_cycles = nx_ * ny_ * 2;
    int current_idx;
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double ttrack;
    
    while (!openList.empty() && cycles++ < max_cycles) 
    {
        std::pop_heap(openList.begin(), openList.end(), greater()); //For minheap: swap first element(lowest) with last, make the rest a heap.
        Cell current_cell = openList.back();
        openList.pop_back(); 

        current_idx = current_cell.idx;

        if (current_idx == goal_idx )
        {
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            std::vector<std::pair<float, float> > path;
            if(path_traceback_->getPath(parents, g_costs, start_x, start_y, goal_x, goal_y, path))
            {
                ros::Time plan_time = ros::Time::now();
                for (int i = path.size() -1; i>=0; i--) 
                {
                    std::pair<float, float> point = path[i];
                    
                    double world_x, world_y;
                    mapToWorld(world_x, world_y, point.first, point.second);

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
                    plan.push_back(pose);
                }
                break;
            }
            else
            {
                ROS_ERROR("Unable to trace back path!");
                break;
            }
            
        }
        visited.insert(current_idx);

        std::vector<int> neighbor_indices = getNeighbors(current_idx); //Get free neighbors
        for (int nb_idx: neighbor_indices){ 
            addToOpenList(current_idx , nb_idx);
        }
    }


    if (debug_){
        ROS_INFO("Time taken: %.5f , No. of loops: %d", ttrack, cycles);
        int plan_len = plan.size();
        std::cout << "current path length: " << plan_len << std::endl;
        MaxTime(ttrack);
        MaxNodes();
        maxpathlength(plan_len);
    }

    if(publish_plan_){
        publishPlan(plan);
    }

    if(publish_potential_){
        publishPotential(g_costs);
    }

    return !plan.empty();
}

void AStar2::reset()
{
    openList.clear(); 
    visited.clear();
    f_costs.clear();
    g_costs.clear();
    parents.clear();
}

void AStar2::setCostmap(costmap_2d::Costmap2DROS * costmap_ros)
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ros->getCostmap()->getMutex()));
    unsigned char *cmap = costmap_ros->getCostmap()->getCharMap();

    for (int i= 0; i < ns_; i++, cmap++)
    {
        cmlocal[i] = costmap_2d::LETHAL_OBSTACLE;
        int v = *cmap; 
        if (v < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) //if not obstacle
        {
            v = neutral_cost_ + cost_factor_ * v;  //assign neutral cost to current cell
            if (v >= costmap_2d::LETHAL_OBSTACLE)                   //if > forbidden region (254)
                v = costmap_2d::LETHAL_OBSTACLE - 1;                  //make it an obstacle (253) or > forbidden
            cmlocal[i] = v;                             //assign to local cost array
        }
        else if (v == costmap_2d::NO_INFORMATION && allow_unknown_) //if unknown and unknowns are allowed
        {
            v = costmap_2d::NO_INFORMATION;                          //assign unknown cost (255)
            cmlocal[i] = v;                          
        }
    }
}

void AStar2::maxpathlength(const int len_plan)
{
    if (max_pathlen < len_plan)
        max_pathlen = len_plan;
    std::cout << "Maximum path length recorded: " << max_pathlen << std::endl;    
}

void AStar2::MaxNodes()
{
    int nodes = 0;
    for (int i= 0; i < ns_; i++)
    {
        if (g_costs[i] < 1e10)
            nodes++;
    }
    if (max_nodes < nodes)
        max_nodes = nodes;
    std::cout << "Maximum nodes expanded: " << max_nodes << std::endl;
}

void AStar2::MaxTime(double ttrack)
{   
    if (ttrack > max_time)
    {
        max_time = ttrack;
    }
    ROS_INFO("Max time taken: %f", max_time);
}

bool AStar2::isFree(int &idx)
{
  int cost = costState(idx);
  int obs_cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE * obs_cost_ratio; 
  obs_cost = obs_cost * cost_factor_ + neutral_cost_;
  bool check = allow_obs_plan_through ? (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) : (cost < obs_cost); //check if cost is free (< costmap_2d::INSCRIBED_INFLATED_OBSTACLE) or unknown (< obs_cost)
  check = allow_unknown_ ? (check || cost == costmap_2d::NO_INFORMATION) : check; 

  return check;
}

void AStar2::addToOpenList(int &current_idx, int &nb_idx)
{
    if(!isFree(nb_idx))
        return;

    if (visited.count(nb_idx))
        return;

    updateVertex(current_idx, nb_idx);

}

int AStar2::costState(const int &idx)
{
    if (idx <= 0 || idx >= ns_ )
        return costmap_2d::LETHAL_OBSTACLE;
    return cmlocal[idx];
}

void AStar2::updateVertex(int &current_idx, int &nb_idx)
{
    float old_cost = g_costs[nb_idx];
    float tg_cost = g_costs[current_idx] +  cost_calc_->calcDist(current_idx, nb_idx, costState(nb_idx) );

    if (tg_cost < old_cost)
    {
        g_costs[nb_idx] = tg_cost; 
        f_costs[nb_idx] = g_costs[nb_idx] + cost_calc_->calcHCost_g(nb_idx);
        parents[nb_idx] = current_idx; 
        openList.push_back(Cell(nb_idx, f_costs[nb_idx]) ); 
        std::push_heap(openList.begin(), openList.end(), greater()); //sorts last element into the minheap
        // std::push_heap(openList.begin(), openList.end(), lesser()); //sorts last element into the maxheap
    }
}

void AStar2::saveCostmapProperties(costmap_2d::Costmap2D* costmap)
{
    nx_ = costmap_->getSizeInCellsX();
    ny_ = costmap_->getSizeInCellsY();
    ox_ = costmap_->getOriginX(); 
    oy_ = costmap_->getOriginY(); 
    res_ = costmap_->getResolution();
    ns_ = nx_ * ny_;    
    cmlocal.resize(ns_);
}

void AStar2::clearRobotCell(unsigned int mx, unsigned int my){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE); //set the associated costs in the cost map to be free
}

void AStar2::mapToWorld(double &wx, double &wy, const int &mx, const int &my)
{
    wx = (mx * res_);
    wx = ox_ + wx;
    wy = (my * res_);
    wy = oy_ + wy;
}

int AStar2::mapToIndex(const int &mx, const int &my)
{
    return mx + (my * nx_) ;
}

void AStar2::indexToMap(const int &idx, int &mx, int &my)
{
    my = idx/nx_; //floor
    mx = idx - my * nx_;
}

// geometry_msgs::PoseStamped AStar2::indexToPose(const int idx, ros::Time plan_time)
// {
//     int mx; int my; //map coordinates
//     double wx; double wy; //world coordinates
//     indexToMap(idx, mx, my);
//     mapToWorld(wx, wy, mx, my);
//     geometry_msgs::PoseStamped pose;
//     pose.header.stamp = plan_time;
//     pose.header.frame_id = global_frame_;
//     pose.pose.position.x = wx;
//     pose.pose.position.y = wy;
//     pose.pose.position.z = 0.0;
//     pose.pose.orientation.x = 0.0;
//     pose.pose.orientation.y = 0.0;
//     pose.pose.orientation.z = 0.0;
//     pose.pose.orientation.w = 1.0;
//     return pose;
// }

std::vector<int> AStar2::getNeighbors(const int &current_idx)
{
    std::vector<int> neighbor_indices;
    for (int nb : neighbors)
    {
        int nb_to_be_added = current_idx + nb;
        neighbor_indices.push_back(nb_to_be_added);
    }
    return neighbor_indices;
}

bool AStar2::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {

    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
}

void AStar2::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
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

void AStar2::publishPotential(std::vector<double> cost_array)
{
    nav_msgs::OccupancyGrid grid;

    // Publish Whole Grid
    grid.header.frame_id = global_frame_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = res_;

    grid.info.width = nx_;
    grid.info.height = ny_;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - res_ / 2;
    grid.info.origin.position.y = wy - res_ / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx_ * ny_);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = cost_array[i];
        if (potential < MAX_COST) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (cost_array[i] >= MAX_COST) {
            grid.data[i] = -1;
        } else
            grid.data[i] = cost_array[i] * publish_scale_ / max;
    }

    potential_pub_.publish(grid);
}

};
