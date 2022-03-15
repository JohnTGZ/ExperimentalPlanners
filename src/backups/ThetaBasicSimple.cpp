#include "global_planner/ThetaBasicSimple.h"
#include <pluginlib/class_list_macros.h>

#define COST_HIGH 1.0e10 // the cost assigned to unassigned cells 

PLUGINLIB_EXPORT_CLASS(global_planner::ThetaBasicSimple, nav_core::BaseGlobalPlanner) //register this planner as a BaseGlobalPlanner plugin
 
namespace global_planner
{

ThetaBasicSimple::ThetaBasicSimple(): costmap_(NULL), initialized_(false), allow_unknown_(true) {}

ThetaBasicSimple::ThetaBasicSimple(std::string name, costmap_2d::Costmap2DROS *costmap_ros): costmap_(NULL), initialized_(false), allow_unknown_(true)
{
    // costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros);
}

ThetaBasicSimple::ThetaBasicSimple(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame): costmap_(NULL), initialized_(false), allow_unknown_(true)
{
    initialize(name, costmap, global_frame);
} 

void ThetaBasicSimple::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    costmap_ros_ = costmap_ros;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void ThetaBasicSimple::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
    if (!initialized_)
    {
        ros::NodeHandle private_nh("~/" + name);

        global_frame_ = global_frame;
        costmap_ = costmap;      
        saveCostmapProperties(costmap_);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        private_nh.param("neutral_cost", neutral_cost_, 1);
        private_nh.param("cost_factor", cost_factor_, 0.8);
        private_nh.param("obstacle_cost_ratio",obs_cost_ratio, 1.0);
        private_nh.param("cheap_threshold", cheapthreshold, 150);

        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);

        private_nh.param("default_tolerance", default_tolerance_, 0.0); 
        private_nh.param("publish_potential", publish_potential_, true);
        private_nh.param("publish_scale", publish_scale_, 100);

        bool allow_obs_plan_through = false;

        counter = 0;
        total_avg_time = 0;
        make_plan_srv_ = private_nh.advertiseService("make_plan", &ThetaBasicSimple::makePlanService, this);

        initialized_ = true;
    }
    else
    {
        ROS_WARN("This planner is already initialized... passing");
    }
}

bool ThetaBasicSimple::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

void ThetaBasicSimple::reset()
{
    openList.clear(); 
    visited.clear();
    openhash.clear();
    g_costs.clear();
    parents.clear();
}


void ThetaBasicSimple::setCostmap(costmap_2d::Costmap2DROS * costmap_ros)
{
    COSTTYPE *cm = costmap_costs;
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
            v = neutral_cost_ + cost_factor_ * v;  //assign neutral cost to current cell
            if (v >= COST_OBS)                   //if > forbidden region (254)
                v = COST_OBS - 1;                  //make it an obstacle (253) or > forbidden
            *cm = v;                             //assign to local cost array
        }
        else if (v == COST_UNKNOWN_ROS && allow_unknown_) //if unknown and unknowns are allowed
        {
            v = COST_UNKNOWN_ROS;                          //assign unknown cost (255)
            *cm = v;                          
        }
        }
    }
}

bool ThetaBasicSimple::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_)
    {
        ROS_ERROR("Initialize before calling makePlan");
        return false;
    }

    //MODULE 0: Clear all pre-existing data structures
    plan.clear(); 
    
    reset();
    g_costs.resize(ns_);
    parents.resize(ns_);

    setCostmap(costmap_ros_);

    //MODULE 1: Check frames are equal: until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame_) {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }
    if (start.header.frame_id != global_frame_) {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
    }

    //MODULE 2: Checked start and goal positions are within bounds
    double start_x_w = start.pose.position.x; double start_y_w = start.pose.position.y;
    double goal_x_w = goal.pose.position.x; double goal_y_w = goal.pose.position.y;
    unsigned int start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(start_x_w, start_y_w, start_x, start_y)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if (!costmap_->worldToMap(goal_x_w, goal_y_w, goal_x, goal_y)) {
        ROS_WARN_THROTTLE(1.0,"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    start_idx = mapToIndex(start_x, start_y);
    goal_idx = mapToIndex(goal_x, goal_y);

    //MODULE 3: Set map size variables, size of the g_costs and clear the starting cell
    // clearRobotCell(start_x, start_y); //clear the starting cell within the costmap because we know it can't be an obstacle
    
    g_costs[start_idx] = 0;    //set gcost of start cell as 0
    openList.push_back(Cell(start_idx, 0)); 
    openhash.insert(start_idx);
    std::fill(parents.begin(), parents.end(), -1); //parent of start cell and all unexplored cells is -1
    
    int cycles = 0;
    int max_cycles = nx_ * ny_ * 2;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    //MODUlE 5: start of search
    while (!openList.empty() && cycles++ < max_cycles) //TODO: Add option for cycles
    {
        Cell current_cell = openList[0]; 
        std::pop_heap(openList.begin(), openList.end(), greater1()); //move the cheapest element to the back
        openList.pop_back(); 

        int current_idx = current_cell.idx;

        if (current_idx == goal_idx )
        {

            ros::Time plan_time = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            std::vector<geometry_msgs::PoseStamped> plan_reversed; //reversed plan, we need to reverse this for move_base

            while (parents[current_idx] != -1) //since start cell's parents = -1, it will stop at start_cell
            {
                pose = indexToPose(current_idx, plan_time); //convert index to world coordinates
                plan_reversed.push_back(pose); 
                current_idx = parents[current_idx]; //current_idx is pointing to parent cell
            }

            //add start cell to plan
            pose = indexToPose(current_idx, plan_time);
            plan_reversed.push_back(pose); 

            //reverse the plan
            int len_plan = plan_reversed.size();
            for (int i = len_plan-1; i>=0; --i)
            {
                plan.push_back(plan_reversed[i]);
            }
            publishPlan(plan);

            break;
        }

        if(openhash.find(current_idx) == openhash.end()) //if current cell is not inside openhash then continue
            continue;

        openhash.erase(current_idx);        

        visited.insert(current_idx);

        std::vector<int> neighbor_indices = getNeighbors(current_idx); //Get free neighbors
        for (int nb_idx: neighbor_indices){ 
            addToOpenList(current_idx , nb_idx);
        }
        

    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    ROS_INFO("Time taken: %.5f , No. of loops: %d", ttrack, cycles);

    // AverageTime(ttrack, cycles);
    MaxTime(ttrack);

    // if(publish_potential_){
    //     publishPotential(g_costs);
    // }

    return !plan.empty();
}

void ThetaBasicSimple::AverageTime(double ttrack, int cycles)
{   
    
    //use cumulative moving average
    double itr_avg_time = ttrack; //average?
    total_avg_time = ttrack + counter*total_avg_time;
    total_avg_time = total_avg_time / (counter+1);

    counter += 1;

    ROS_INFO("Culmulative average time taken: %f", total_avg_time);
}

void ThetaBasicSimple::MaxTime(double ttrack)
{   
    if (ttrack > total_avg_time)
    {
        total_avg_time = ttrack;
    }
    counter += 1;
    ROS_INFO("Max time taken: %f", total_avg_time);
}

bool ThetaBasicSimple::isFree(int idx)
{
  int cost = costState(idx);
  int obs_cost = COST_OBS_ROS * obs_cost_ratio; 
  obs_cost = obs_cost * cost_factor_ + neutral_cost_;
  bool check = allow_obs_plan_through ? (cost < COST_OBS_ROS) : (cost < obs_cost); //check if cost is free (< COST_OBS_ROS) or unknown (< obs_cost)
  check = allow_unknown_ ? (check || cost == COST_UNKNOWN_ROS) : check; 

  return check;
}

void ThetaBasicSimple::addToOpenList(int current_idx, int nb_idx)
{
    if(!isFree(nb_idx))
        return;

    if (visited.count(nb_idx))
        return;

    if(openhash.find(nb_idx) == openhash.end())
        g_costs[nb_idx] = INFINITY;

    updateVertex(current_idx, nb_idx);

}

int ThetaBasicSimple::costState(int idx)
{
    if (idx <= 0 || idx >= ns_ )
        return COST_OBS + 1; //if out of bounds, mark as forbidden
    int cost = costmap_costs[idx];
    return cost ;
}



void ThetaBasicSimple::updateVertex(int current_idx, int nb_idx)
{
    float old_cost = g_costs[nb_idx];
    float tg_cost;

    if (lineOfSight(parents[current_idx], nb_idx))
    {
        // float tentative_g_cost = g_costs[parents[current_idx]] + costmap_costs[nb_idx] + neutral_cost_ ; //neutral_Cost is cost of moving to an adjacent cell
        tg_cost = calculatedist(parents[current_idx], nb_idx ) * costState(nb_idx)  ;
        if (g_costs[parents[current_idx]] + tg_cost < g_costs[nb_idx])
        {
            parents[nb_idx] = parents[current_idx];
            g_costs[nb_idx] = g_costs[parents[current_idx]] + tg_cost;  
        }
    }
    else
    {
        // float tentative_g_cost = g_costs[current_idx] + costmap_costs[nb_idx] + neutral_cost_ ; 
        tg_cost =   calculatedist(current_idx, nb_idx ) * costState(nb_idx)  ;
        if (g_costs[current_idx] + tg_cost < g_costs[nb_idx])
        {
            parents[nb_idx] = current_idx; 
            g_costs[nb_idx] = g_costs[current_idx] + tg_cost; 
        }
    }

    if (g_costs[nb_idx] < old_cost)
    {
        if (openhash.find(nb_idx) != openhash.end()) //if nb_idx is in openhash, then remove nb_idx
            removeFromOpenhash(nb_idx);
        
        float f_cost = g_costs[nb_idx] + calculatehcost(nb_idx) * neutral_cost_;  
        openList.push_back(Cell(nb_idx, f_cost) ); 
        openhash.insert(nb_idx);
        std::push_heap(openList.begin(), openList.end(), greater1());

    }


}

void ThetaBasicSimple::removeFromOpenhash(int nb_idx)
{
  auto c = openhash.find(nb_idx);
  if (c == openhash.end())
    return;
  openhash.erase(c);
}


bool ThetaBasicSimple::lineOfSight(int idx1, int idx2)
{
    int sx1, sy1, sx2, sy2;
    int x0, x1, y1, y0;
    indexToMap(idx1, sx1, sy1);
    indexToMap(idx2, sx2, sy2);
    x0 = sx1; y0 = sy1;
    x1 = sx2; y1 = sy2;

    int dx = x1-x0;
    int dy = y1-y0; 
    int f =0;

    if(dy<0){
        dy = -dy;
        sy1 = -1;
    }
    else
        sy1 = 1;

    if(dx<0){
        dx = -dx;
        sx1 = -1;
    }
    else
        sx1 = 1;
    
    if (dx >= dy)
    {
        while (x0 != x1)
        {
            f = f + dy;
            if (f >= dx)
            {
                if ( !isCheap( x0+((sx1-1)/2), y0 + ((sy1-1)/2), cheapthreshold ) ) //isCheap(x,y, thres) is TRUE if the cell is within the map and below the cost threshold
                    return false;
                y0 = y0 + sy1;
                f = f - dx;
            }
            if ( f!=0 && !isCheap(x0+((sx1-1)/2), y0 + ((sy1-1)/2), cheapthreshold ) )
                return false;
            if (dy == 0 && !isCheap(x0+((sx1-1)/2), y0, cheapthreshold) && !isCheap(x0+((sx1-1)/2), y0-1, cheapthreshold ) )
                return false;
            x0 = x0 + sx1;
        }
    }
    else 
    {
        while (y0 != y1)
        {
            f = f + dx;
            if (f >= dy)
            {
                if ( !isCheap(x0 + ((sx1 - 1)/2), y0 + ((sy1-1)/2), cheapthreshold) )
                    return false;
                x0 = x0 + sx1;
                f = f-dy;
            }
            if ( f!=0 && !isCheap(x0+((sx1-1)/2), y0 + ((sy1-1)/2), cheapthreshold) )
                return false;
            if ( dx==0 && !isCheap(x0, y0+((sy1-1)/2), cheapthreshold) && !isCheap(x0-1 , y0 + ((sy1-1)/2), cheapthreshold) )
                return false;
            y0 = y0 + sy1;
        }
    }
    return true;
}


bool ThetaBasicSimple::isCheap(int mx, int my, int thresh)
{
    int idx = mx + my * nx_; 
    if (idx < 0 || idx > ns_) 
        return false;
    // if (!(0 < mx && mx < nx_ && 0 < my && my < ny_)) //do not use index here, something to do wit the conversion btw index and map coordinates affects the accuracy
    //     return false;
    float cost = costmap_costs[idx];
    return (cost < thresh);
}

void ThetaBasicSimple::saveCostmapProperties(costmap_2d::Costmap2D* costmap)
{
    nx_ = costmap_->getSizeInCellsX();
    ny_ = costmap_->getSizeInCellsY();
    ox_ = costmap_->getOriginX(); 
    oy_ = costmap_->getOriginY(); 
    res_ = costmap_->getResolution();
    ns_ = nx_ * ny_;    
    costmap_costs = new COSTTYPE[ns_];
    memset(costmap_costs, 0, ns_ * sizeof(COSTTYPE));
}

void ThetaBasicSimple::clearRobotCell(unsigned int mx, unsigned int my){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE); //set the associated costs in the cost map to be free
}

void ThetaBasicSimple::mapToWorld(double &wx, double &wy, const int mx, const int my)
{
    wx = (mx * res_);
    wx = ox_ + wx;
    wy = (my * res_);
    wy = oy_ + wy;
}

int ThetaBasicSimple::mapToIndex(int mx, int my)
{
    return mx + (my * nx_) ;
}

void ThetaBasicSimple::indexToMap(const int idx, int &mx, int &my)
{
    my = idx/nx_;
    mx = (my * nx_);
    mx = idx - mx;
}

geometry_msgs::PoseStamped ThetaBasicSimple::indexToPose(const int idx, ros::Time plan_time)
{
    int mx; int my; //map coordinates
    double wx; double wy; //world coordinates
    indexToMap(idx, mx, my);
    mapToWorld(wx, wy, mx, my);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    return pose;
}



std::vector<int> ThetaBasicSimple::getNeighbors(const int current_idx)
{
    std::vector<int> neighbors{ nx_, nx_+1, 1, -nx_+1, -nx_, -nx_-1, -1, nx_-1}; //in octile direction (From top cell in clockwise fashion)
    std::vector<int> neighbor_indices;
    //add in octile direction (From top cell in clockwise fashion)
    for (int nb : neighbors)
    {
        int nb_to_be_added = current_idx + nb;
        neighbor_indices.push_back(nb_to_be_added);
    }
    return neighbor_indices;
}

float ThetaBasicSimple::calculatedist(int current_idx, int nb_idx)
{   
    int mx1; int my1;
    int mx2; int my2;
    
    indexToMap(current_idx, mx1, my1);
    indexToMap(nb_idx, mx2, my2);

    float dist = std::sqrt((mx2 - mx1)*(mx2 - mx1)+ (my2 - my1)*(my2 - my1));
    return dist; 
}

float ThetaBasicSimple::calculatehcost(int current_idx)
{   
    int mx1; int my1;
    int mx2; int my2;
    
    indexToMap(current_idx, mx1, my1);
    indexToMap(goal_idx, mx2, my2);

    float hcost = std::sqrt((mx2 - mx1)*(mx2 - mx1)+ (my2 - my1)*(my2 - my1));
    return hcost;
}

bool ThetaBasicSimple::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {

    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
}

void ThetaBasicSimple::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
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

// void ThetaBasicSimple::publishPotential(std::vector<double> cost_array)
// {
//     // int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
//     // double resolution = costmap_->getResolution();
//     nav_msgs::OccupancyGrid grid;

//     // Publish Whole Grid
//     grid.header.frame_id = global_frame_;
//     grid.header.stamp = ros::Time::now();
//     grid.info.resolution = res_;

//     grid.info.width = nx_;
//     grid.info.height = ny_;

//     double wx, wy;
//     costmap_->mapToWorld(0, 0, wx, wy);
//     grid.info.origin.position.x = wx - res_ / 2;
//     grid.info.origin.position.y = wy - res_ / 2;
//     grid.info.origin.position.z = 0.0;
//     grid.info.origin.orientation.w = 1.0;

//     grid.data.resize(nx_ * ny_);

//     float max = 0.0;
//     for (unsigned int i = 0; i < grid.data.size(); i++) {
//         float potential = cost_array[i];
//         if (potential < COST_HIGH) {
//             if (potential > max) {
//                 max = potential;
//             }
//         }
//     }

//     for (unsigned int i = 0; i < grid.data.size(); i++) {
//         if (cost_array[i] >= COST_HIGH) {
//             grid.data[i] = -1;
//         } else
//             grid.data[i] = cost_array[i] * publish_scale_ / max;
//     }
//     potential_pub_.publish(grid);
// }


};
