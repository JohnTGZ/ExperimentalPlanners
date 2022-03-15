#include <cstdlib> //abs
#include <cmath>   //sqrt
#include <string.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <unordered_set>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_global_planner.h>

#define COST_UNKNOWN_ROS 255 // 255 is unknown cost
#define COST_OBS 254         // 254 for forbidden regions
#define COST_OBS_ROS 253     // ROS values of 253 are obstacles
#define COSTTYPE unsigned char 


#ifndef THETABASICSIMPLE_H
#define THETABASICSIMPLE_H

namespace global_planner
{

    class Cell 
    {
        public:
            Cell(int a, int b): idx(a), f_cost(b) {}
            // bool operator<(const Cell& other)
            // {
            //     return f_cost < other.f_cost;
            // }
            int idx;
            float f_cost;
    };

    // operator overload to compare costs
    struct greater1 {
            bool operator()(const Cell& a, const Cell& b) const {
                return a.f_cost > b.f_cost;
            }
    };

    class ThetaBasicSimple : public nav_core::BaseGlobalPlanner
    {
    public:
        /** constructors **/
        ThetaBasicSimple();
        ThetaBasicSimple(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        ThetaBasicSimple(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
        /** overriden methods from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &plan);
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, 
                        double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

        /** publishers and services**/
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishPotential(std::vector<double> cost_array);
        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

        /** initialization methods **/
        void reset();
        void setCostmap(costmap_2d::Costmap2DROS * costmap_ros);
        void saveCostmapProperties(costmap_2d::Costmap2D* costmap);
        void clearRobotCell(unsigned int mx, unsigned int my);
        
        /** conversions **/
        void mapToWorld(double &wx, double &wy, const int mx, const int my);
        int mapToIndex(int mx, int my);
        void indexToMap(const int idx, int &mx, int &my);
        geometry_msgs::PoseStamped indexToPose(const int idx, ros::Time plan_time);
        bool isCheap(int mx, int my, int thres);
        bool isFree(int idx);

        /** neighbor expansion methods **/
        std::vector<int> getNeighbors(const int current_idx);
        void addToOpenList(int current_idx, int nb_idx);
        void updateVertex(int current_idx, int nb_idx);
        bool lineOfSight(int idx1, int idx2);
        void removeFromOpenhash(int nb_idx);
        
        int costState(int idx);
        float calculatedist(int current_idx, int nb_idx);
        float calculatehcost(int current_idx);

        int counter;
        double total_avg_time;
        void AverageTime(double ttrack, int cycles);
        void MaxTime(double ttrack);
        

        /** params **/
        bool initialized_; 
        bool allow_unknown_;
        bool publish_potential_; int publish_scale_;
        int neutral_cost_;
        double planner_window_x_, planner_window_y_, default_tolerance_;
        int cheapthreshold;
        double cost_factor_;
        bool allow_obs_plan_through;
        double obs_cost_ratio;

        /** costmap properties **/
        costmap_2d::Costmap2DROS *costmap_ros_;        // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html
        costmap_2d::Costmap2D *costmap_;        // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

        COSTTYPE * costmap_costs;           // array of costs based on costmap 
        std::string global_frame_;              // goal and start coordinate frame
        int nx_; int ny_;                       // max no. of cells in x and y direction,
        int ns_;                                // total no. of cells
        double ox_; double oy_;                 // origin of cosmap
        double res_;                            // resolution of costmap

        /** plan properties **/
        int start_idx; int goal_idx;            //start and goal index

        /**publishers and services **/
        ros::Publisher plan_pub_;
        ros::Publisher potential_pub_;
        ros::ServiceServer make_plan_srv_;

        //Data structures
        std::vector<Cell> openList;    //open list containing cell idx and f cost
        std::unordered_set<int> visited; 
        std::unordered_set<int> openhash; 
        std::vector<double> g_costs; 
        std::vector<int> parents;

    };
};
#endif

