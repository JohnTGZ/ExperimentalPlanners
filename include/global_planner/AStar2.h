#ifndef _ASTAR2_H
#define _ASTAR2_H

#include <cstdlib> //TODO: necessary?
// #include <cmath>   //sqrt, abs
#include <string.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <iostream>

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>

#include <cost_calculator/CostCalculator.h>
#include <cost_calculator/L1CostCalculator.h>
#include <cost_calculator/L2CostCalculator.h>
#include <cost_calculator/OctileCostCalculator.h>
#include <cost_calculator/ChebyshevCostCalculator.h>

#include <path_traceback/PathTraceback.h>
#include <path_traceback/ParentPathTraceback.h>
#include <path_traceback/MinCostPathTraceback.h>
#include <path_traceback/GradientPathTraceback.h>

#define MAX_COST 1.0e10 // the cost assigned to unassigned cells /obstacles

namespace global_planner
{

    class Cell 
    {
        public:
            Cell(int a, int b): idx(a), f_cost(b) {}
            int idx;
            float f_cost;
    };

    // operator overload for min heap
    struct greater {
            bool operator()(const Cell& a, const Cell& b) const {
                return a.f_cost > b.f_cost;
            }
    };

    // operator overload for max heap
    struct lesser {
            bool operator()(const Cell& a, const Cell& b) const {
                return a.f_cost < b.f_cost;
            }
    };

    class AStar2 : public nav_core::BaseGlobalPlanner
    {
    public:
        /** constructors **/
        AStar2();
        ~AStar2();
        AStar2(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        AStar2(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
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
        void mapToWorld(double &wx, double &wy, const int &mx, const int &my);
        int mapToIndex(const int &mx, const int &my);
        void indexToMap(const int &idx, int &mx, int &my);
        geometry_msgs::PoseStamped indexToPose(const int idx, ros::Time plan_time);
        bool isFree(int &idx);

        /** neighbor expansion methods **/
        void updateVertex(int &current_idx, int &nb_idx);
        void addToOpenList(int &current_idx, int &nb_idx);
        std::vector<int> getNeighbors(const int &current_idx);
        std::vector<int> neighbors;
        
        /** cost calculations **/
        int costState(const int &idx);
        CostCalculator* cost_calc_; 
        PathTraceback* path_traceback_; 


        /** time **/
        int counter;
        double max_time;
        int max_nodes;
        int max_pathlen;
        void MaxTime(double ttrack);
        void MaxNodes();
        void maxpathlength(const int len_plan);

        /** params **/
        bool initialized_; 
        bool allow_unknown_;
        int neutral_cost_;
        double planner_window_x_, planner_window_y_, default_tolerance_;
        double cost_factor_;
        bool allow_obs_plan_through;
        double obs_cost_ratio;
        int get_path_traceback_;
        bool debug_;
        bool con8_;
        int distance_metric_;

        /** costmap properties **/
        costmap_2d::Costmap2DROS *costmap_ros_;  // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html
        costmap_2d::Costmap2D *costmap_;         // http://docs.ros.org/jade/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

        std::vector<int> cmlocal;               // array of costs based on costmap 
        std::string global_frame_;               // goal and start coordinate frame
        int nx_; int ny_;                        // max no. of cells in x and y direction,
        int ns_;                                 // total no. of cells
        double ox_; double oy_;                  // origin of cosmap
        double res_;                             // resolution of costmap

        /** plan properties **/
        int start_idx; int goal_idx;             //start and goal index
        int start_x, start_y;
        int goal_x, goal_y;

        /**publishers and services **/
        bool publish_potential_; 
        int publish_scale_;
        bool publish_plan_;
        ros::Publisher plan_pub_;
        ros::Publisher potential_pub_;
        ros::ServiceServer make_plan_srv_;

        //Data structures
        std::vector<Cell> openList;               //open list containing cell idx and f cost
        std::unordered_set<int> visited; 
        std::vector<double> g_costs;  
        std::vector<int> parents;
        std::vector<double> f_costs;
    };
};
#endif