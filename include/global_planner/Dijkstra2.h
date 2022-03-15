#ifndef _DIJKSTRA2_H
#define _DIJKSTRA2_H

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

#define COSTTYPE unsigned char 
#define MAX_COST 1.0e10 // the cost assigned to unassigned cells 

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

    class Dijkstra2 : public nav_core::BaseGlobalPlanner
    {
    public:
        /** constructors **/
        Dijkstra2();
        Dijkstra2(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        Dijkstra2(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
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
        std::vector<int> getNeighbors(const int &current_idx);
        void addToOpenList(int &current_idx, int &nb_idx);
        void updateVertex(int &current_idx, int &nb_idx);
        
        /** cost calculations **/
        int costState(const int &idx);
        float calculatedist(const int &current_idx, const int &nb_idx);

        /** time **/
        int counter;
        double max_time;
        int max_nodes;
        void MaxTime(double ttrack);
        void MaxNodes();

        /** gradient path **/
        bool extractGradientPath(std::vector<double> potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path);
        float gradCell(std::vector<double> potential, int n);
        float *gradx_, *grady_; /**< gradient arrays, size of potential array */
        float pathStep_; /**< step size for following gradient */

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
        bool get_grad_path_;

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
        ros::Publisher plan_pub_;
        ros::Publisher potential_pub_;
        ros::ServiceServer make_plan_srv_;

        //Data structures
        std::vector<Cell> openList;               //open list containing cell idx and f cost
        std::unordered_set<int> visited; 
        std::vector<double> g_costs;  //TODO: change this to a pointer too?
        std::vector<int> parents;
    };
};
#endif