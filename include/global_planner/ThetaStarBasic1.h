#ifndef _THETASTARBASIC1_H
#define _THETASTARBASIC1_H

#include <ros/ros.h>
#include <ros/console.h>

#include <cmath>
#include <math.h> //INFINITY
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <set>
#include <utility>
#include <iostream>
#include <chrono>
#include <functional>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_global_planner.h>

#define COST_UNKNOWN_ROS 255 // 255 is unknown cost
#define COST_OBS 254         // 254 for forbidden regions
#define COST_OBS_ROS 253     // ROS values of 253 are obstacles
// #define COST_NEUTRAL 1      // Set this to "open space" value
// #define COST_FACTOR 0.8      // Used for translating costs in NavFn::setCostmap()
#define COSTTYPE unsigned char 
#define MAX_COST 1.0e10 // the cost assigned to unassigned cells 

namespace global_planner
{
  typedef std::vector<float> State;

  class IState {
    public: 
      IState() {}
      IState(int x, int y): x(x), y(y) {}
      IState(int x, int y, float z): x(x), y(y), z(z) {}
      int x;
      int y;
      float z;
      bool operator == (const IState &s2) const {
        return ((x == s2.x) && (y == s2.y));
      }
      bool operator != (const IState &s2) const {
        return ((x != s2.x) || (y != s2.y));
      }
  };

  class TQPair
  {
  public:
    TQPair() {}
    TQPair(float f, IState s)
    {
      first = f;
      second = s;
    }

    float first;
    IState second;
    bool operator > (const TQPair &q) const
    {
      return first > q.first;
    }
    bool operator < (const TQPair &q) const
    {
      return first < q.first;
    }
  };

  class state_hash {
  public:
    size_t operator()(const IState &s) const {
      return s.x + 16384*s.y;
    }
  };
  
  typedef std::unordered_map<IState, IState, state_hash, std::equal_to<IState> > Fhash;
  typedef std::unordered_map<IState, float, state_hash, std::equal_to<IState> > Chash;
  typedef std::unordered_set<IState, state_hash, std::equal_to<IState> > StateSet;
  //Unordered map is an associative container that contains key-value pairs with unique keys. Search, insertion, and removal of elements have average constant-time complexity. 
  //Internally, the elements are not sorted in any particular order, but organized into buckets. Which bucket an element is placed into depends entirely on the hash of its key. This allows fast access to individual elements, since once the hash is computed, it refers to the exact bucket the element is placed into. 

  class ThetaStarBasic1 : public nav_core::BaseGlobalPlanner
  {
  public:
    ThetaStarBasic1();
    ThetaStarBasic1(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
    ThetaStarBasic1(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    /** init methods **/

    /** @brief assigns size of map to class variables
     */ 
    void setup_map(int nx, int ny);

    /** @brief initialize costarr with costmap_ros and user defined cost ratios 
     */ 
    void setCostmap(costmap_2d::Costmap2DROS *costmap_ros);

    /** @brief assign goal coordinates to class variable and call init
     */ 
    void setGoal(const State& goal);

    /** @brief assign start coordinates to class variable 
     */ 
    void setStart(const State& start);
    
    /** expansion **/

    /** @brief Main expansion loop
     *  @returns returns path in map coordinates
     */ 
    std::vector<State> calcPath();

    /** @brief Extracts path from stored array (call only after calcPath() has been called)
     *  @returns returns path in map coordinates
     */ 
    std::vector<State> extractPath();

    /**publishers and services **/
    ros::Publisher plan_pub_;
    ros::Publisher potential_pub_;
    ros::ServiceServer make_plan_srv_;
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishPotential();
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
    void MaxNodes();
    int max_nodes;
    void MaxTime(double ttrack);
    double max_time;

    /** rosparams **/
    int num_search;  
    double allotted;
    bool allow_unknown;
    double obs_cost_ratio;
    bool allow_obs_plan_through;
    int step;
    int cheap_cost;
    bool publish_plan_;
    bool publish_potential_; 
    int publish_scale_;
    int COST_NEUTRAL;
    double COST_FACTOR;

    /** global variables **/
    bool initialized_;
    IState start, goal;

  protected:

    /** costmap properties **/
    std::string global_frame_;
    int nx_, ny_;
    int ns_;
    float map_res;
    COSTTYPE *costarr;
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;

    /** initialization **/

    /** @brief clears openhash, from, cost, visited, and open. Then sets allow_obs_plan_through = false
     */ 
    void reset();

    /** @brief calls reset, then initializes start cell.
     */ 
    void init();

    /** @brief expansion 
    */
    bool CalculatePath();

    /** @brief expansion 
    */
    void UpdateVertex(IState p, IState t);

    /** @brief expansion 
    */
    bool lineOfSight(const IState& s, const IState& n);

    /** @brief check if goal is reached 
    */
    bool isGoalReached(const IState& s);

    /** @brief check cost of a cell is within free space
     *  @return true if cell is within free space
    */
    bool isFree(const IState& s);

    /** @brief check cost of a cell is within free space
     *  @return true if cell is within free space
    */
    bool isFree(int x, int y);


    /** @brief check cost of a cell is cheaper a user defined threshold
     *  @return true if cell is cheaper and within map bounds
    */
    bool isCheaper(int x, int y, float thres);

    /** @brief checks if the cell has already been expanded i.e. added to openhash
     *  @param qp the cell to be expanded
     */
    bool isValid(TQPair qp);

    /** @brief Returns cost of s according to costmap if s is not out of bounds
     *  @param s cell to be checked
     */ 
    int costState(const IState& s);
    
    /** @brief check if map coordinates x and y are out of bounds
     *  @param x x coordinate of map
     *  @param y y coordinate of map
     *  @return true if out of map
     */ 
    bool outOfBound(int x, int y);

    /** @brief calculate g-cost of cell t, it calls DistState which uses euclidean distance
     *  @param p parent cell
     *  @param t target cell 
     *  @return g-cost of cell t
     */
    float MoveCost(IState p, IState t);

    /** @brief calculate euclidean distance from cell p to cell t
     *  @return euclidean distance between p and t
     */
    float DistState(IState p, IState t);

    /** @brief checks line of sight and computes g-cost of target cell t
     *  @param p parent cell
     *  @param t target cell 
     */
    void ComputeCost(IState p, IState t);

    /** @brief calculate h_cost of cell s, it will call distToGoal 
      */
    float heuristic(const IState& s);

    /** @brief returns euclidean distance from cell s to goal cell 
      */
    float distToGoal(const IState& s);

    /** @brief helper method to square a value 
    */
    float sqr(float x); 

    /** @brief adds cell coordinates and f-cost to the open and openhash.
     *  @param total the total f_cost
     *  @param t the cell's map coordinates
     */
    void insert(float total, IState t); 

    /** @brief remove cell from openhash if found in openhash
     *  @param s cell to be removed
     */
    void remove(IState s);

    /** @brief convert from State to IState 
    */
    IState stateToIState(const State& s);

    /** @brief convert from IState to State 
    */
    State IStateToState(const IState& is); 
    
    /** data structures **/
    std::priority_queue<TQPair, std::vector<TQPair>, std::greater<TQPair>> open; //container adaptor that provides constant time lookup of the largest (by default) element, at the expense of logarithmic insertion and extraction. 
    std::unordered_map<IState, IState, state_hash, std::equal_to<IState> > from; //data structure of parents
    std::unordered_map<IState, float, state_hash, std::equal_to<IState> > cost; //data structure of movement cost (or g-cost)
    std::unordered_set<IState, state_hash, std::equal_to<IState> > openhash; //
    std::unordered_set<IState, state_hash, std::equal_to<IState> > visited;  //data structure of visted cells
    std::vector<IState> neighbors(IState s);
  };
}

#endif
