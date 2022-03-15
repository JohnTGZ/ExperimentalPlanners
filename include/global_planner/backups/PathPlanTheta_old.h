#ifndef _PATHPLANTHETA_H
#define _PATHPLANTHETA_H


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
#define COST_NEUTRAL 1      // Set this to "open space" value
#define COST_FACTOR 0.8      // Used for translating costs in NavFn::setCostmap()
#define COSTTYPE unsigned char 

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


  class PathPlanTheta : public nav_core::BaseGlobalPlanner
  {
  public:
    PathPlanTheta();
    ~PathPlanTheta();
    // void initialize(ros::NodeHandle nh, int xs, int ys, float map_res);
    PathPlanTheta(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
    PathPlanTheta(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool initialized_;
    bool allow_unknown;

    int step;
    int num_search;
    float map_res;
    int ns;
    double allotted;
    IState start, goal;
    int cheap_cost;
    double obs_cost_ratio;

    bool allow_obs_plan_through;

    void setGoal(const State& goal);
    void setStart(const State& start);
    void setup_map(int nx, int ny);
    void setCostmap(costmap_2d::Costmap2DROS *costmap_ros);
    std::vector<State> calcPath();
    std::vector<State> extractPath();

  protected:

    int nx, ny;
    COSTTYPE *costarr;
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;

    float heuristic(const IState& s);
    float distToGoal(const IState& s);
    bool isGoalReached(const IState& s);
    bool isFree(const IState& s);
    bool isFree(int x, int y);
    bool isCheap(int x, int y);
    bool isCheaper(int x, int y, float thres);
    float TravelCost(IState p, IState t);
    float MoveCost(IState p, IState t);
    void ComputeCost(IState p, IState t);
    void insert(float total, IState t);
    void remove(IState s);
    bool isValid(TQPair qp);
    std::vector<IState> neighbors(IState s);
    void UpdateVertex(IState p, IState t);
    bool lineOfSight(const IState& s, const IState& n);
    int costState(const IState& s);
    float DistState(IState p, IState t);
    float sqr(float x); 
    bool outOfBound(int x, int y);

    StateSet visited;
    void reset();
    void init();
    bool CalculatePath();
    std::priority_queue<TQPair, std::vector<TQPair>, std::greater<TQPair>> open; //container adaptor that provides constant time lookup of the largest (by default) element, at the expense of logarithmic insertion and extraction. 
    Fhash from; //std::unordered_map<IState, IState, state_hash, std::equal_to<IState> >
    Chash cost; //std::unordered_map<IState, float, state_hash, std::equal_to<IState> >
    StateSet openhash;  //openhash serves as a sort of closed list, std::unordered_set<IState, state_hash, std::equal_to<IState> >

    IState stateToIState(const State& s);
    State IStateToState(const IState& is);
  };
}

#endif
