#ifndef _DSTARLITE1_H
#define _DSTARLITE1_H

#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <utility>
#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>

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
#define COST_NEUTRAL 50      // Set this to "open space" value
#define COST_FACTOR 0.8      // Used for translating costs in NavFn::setCostmap()
#define COSTTYPE unsigned char 

#define MAX_COST 1e10

namespace global_planner {

  typedef std::vector<float> State;

  const float TOL = 0.0001;
  
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
  
  class state_hash {
  public:
    size_t operator()(const IState &s) const {
      return s.x + 16384*s.y;
    }
  };

  inline IState stateToIState(const State& s) {
    if (s.size() == 2) {
      return IState(s[0], s[1]);
    }
    if (s.size() == 3) {
      return IState(s[0], s[1], s[2]);
    }
    return IState();
  }

  inline State IStateToState(const IState& is) {
    State s = {float(is.x), float(is.y), float(is.z)};
    return s;
  }
  
  typedef std::unordered_map<IState, IState, state_hash, std::equal_to<IState> > Fhash;
  typedef std::unordered_map<IState, float, state_hash, std::equal_to<IState> > Chash;
  typedef std::unordered_set<IState, state_hash, std::equal_to<IState> > StateSet;

  class ValuePair {
  public:
    ValuePair() {}
    ValuePair(float f, float s) {
      first = f;
      second = s;
    }
    float first;
    float second;
    bool operator > (const ValuePair &v) const {
      if (first - TOL > v.first) return true;
      if (first < v.first - TOL) return false;
      return second > v.second;
    }
    bool operator <= (const ValuePair &v) const {
      if (first < v.first) return true;
      if (first > v.first) return false;
      return second < v.second + TOL;
    }
    bool operator < (const ValuePair &v) const {
      if (first + TOL < v.first) return true;
      if (first - TOL > v.first) return false;
      return second < v.second;
    }
  };
  
  class DQPair {
  public:
    DQPair() {}
    DQPair(ValuePair f, IState s) {
      first = f;
      second = s;
    }
    ValuePair first;
    IState second;
    bool operator > (const DQPair &q) const {
      return first > q.first;
    }
    bool operator < (const DQPair &q) const {
      return first < q.first;
    }
  };

  class DStarLite1 : public nav_core::BaseGlobalPlanner {
  public:
    DStarLite1();
    DStarLite1(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
    DStarLite1(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    
    /** init methods **/
    void reset();
    void init();
    void setGoal(const State &goal);
    void setStart(const State &start);
    bool firsttime_;
    bool initialized_;
    bool allow_obs_plan_through;
    

    /** rosparams **/
    int num_search;
    double allotted;
    bool allow_unknown;
    double obs_cost_ratio;
    bool validate_first;
    int step;
    bool allow_smooth;
    int cheap_cost;
    
    bool publish_plan_;
    bool publish_potential_;
    int publish_scale_;

    /** Expansion methods **/
    
    std::vector<State> calcPath();
    bool computeShortestPath();
    ValuePair calcKey(const IState &s);
    void UpdateVertex(IState t);
    std::vector<IState> simpleNeighbors(IState s);
    std::vector<IState> neighbors(IState s);
    int equivalent(int v);
    void replan();
    bool replan_;
    float km_;
    bool recurring;
    IState start, goal, last_start, last_goal;

    /**publishers and services **/
    ros::Publisher plan_pub_;
    ros::Publisher potential_pub_;
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishPotential();
    void MaxNodes();
    void MaxTime(double ttrack);
    int max_nodes;
    float max_time;
    

  protected:
    /** ulist methods **/
    bool inUList(IState s);
    void removeUList(IState s);
    void insertUList(ValuePair total, IState s);

    /** cost calculations **/
    float getRhs(const IState &s);
    float getg(const IState &s);
    int costState(const IState &s);
    int costState(unsigned int x, unsigned int y);
    float distToStart(const IState &s);
    float calcHCost(const IState &s);
    float DistState(IState p, IState t);
    float calcDist(IState p, IState t);
    float DistConnNeighbour(IState p, IState t);

    /** Checks **/
    float smaller(float x, float y);
    bool different(float x, float y);
    bool isFree(const IState &s);
    bool isFree(int x, int y);
    bool isCheap(int x, int y);
    bool isCheaper(int x, int y, float thres);
    bool outOfBound(int x, int y);
    bool isValid(DQPair qp);
    
    /** Path extraction **/
    std::vector<State> extractPath_parents();
    std::vector<State> extractPath_min();
    std::vector<IState> smooth(std::vector<IState> orig);
    bool lineOfSight(const IState &s, const IState &n);

    /** costmap methods **/
    void saveCostmapProperties();
    void setCostmap();
    
    std::string global_frame_;
    int nx, ny;
    int ns;
    float map_res;
    // unsigned char *cmlocal;
    std::vector<int> cmlocal;
    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;

    /** data structures **/
    std::priority_queue<DQPair, std::vector<DQPair>, std::greater<DQPair>> ulist;
    std::unordered_map<IState, IState, state_hash, std::equal_to<IState> > from; //parent
    std::unordered_map<IState, float, state_hash, std::equal_to<IState> > gval;
    std::unordered_map<IState, float, state_hash, std::equal_to<IState> > rhs;
    std::unordered_set<IState, state_hash, std::equal_to<IState> > ulisthash;
  };

}

#endif
