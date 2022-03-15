cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
    tf
    custom_layers
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
    tf
    custom_layers
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/orientation_filter.cpp
  src/planner_core.cpp
  
  src/global_planner/Dijkstra2.cpp
  src/global_planner/AStar2.cpp
  src/global_planner/ThetaStarBasic1.cpp
  src/global_planner/ThetaStarBasic2.cpp
  src/global_planner/DStarLite1.cpp

  src/cost_calculator/ChebyshevCostCalculator.cpp
  src/cost_calculator/L1CostCalculator.cpp
  src/cost_calculator/L2CostCalculator.cpp
  src/cost_calculator/OctileCostCalculator.cpp
  
  src/path_traceback/GradientPathTraceback.cpp
  src/path_traceback/MinCostPathTraceback.cpp
  src/path_traceback/ParentPathTraceback.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(planner
  src/plan_node.cpp
)

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
