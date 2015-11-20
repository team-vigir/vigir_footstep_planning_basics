cmake_minimum_required(VERSION 2.8.3)
project(vigir_footstep_planning_lib)

set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy rqt_gui rqt_gui_py actionlib_msgs actionlib std_msgs nav_msgs visualization_msgs tf pcl_conversions vigir_generic_params vigir_pluginlib vigir_footstep_planning_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_footstep_planning_lib
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_py actionlib_msgs actionlib std_msgs nav_msgs visualization_msgs tf pcl_conversions vigir_generic_params vigir_pluginlib vigir_footstep_planning_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/helper.h
  include/${PROJECT_NAME}/math.h
  include/${PROJECT_NAME}/modeling/state.h
  include/${PROJECT_NAME}/plugins/collision_check_plugin.h
  include/${PROJECT_NAME}/plugins/collision_check_grid_map_plugin.h
  include/${PROJECT_NAME}/plugins/heuristic_plugin.h
  include/${PROJECT_NAME}/plugins/robot_model_plugin.h
  include/${PROJECT_NAME}/plugins/post_process_plugin.h
  include/${PROJECT_NAME}/plugins/reachability_plugin.h
  include/${PROJECT_NAME}/plugins/step_cost_estimator_plugin.h
  include/${PROJECT_NAME}/plugins/step_plan_msg_plugin.h
  include/${PROJECT_NAME}/plugins/terrain_model_plugin.h
  include/${PROJECT_NAME}/threading/job.h
  include/${PROJECT_NAME}/threading/threading_manager.h
  include/${PROJECT_NAME}/threading/queue.h
  include/${PROJECT_NAME}/threading/worker.h
  include/${PROJECT_NAME}/visualization/footstep_planning_vis_node.h
  include/${PROJECT_NAME}/visualization/footstep_planning_vis.h
)

set(SOURCES
  src/helper.cpp
  src/math.cpp
  src/modeling/state.cpp
  src/plugins/collision_check_plugin.cpp
  src/plugins/collision_check_grid_map_plugin.cpp
  src/plugins/heuristic_plugin.cpp
  src/plugins/post_process_plugin.cpp
  src/plugins/reachability_plugin.cpp
  src/plugins/robot_model_plugin.cpp
  src/plugins/step_cost_estimator_plugin.cpp
  src/plugins/step_plan_msg_plugin.cpp
  src/plugins/terrain_model_plugin.cpp
  src/visualization/footstep_planning_vis.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(footstep_planning_vis_node src/visualization/footstep_planning_vis_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(footstep_planning_vis_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} footstep_planning_vis_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vigir_footstep_planner_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
