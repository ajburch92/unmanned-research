cmake_minimum_required(VERSION 2.8.3)
project(virtual_infrastructure_pkg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
)

find_package(OpenCV REQUIRED) 

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

#install python scripts
#catkin_install_python(PROGRAMS bin/yaml_info_pub
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

## Generate messages in the 'msg' folder # Add messages for object position
#add_message_files(
#   FILES
#  vehicle_pose.msg
#  goal_pose.msg
#)

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
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
  LIBRARIES virtual_infrastructure_pkg
  CATKIN_DEPENDS cv_bridge image_transport message_runtime roscpp rospy sensor_msgs std_msgs geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories( include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a library

add_library( ${PROJECT_NAME} src/Object.cpp src/Odom.cpp) # src/Planner.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS virtual_infrastructure_pkg virtual_infrastructure_pkg
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#################
## Executables ##
#################


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(virtual_infrastructure_pkg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(virtual_infrastructure_pkg_node src/virtual_infrastructure_pkg_node.cpp)
add_executable(ground_station_rgb_node src/ground_station_rgb.cpp)
add_executable(ground_station_therm_node src/ground_station_therm.cpp)
add_executable(vehicle_odom src/Odom.cpp)
add_executable(vehicle_planner src/Planner.cpp)
add_executable(astar_planner src/Astar.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ground_station_rgb_node 
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

 target_link_libraries(ground_station_therm_node 
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

 target_link_libraries( vehicle_odom
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

 target_link_libraries( vehicle_planner
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

 target_link_libraries ( astar_planner
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
 )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(virtual_infrastructure_pkg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ground_station_rgb_node src/ground_station_rgb.cpp)
add_dependencies(ground_station_therm_node src/ground_station_therm.cpp)
add_dependencies(vehicle_odom src/Odom.cpp)
add_dependencies(vehicle_planner src/Planner.cpp)
add_dependencies(astar_planner src/Astar.cpp)