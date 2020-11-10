cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(object_position src/object_position.cpp)
add_executable(trajectory_path src/trajectory_path.cpp)
add_executable(moveit_execution src/moveit_execution.cpp)
add_executable(zivid_camera_temp src/zivid_camera_temp.cpp)
target_link_libraries(object_position ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(trajectory_path ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(moveit_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(zivid_camera_temp ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIRS})

#############
## Install ##
#############
install(TARGETS object_position DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS trajectory_path DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS moveit_execution DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS zivid_camera_temp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

