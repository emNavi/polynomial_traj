cmake_minimum_required(VERSION 3.0.2)
project(polynomial_traj)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  PvayCommand.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES polynomial_traj
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/test_node.cpp src/polynomial_traj.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_executable(pub_traj_node src/pub_traj.cpp src/polynomial_traj.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pub_traj_node
  ${catkin_LIBRARIES}
)
