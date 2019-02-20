cmake_minimum_required(VERSION 2.8.3)
project(quad_racer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
catkin_package(
	CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quad_racer
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(quad_racer src/quad_racer.cpp)
target_link_libraries(quad_racer ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(image_listener src/image_listener.cpp)
target_link_libraries(image_listener ${OpenCV_LIBS} ${catkin_LIBRARIES})
