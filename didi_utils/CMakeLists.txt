cmake_minimum_required(VERSION 2.8.3)
project(didi_challenge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
## Enable compiler optimizations
add_definitions(-O3)
add_definitions(-ffast-math)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  perception_msgs
# message_generation
)

## System dependencies are found with CMake's conventions
set(OpenCV2_DIR ${OpenCV_DIR})
set(OpenCV_DIR "/usr/local/opencv3/share/OpenCV/")
#set(OpenCV_DIR "/usr/share/OpenCV/")
find_package(OpenCV REQUIRED)
set(OpenCV_DIR ${OpenCV2_DIR})

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
#  INCLUDE_DIRS include
#  LIBRARIES didi_challenge
#  CATKIN_DEPENDS cuda_cv_bridge image_transport pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(cloud_filter_lib
  src/${PROJECT_NAME}/cloud_filter.cpp
)
target_link_libraries(cloud_filter_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library(smart_tracker_lib
  src/${PROJECT_NAME}/smart_tracker.cpp
  src/${PROJECT_NAME}/multi_smart_tracker.cpp
)

add_dependencies(smart_tracker_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(smart_tracker_lib
  ${OpenCV_LIBS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(laser_birdview src/laser_birdview.cpp)

add_executable(interpolation src/interpolation.cpp)

add_executable(smart_tracking src/smart_tracking.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(interpolation ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(laser_birdview
  cloud_filter_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(interpolation
  ${catkin_LIBRARIES}
)

target_link_libraries(smart_tracking
  smart_tracker_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
