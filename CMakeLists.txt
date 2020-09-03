cmake_minimum_required(VERSION 2.8.12)
project(pose_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp nav_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_listener src/pose_listener.cpp src/message_to_log.cpp)
target_link_libraries(pose_listener ${catkin_LIBRARIES})
