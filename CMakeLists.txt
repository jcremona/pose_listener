cmake_minimum_required(VERSION 2.8.12)
project(pose_listener)
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_compile_options(-std=c++11)

# Modify cmake module path if new .cmake files are required
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # std_msgs
  # tf
  nav_msgs
  #sensor_msgs
  #geometry_msgs
  #eigen_conversions
  #tf_conversions
  #random_numbers
  #message_generation
  #nodelet
  #image_transport
  #cv_bridge
  #message_filters
  #pcl_conversions
  #pcl_ros
  #std_srvs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(SuiteSparse REQUIRED)

##################
## ROS messages ##
##################
#add_message_files(
#  FILES

#  FeatureMeasurement.msg
#  CameraMeasurement.msg
#  TrackingInfo.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES msckf_vio image_processor
  CATKIN_DEPENDS
    roscpp nav_msgs #std_msgs tf nav_msgs sensor_msgs geometry_msgs
    #    eigen_conversions tf_conversions random_numbers message_runtime
    #image_transport cv_bridge message_filters pcl_conversions
    #pcl_ros std_srvs
    #DEPENDS Boost EIGEN3 OpenCV SUITESPARSE
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}
  #${Boost_INCLUDE_DIR}
  #${OpenCV_INCLUDE_DIRS}
  #${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(pose_listener src/pose_listener.cpp src/message_to_log.cpp)
target_link_libraries(pose_listener ${catkin_LIBRARIES})

