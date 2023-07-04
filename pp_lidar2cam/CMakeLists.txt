cmake_minimum_required(VERSION 3.2)
project(pp_lidar2cam_projector)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_runtime
        cv_bridge
        roscpp
  rospy
  std_msgs
  sensor_msgs
        rosbag
        image_geometry
        image_transport
        image_view
        pcl_conversions
        pcl_ros
        tf_conversions #added 200324 for tf to eigen transofrmation conversions
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
        include ${catkin_INCLUDE_DIRS} include/pp_lidar2cam_projector
)

add_executable(lidar_cams_synchroniser src/lidar_cams_synchroniser.cpp)
target_link_libraries(lidar_cams_synchroniser ${catkin_LIBRARIES})

add_executable(bag2bags src/bag2bags.cpp)
target_link_libraries(bag2bags ${catkin_LIBRARIES})

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
        scripts/pp_img_undistorter.py
        scripts/projector2.py
        scripts/projector4dense.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#############
## Testing ##
#############
