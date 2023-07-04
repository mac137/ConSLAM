cmake_minimum_required(VERSION 3.2)
project(pp_launcher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

#############
## Install ##
#############

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
