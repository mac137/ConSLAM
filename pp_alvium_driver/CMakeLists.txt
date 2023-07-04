cmake_minimum_required(VERSION 3.2)
project(pp_alvium_driver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#set(ALVIUM_LIBRARY1 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/x86_64bit/libVimbaCPP.so)
#set(ALVIUM_LIBRARY2 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/x86_64bit/libVimbaC.so)
message(STATUS "The host processor is ${CMAKE_HOST_SYSTEM_PROCESSOR}")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
	set(ALVIUM_LIBRARY1 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/arm_64bit/libVimbaCPP.so)
	set(ALVIUM_LIBRARY2 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/arm_64bit/libVimbaC.so)
	message(STATUS "Vimba for aarch64 will be used")
else (NOT ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
	set(ALVIUM_LIBRARY1 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/x86_64bit/libVimbaCPP.so)
	set(ALVIUM_LIBRARY2 ${CMAKE_CURRENT_SOURCE_DIR}/DynamicLib/x86_64bit/libVimbaC.so)
	message(STATUS "Vimba for x will be used")
endif ()


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        message_runtime
        cv_bridge
        roscpp
        rospy
        std_msgs
        sensor_msgs
#        rosbag
        image_geometry
        image_transport
        image_view

        camera_info_manager
        diagnostic_updater
        dynamic_reconfigure
        message_filters
        polled_camera
        nodelet
)

##Get architecture
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#include(TargetArchitecture)
#target_architecture(ARCH)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_python_setup()

#add dynamic reconfigure api
#generate_dynamic_reconfigure_options(
#        cfg/AvtVimbaCamera.cfg
        #        cfg/AvtVimbaCameraStereo.cfg
#)

#add_service_files(
#  FILES
#        PPLaunchSensor.srv
#)
#
#generate_messages(
#  DEPENDENCIES
#  #std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

#catkin_package(
#  CATKIN_DEPENDS
#  #message_runtime
#  #roscpp
#  rospy
#  #std_msgs
#  sensor_msgs
#
#)

catkin_package(
        INCLUDE_DIRS #include
        CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_geometry image_transport roscpp sensor_msgs std_msgs polled_camera

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
        include
        include ${catkin_INCLUDE_DIRS}
#        include/${PROJECT_NAME}
)


#add_executable(mono_camera_node
#  ${PROJECT_NAME}
#        src/nodes/mono_camera_node.cpp
#        src/mono_driver.cpp
#        src/mono_camera.cpp
#        src/avt_vimba_camera.cpp
#        src/frame_observer.cpp
#)

#add_dependencies(mono_camera_node
#        ${PROJECT_NAME}_gencfg
#        #${libvimba_EXPORTED_TARGETS}
#)


#add_dependencies_and_linkings()
#add_dependencies(mono_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_library(avt_camera_nodelets
#        src/nodes/mono_camera_nodelet.cpp
#        src/nodes/stereo_camera_nodelet.cpp
#        src/mono_camera.cpp
#        src/stereo_camera.cpp
#        src/avt_vimba_camera.cpp
#        src/frame_observer.cpp
#        )

#target_link_libraries(
#        mono_camera_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#        ${ALVIUM_LIBRARY1}
#        ${ALVIUM_LIBRARY2}
#)

#add_dependencies(
#  ${PROJECT_NAME}
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${catkin_EXPORTED_TARGETS}
#)



#############
## Install ##
#############

catkin_install_python(PROGRAMS
        scripts/ros_alvium_rgb.py
        scripts/ros_alvium_nir.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

#install(TARGETS mono_camera_node avt_camera_nodelets
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark executables and/or libraries for installation
#install(
#  TARGETS
#  ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#############
## Testing ##
#############
