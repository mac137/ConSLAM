cmake_minimum_required(VERSION 3.1)
project(velodyne_pointcloud)

set(CMAKE_BUILD_TYPE "Release")

# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    roscpp
    roslib
    sensor_msgs
    tf2_ros
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
    diagnostic_updater
        pcl_conversions
)

find_package(PCL 1.2 REQUIRED)# PATHS ${PCL_DIR} NO_DEFAULT_PATH)# PATHS "/home/maciej/git/pcl/pcl-pcl-1.10.0/build/")# PATHS "/usr/local/bin") # change to find_package(PCL 1.2 REQUIRED) if previous state is required
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             roslint)

find_package(Boost COMPONENTS signals)
find_package(Eigen3 REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TransformNode.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata data_containers)

#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )

add_subdirectory(src/lib)
add_subdirectory(src/conversions)

add_executable(cpp_correct_velo_timestamps src/correct_timestamps.cpp include/velodyne_pointcloud/pointXYZIRT.h)
add_dependencies(cpp_correct_velo_timestamps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_correct_velo_timestamps ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)



install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(PROGRAMS 
	scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        

catkin_install_python(PROGRAMS
	scripts/velodyne_timestamp_corrector.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

roslint_cpp()

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
