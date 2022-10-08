### CMakeLists.txt ---
##

project(constrained_RMSE)

cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)

find_package(CGAL REQUIRED)
find_package(TBB REQUIRED tbb)


## #############################################################################
## Sources
## #############################################################################

set(${PROJECT_NAME}_SOURCES main.cpp )


## #############################################################################
##
## #############################################################################

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})




if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${TBB_IMPORTED_TARGETS} CGAL)
endif (WIN32)

if (UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${TBB_IMPORTED_TARGETS} CGAL stdc++fs)
endif (UNIX)

######################################################################
### CMakeLists.txt ends here
