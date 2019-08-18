cmake_minimum_required(VERSION 2.8.3)
project(gps_tran)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(gps_lib
	src/gps.cpp
)

target_include_directories(gps_lib PUBLIC
	include/
	)

############
# BINARIES #
############
cs_add_executable(gps_tran_node src/main.cpp)


catkin_package( CATKIN_DEPENDS )
include_directories( ${catkin_INCLUDE_DIRS} )

target_link_libraries(gps_tran_node gps_lib)
target_link_libraries(gps_tran_node ${catkin_LIBRARIES})
add_dependencies(gps_tran_node ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()



