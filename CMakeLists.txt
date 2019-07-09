cmake_minimum_required(VERSION 2.8.3)
project(mav_mapping_comp)


# ROS 
find_package(catkin REQUIRED roscpp 
 octomap_msgs 
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

#OCTOMAP 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#EDT 
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES mav_mapping_comp
)

#Build 
add_executable(octomap2edf_node src/octomap2edf_main.cpp)
target_link_libraries(octomap2edf_node ${catkin_LIBRARIES})

