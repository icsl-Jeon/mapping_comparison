cmake_minimum_required(VERSION 2.8.3)
project(mapping_comparison)


# ROS 
find_package(catkin REQUIRED roscpp
        voxblox_ros octomap_ros octomap_server
 octomap_msgs 
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

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

add_executable(mapping_comparison src/vsf_mapping_comparison.cpp )
target_link_libraries(mapping_comparison ${catkin_LIBRARIES} )

add_executable(pcd_to_pointcloud src/pcd_to_pointcloud.cpp )
target_link_libraries(pcd_to_pointcloud ${catkin_LIBRARIES} )

