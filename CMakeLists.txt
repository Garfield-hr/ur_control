cmake_minimum_required(VERSION 3.0.2)
project(my_ur_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_perception
        pluginlib
        geometric_shapes
        pcl_ros
        pcl_conversions
        rosbag
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
        doc/interactivity/include
        )


catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        CATKIN_DEPENDS
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        DEPENDS
        EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
        ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
)

add_executable(my_ur_ik src/myUrIk.cpp src/ur_ik.h src/ur_ik.cpp)
target_link_libraries(my_ur_ik ${catkin_LIBRARIES} ${Boost_LIBRARIES})


