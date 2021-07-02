cmake_minimum_required(VERSION 3.0.2)
project(my_ur_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS

        roscpp rospy std_msgs genmsg
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

include_directories( ~/XIMEA/package/samples/_libs)
include(/home/liangxiao/XIMEA/samples/_libs/_cmake/add_xiapi.cmake)
include(/home/liangxiao/XIMEA/samples/_libs/_cmake/add_opencv.cmake)

include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(THIS_PACKAGE_INCLUDE_DIRS
        doc/interactivity/include
        )


catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        CATKIN_DEPENDS
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

add_executable(vf  src/VisualFeedback.cpp src/CamROI.cpp src/CamROI.h src/xiApiPlusOcv.cpp
        src/xiApiPlusOcv.hpp)
target_link_libraries(vf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})


