cmake_minimum_required(VERSION 2.8.3)
project(detect)

find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    sensor_msgs
    geometry_msgs
    cv_bridge
    image_transport
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS  roscpp
                    sensor_msgs
                    geometry_msgs
                    cv_bridge
                    image_transport
                    message_filters
                    tf2 
                    tf2_ros
    DEPENDS OpenCV
)

# Directories
include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
)

# Node Executable
add_executable(detect_node src/detect_node.cpp)
add_executable(detect_estimation src/detect_with_estimation.cpp)

# Link Libraries
target_link_libraries(detect_node 
    ${catkin_LIBRARIES} 
 #   ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(detect_estimation 
    ${catkin_LIBRARIES} 
 #   ${OpenCV_INCLUDE_DIRS}
)