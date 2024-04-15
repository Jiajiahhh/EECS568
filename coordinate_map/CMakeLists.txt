cmake_minimum_required(VERSION 2.8.3)
project(coordinate_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs message_filters cv_bridge image_transport 
compressed_image_transport tf compressed_depth_image_transport  geometry_msgs PCL)
set(PCL_DIR /home/wmc/eecs568_ws/src/pcl/build)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)



catkin_package(

)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(coordinate_map src/coordinate_map.cpp)
target_link_libraries(coordinate_map
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


