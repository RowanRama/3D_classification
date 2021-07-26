cmake_minimum_required(VERSION 3.0.2)
project(classification)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
)

find_package(Open3D REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_extract_pcl src/convert_svr.cpp)
target_link_libraries(${PROJECT_NAME}_extract_pcl ${catkin_LIBRARIES})