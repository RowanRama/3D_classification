cmake_minimum_required(VERSION 3.0.2)
project(classification)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  bbox.msg
  bbox_2d.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

catkin_install_python(PROGRAMS script/detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

add_executable(extract_pcl src/convert_svr.cpp)
add_executable(path_planner src/pathplanner.cpp)

add_dependencies(extract_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(extract_pcl ${catkin_LIBRARIES})
target_link_libraries(path_planner ${catkin_LIBRARIES})