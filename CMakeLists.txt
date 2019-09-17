cmake_minimum_required(VERSION 2.8.3)
project(cluster_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_filters
  nodelet
  pcl_ros
  pluginlib
  roscpp
  sensor_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  hungarian_solver
)

generate_dynamic_reconfigure_options(
  cfg/ClusterTracker.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cluster_tracker
  CATKIN_DEPENDS dynamic_reconfigure message_filters nodelet pcl_ros pluginlib roscpp sensor_msgs vision_msgs tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(cluster_tracker src/cluster_tracker.cpp)
target_link_libraries(cluster_tracker ${catkin_LIBRARIES})
class_loader_hide_library_symbols(cluster_tracker)

add_executable(cluster_tracker_node src/cluster_tracker_node.cpp)
target_link_libraries(cluster_tracker_node ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS cluster_tracker_node cluster_tracker
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)