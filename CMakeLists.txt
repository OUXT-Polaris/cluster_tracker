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
  quaternion_operation
  jsk_rviz_plugins
)

generate_dynamic_reconfigure_options(
  cfg/ClusterTracker.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cluster_tracker
  CATKIN_DEPENDS dynamic_reconfigure message_filters nodelet pcl_ros pluginlib roscpp sensor_msgs vision_msgs tf2_ros tf2_geometry_msgs jsk_rviz_plugins
#  DEPENDS system_lib
)

find_package(PCL REQUIRED COMPONENTS core features filters tracking segmentation)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(cluster_tracker src/cluster_tracker.cpp src/tracker_instance.cpp src/tracking_manager.cpp)
target_link_libraries(cluster_tracker ${catkin_LIBRARIES} ${PCL_TRACKING_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(cluster_tracker)
add_dependencies(cluster_tracker ${catkin_EXPORTED_TARGETS})

add_executable(cluster_tracker_node src/cluster_tracker_node.cpp)
target_link_libraries(cluster_tracker_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cluster_tracker_node ${catkin_EXPORTED_TARGETS})

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