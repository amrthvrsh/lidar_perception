cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find PCL and Eigen
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree features)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraDetection.msg"
  "msg/CameraDetectionArray.msg"
  "msg/BoundingBox3D.msg"
  "msg/BoundingBox3DArray.msg"
  DEPENDENCIES std_msgs
)

# Set up link properties
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# ── TASK 1: ROI Cropping Node ─────────────────────────────────────────────
add_library(roi_crop_filter_node SHARED src/roi_crop_filter_node.cpp)
target_include_directories(roi_crop_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(roi_crop_filter_node ${dependencies})
target_link_libraries(roi_crop_filter_node ${PCL_LIBRARIES})
rclcpp_components_register_node(roi_crop_filter_node PLUGIN "lidar_perception::RoiCropFilterNode" EXECUTABLE roi_crop_filter_node_exe)

# ── TASK 2: Voxel Grid Filter Node ────────────────────────────────────────
add_library(voxel_grid_filter_node SHARED src/voxel_grid_filter_node.cpp)
target_include_directories(voxel_grid_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(voxel_grid_filter_node ${dependencies})
target_link_libraries(voxel_grid_filter_node ${PCL_LIBRARIES})
rclcpp_components_register_node(voxel_grid_filter_node PLUGIN "lidar_perception::VoxelGridFilterNode" EXECUTABLE voxel_grid_filter_node_exe)

# ── TASK 3: Ground Removal Node ───────────────────────────────────────────
add_library(ground_removal_node SHARED src/ground_removal_node.cpp)
target_include_directories(ground_removal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ground_removal_node ${dependencies})
target_link_libraries(ground_removal_node ${PCL_LIBRARIES})
rclcpp_components_register_node(ground_removal_node PLUGIN "lidar_perception::GroundRemovalNode" EXECUTABLE ground_removal_node_exe)

# ── TASK 4: Euclidean Cluster Node ────────────────────────────────────────
add_library(euclidean_cluster_node SHARED src/euclidean_cluster_node.cpp)
target_include_directories(euclidean_cluster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(euclidean_cluster_node ${dependencies})
target_link_libraries(euclidean_cluster_node ${PCL_LIBRARIES})
rclcpp_components_register_node(euclidean_cluster_node PLUGIN "lidar_perception::EuclideanClusterNode" EXECUTABLE euclidean_cluster_node_exe)

# ── TASK 5: Pipeline Monitor Node ─────────────────────────────────────────
add_library(pipeline_monitor_node SHARED src/pipeline_monitor_node.cpp)
target_include_directories(pipeline_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pipeline_monitor_node ${dependencies})
rclcpp_components_register_node(pipeline_monitor_node PLUGIN "lidar_perception::PipelineMonitorNode" EXECUTABLE pipeline_monitor_node_exe)



install(DIRECTORY include/
  DESTINATION include
)



install(PROGRAMS
  scripts/camera_detection_node.py
  scripts/lidar_projection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(TARGETS
  roi_crop_filter_node
  voxel_grid_filter_node
  ground_removal_node
  euclidean_cluster_node
  pipeline_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch & config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
