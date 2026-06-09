# lidar_perception
# LiDAR Perception Pipeline

A high-performance, real-time, C++ ROS 2 LiDAR perception pipeline designed for autonomous vehicle applications. 

This package ingests raw 3D PointCloud data, isolates obstacles, groups them into discrete objects.

---

## 🚀 Architecture & Data Flow

To ensure strict real-time performance (>10Hz), the entire pipeline is implemented using **ROS 2 Components** (`rclcpp_components`). All nodes are loaded into a single `MultiThreadedExecutor` container. This allows PointCloud data to be passed between nodes using **Zero-Copy Intra-Process Communication**, avoiding expensive network serialization and heap allocations.

### Pipeline Steps:

1. **ROI Crop Filter** (`/sensing/lidar/concatenated/pointcloud` ➔ `/lidar/points_roi`)
   - **Method**: `pcl::CropBox`
   - **Purpose**: Drops points far outside the vehicle's operational area (e.g., beyond 50m) and points behind the ego-vehicle. This drastically reduces the number of points downstream nodes have to process.

2. **Voxel Grid Filter** (`/lidar/points_roi` ➔ `/lidar/points_downsampled`)
   - **Method**: `pcl::VoxelGrid`
   - **Purpose**: Uniformly downsamples the point cloud (default 0.1m leaf size). This equalizes point density (near vs far) and guarantees an upper bound on computational load for the clustering algorithm.

3. **Ground Removal** (`/lidar/points_downsampled` ➔ `/lidar/points_obstacle` & `/lidar/points_ground`)
   - **Method**: `pcl::SACSegmentation` (RANSAC Plane Fitting)
   - **Purpose**: Extracts the dominant ground plane (the road). Non-ground points are published as the obstacle cloud. RANSAC provides a highly robust method for flat or gently sloping roads.

4. **Euclidean Clustering** (`/lidar/points_obstacle` ➔ `/lidar/clusters`)
   - **Method**: `pcl::EuclideanClusterExtraction` via `pcl::search::KdTree`
   - **Purpose**: Groups isolated obstacle points into distinct object clusters based on their Euclidean distance. 

---

## 🛠️ Build & Launch Instructions

### Building
This package requires `PCL`, `Eigen3`, and `tf2`.
```bash
# Navigate to your workspace
cd ~/Fperception_ws

# Build only the lidar_perception package
colcon build --packages-select lidar_perception --symlink-install

# Source the workspace
source install/setup.bash
```

### Launching
To launch the full pipeline within the optimized C++ component container:
```bash
ros2 launch lidar_perception lidar_perception_pipeline.launch.py
```


---

## 🎛️ Parameter Tuning Guide

Parameters are currently defined as defaults within the C++ constructors. If you need to tune the pipeline for different environments, look at the following key variables in the source files:

### 1. ROI Filter (`src/roi_crop_filter_node.cpp`)
*   `x_min` / `x_max`: Forward/Backward distance. Default: `0.0` to `50.0` meters.
*   `y_min` / `y_max`: Lateral distance. Default: `-20.0` to `20.0` meters.
*   **Tuning advice**: Keep this as small as your application allows. The less space you process, the faster the pipeline runs.

### 2. Voxel Grid (`src/voxel_grid_filter_node.cpp`)
*   `leaf_size`: Size of the voxel cubes. Default: `0.1` (10 cm).
*   **Tuning advice**: If clustering is too slow, increase to `0.15` or `0.2`. If you are missing small objects like pedestrians, decrease to `0.05`.

### 3. Ground Removal (`src/ground_removal_node.cpp`)
*   `distance_threshold`: How thick the ground plane is allowed to be. Default: `0.2` meters.
*   `max_iterations`: RANSAC iterations. Default: `100`.
*   **Tuning advice**: If curbs are being classified as ground, decrease the distance threshold. If the road is bumpy and pieces of the road are showing up as obstacles, increase it.

### 4. Euclidean Clustering (`src/euclidean_cluster_node.cpp`)
*   `cluster_tolerance`: Maximum distance between points to be considered the same object. Default: `0.5` meters.
*   `min_cluster_size` / `max_cluster_size`: Point count limits. Default: `10` to `15000`.
*   **Tuning advice**: If a single car is being split into two boxes, increase `cluster_tolerance`. If two cars parked next to each other are merged into one giant box, decrease it.
---

## 🎥 RViz Visualization
Open RViz2 and add the following displays:
1. **PointCloud2**: Topic `/lidar/clusters`
   * Set **Color Transformer** to `Intensity`. This assigns a unique color to each discrete object cluster.
2. **PointCloud2**: Topic `/lidar/points_obstacle`
   * Displays the raw obstacle points fed into the clusterer.

---

number of points 

[component_container_mt-1] [INFO] [1780548760.350213723] [pipeline_monitor_node]: 
[component_container_mt-1] --- LiDAR Perception Pipeline Status ---
[component_container_mt-1] 1. Original Point Cloud     : 168598 points
[component_container_mt-1] 2. After ROI Filtering      : 112146 points
[component_container_mt-1] 3. After Voxel Downsampling : 35186 points
[component_container_mt-1] 4. After Ground Removal     : 11950 points (obstacle)
[component_container_mt-1] 5. After Clustering         : 11642 points (in clusters)
