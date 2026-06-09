#include "lidar_perception/euclidean_cluster.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cstring>
#include <cmath>

namespace lidar_perception
{

EuclideanClusterNode::EuclideanClusterNode(const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_node", options), tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
  double tolerance = this->declare_parameter("cluster_tolerance", 0.5);
  int min_size = this->declare_parameter("min_cluster_size", 10);
  int max_size = this->declare_parameter("max_cluster_size", 15000);

  ec_.setClusterTolerance(tolerance);
  ec_.setMinClusterSize(min_size);
  ec_.setMaxClusterSize(max_size);
  ec_.setSearchMethod(tree_);

  std::string input_topic = this->declare_parameter("input_topic", "/lidar/points_obstacle");
  std::string output_topic = this->declare_parameter("cluster_topic", "/lidar/clusters");

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(5);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, sensor_qos,
    std::bind(&EuclideanClusterNode::pointcloud_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic, sensor_qos);

  RCLCPP_INFO(this->get_logger(), "Euclidean Cluster initialized.\n  Tolerance: %.2f\n  Size: [%d, %d]",
    tolerance, min_size, max_size);
}

void EuclideanClusterNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto start_time = std::chrono::steady_clock::now();

  if (msg->data.empty() || msg->width == 0) return;

  // --- Safe XYZ extraction: reads field offsets by name, works with any point type ---
  int32_t x_off = -1, y_off = -1, z_off = -1;
  for (const auto & field : msg->fields) {
    if      (field.name == "x") x_off = static_cast<int32_t>(field.offset);
    else if (field.name == "y") y_off = static_cast<int32_t>(field.offset);
    else if (field.name == "z") z_off = static_cast<int32_t>(field.offset);
  }
  if (x_off < 0 || y_off < 0 || z_off < 0) return;

  const uint8_t * src = msg->data.data();
  const uint32_t step = msg->point_step;
  const uint32_t n    = msg->width * msg->height;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(n);
  for (uint32_t i = 0; i < n; ++i) {
    const uint8_t * pt = src + i * step;
    float px, py, pz;
    std::memcpy(&px, pt + x_off, sizeof(float));
    std::memcpy(&py, pt + y_off, sizeof(float));
    std::memcpy(&pz, pt + z_off, sizeof(float));
    if (std::isfinite(px) && std::isfinite(py) && std::isfinite(pz)) {
      cloud->points.emplace_back();
      cloud->points.back().x = px;
      cloud->points.back().y = py;
      cloud->points.back().z = pz;
    }
  }
  cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;

  if (cloud->empty()) return;

  tree_->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  
  // Note: For adaptive tolerance, one would iterate over points and group them manually, 
  // but for performance and standard PCL usage, we use fixed tolerance here first.
  ec_.setInputCloud(cloud);
  ec_.extract(cluster_indices);

  // Re-pack into a single cloud, with 'intensity' serving as the cluster ID
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZI>());
  int cluster_id = 1;
  for (const auto& indices : cluster_indices) {
    for (const auto& idx : indices.indices) {
      cloud_clusters->points.emplace_back();
      cloud_clusters->points.back().x = cloud->points[idx].x;
      cloud_clusters->points.back().y = cloud->points[idx].y;
      cloud_clusters->points.back().z = cloud->points[idx].z;
      cloud_clusters->points.back().intensity = static_cast<float>(cluster_id);
    }
    cluster_id++;
  }

  cloud_clusters->width = cloud_clusters->points.size();
  cloud_clusters->height = 1;
  cloud_clusters->is_dense = true;

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_clusters, output_msg);
  output_msg.header = msg->header;

  pub_->publish(output_msg);

  auto end_time = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  RCLCPP_DEBUG(this->get_logger(), "Clustering: %zu pts -> %d clusters [%.2f ms]",
    cloud->size(), cluster_id - 1, dt_ms);
}

}  // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::EuclideanClusterNode)
