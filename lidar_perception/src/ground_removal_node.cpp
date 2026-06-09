#include "lidar_perception/ground_removal.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cstring>
#include <cmath>

namespace lidar_perception
{

GroundRemovalNode::GroundRemovalNode(const rclcpp::NodeOptions & options)
: Node("ground_removal_node", options)
{
  double dist_thresh = this->declare_parameter("distance_threshold", 0.2);
  int max_iterations = this->declare_parameter("max_iterations", 100);

  seg_ = std::make_unique<pcl::SACSegmentation<pcl::PointXYZ>>();
  extract_ = std::make_unique<pcl::ExtractIndices<pcl::PointXYZ>>();

  seg_->setOptimizeCoefficients(true);
  seg_->setModelType(pcl::SACMODEL_PLANE);
  seg_->setMethodType(pcl::SAC_RANSAC);
  seg_->setMaxIterations(max_iterations);
  seg_->setDistanceThreshold(dist_thresh);

  std::string input_topic = this->declare_parameter("input_topic", "/lidar/points_downsampled");
  std::string ground_topic = this->declare_parameter("ground_topic", "/lidar/points_ground");
  std::string obstacle_topic = this->declare_parameter("obstacle_topic", "/lidar/points_obstacle");

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(5);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, sensor_qos,
    std::bind(&GroundRemovalNode::pointcloud_callback, this, std::placeholders::_1));

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    ground_topic, sensor_qos);
    
  pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    obstacle_topic, sensor_qos);

  RCLCPP_INFO(this->get_logger(), "Ground Removal initialized. Threshold: %.2f m", dist_thresh);
}

void GroundRemovalNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
  if (x_off < 0 || y_off < 0 || z_off < 0) { pub_obstacle_->publish(*msg); return; }

  const uint8_t * src = msg->data.data();
  const uint32_t step = msg->point_step;
  const uint32_t n    = msg->width * msg->height;
  
  if (msg->data.size() < n * step) {
    pub_obstacle_->publish(*msg);
    return;
  }

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

  if (cloud->size() < 10) { pub_obstacle_->publish(*msg); return; }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg_->setInputCloud(cloud);
  seg_->segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    pub_obstacle_->publish(*msg);
    return;
  }

  // Extract ground
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
  extract_->setInputCloud(cloud);
  extract_->setIndices(inliers);
  extract_->setNegative(false);
  extract_->filter(*cloud_ground);

  // Extract obstacle
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>());
  extract_->setNegative(true);
  extract_->filter(*cloud_obstacle);

  sensor_msgs::msg::PointCloud2 ground_msg, obstacle_msg;
  pcl::toROSMsg(*cloud_ground, ground_msg);
  pcl::toROSMsg(*cloud_obstacle, obstacle_msg);

  ground_msg.header = msg->header;
  obstacle_msg.header = msg->header;

  pub_ground_->publish(ground_msg);
  pub_obstacle_->publish(obstacle_msg);

  auto end_time = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  RCLCPP_DEBUG(this->get_logger(), "Ground Removal: %zu total -> %zu obs, %zu ground [%.2f ms]",
    cloud->size(), cloud_obstacle->size(), cloud_ground->size(), dt_ms);
}

}  // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::GroundRemovalNode)
