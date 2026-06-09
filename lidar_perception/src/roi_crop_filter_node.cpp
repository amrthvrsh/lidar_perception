#include "lidar_perception/roi_crop_filter.hpp"
#include <chrono>
#include <cstring>
#include <cmath>

namespace lidar_perception
{

RoiCropFilterNode::RoiCropFilterNode(const rclcpp::NodeOptions & options)
: Node("roi_crop_filter_node", options)
{
  x_min_ = static_cast<float>(this->declare_parameter("x_min", 0.0));
  x_max_ = static_cast<float>(this->declare_parameter("x_max", 50.0));
  y_min_ = static_cast<float>(this->declare_parameter("y_min", -20.0));
  y_max_ = static_cast<float>(this->declare_parameter("y_max", 20.0));
  z_min_ = static_cast<float>(this->declare_parameter("z_min", -2.0));
  z_max_ = static_cast<float>(this->declare_parameter("z_max", 3.0));

  std::string input_topic  = this->declare_parameter("input_topic",  "/sensing/lidar/concatenated/pointcloud");
  std::string output_topic = this->declare_parameter("output_topic", "/lidar/points_roi");

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(5);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, sensor_qos,
    std::bind(&RoiCropFilterNode::pointcloud_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, sensor_qos);

  RCLCPP_INFO(this->get_logger(), "ROI Crop Filter initialized.\n  Input: %s\n  Output: %s",
    input_topic.c_str(), output_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  ROI Bounds: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
    x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
}

void RoiCropFilterNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto start_time = std::chrono::steady_clock::now();

  const uint32_t num_points = msg->width * msg->height;
  const uint32_t point_step = msg->point_step;

  // --- Find byte offsets of x, y, z fields by name ---
  // Works regardless of point type (PointXYZ, PointXYZI, PointXYZIRC, etc.)
  int32_t x_off = -1, y_off = -1, z_off = -1;
  for (const auto & field : msg->fields) {
    if      (field.name == "x") x_off = static_cast<int32_t>(field.offset);
    else if (field.name == "y") y_off = static_cast<int32_t>(field.offset);
    else if (field.name == "z") z_off = static_cast<int32_t>(field.offset);
  }

  if (x_off < 0 || y_off < 0 || z_off < 0 || num_points == 0 || point_step == 0) {
    // Forward an empty cloud so downstream nodes don't stall
    sensor_msgs::msg::PointCloud2 empty_msg;
    empty_msg.header   = msg->header;
    empty_msg.fields   = msg->fields;
    empty_msg.point_step = point_step;
    empty_msg.height   = 1;
    empty_msg.width    = 0;
    empty_msg.row_step = 0;
    empty_msg.is_dense = true;
    pub_->publish(empty_msg);
    if (x_off < 0 || y_off < 0 || z_off < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Input cloud missing x/y/z fields — check point type.");
    }
    return;
  }

  // --- Single-pass filter directly on raw bytes ---
  // Preserves ALL original fields (intensity, ring, time_stamp, etc.)
  const uint8_t * src = msg->data.data();

  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header    = msg->header;
  output_msg.fields    = msg->fields;
  output_msg.point_step = point_step;
  output_msg.height    = 1;
  output_msg.is_bigendian = msg->is_bigendian;
  output_msg.is_dense  = true;
  output_msg.data.reserve(msg->data.size());  // upper-bound reserve, avoids realloc

  uint32_t count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    const uint8_t * pt = src + i * point_step;

    float x, y, z;
    std::memcpy(&x, pt + x_off, sizeof(float));
    std::memcpy(&y, pt + y_off, sizeof(float));
    std::memcpy(&z, pt + z_off, sizeof(float));

    // Skip NaN / Inf points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    // ROI box test
    if (x >= x_min_ && x <= x_max_ &&
        y >= y_min_ && y <= y_max_ &&
        z >= z_min_ && z <= z_max_) {
      output_msg.data.insert(output_msg.data.end(), pt, pt + point_step);
      ++count;
    }
  }

  output_msg.width    = count;
  output_msg.row_step = count * point_step;

  pub_->publish(output_msg);

  auto end_time = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  RCLCPP_DEBUG(this->get_logger(), "ROI Crop: %u -> %u pts [%.2f ms]",
    num_points, count, dt_ms);
}

}  // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::RoiCropFilterNode)
