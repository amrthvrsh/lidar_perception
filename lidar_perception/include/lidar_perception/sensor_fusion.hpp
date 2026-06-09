#ifndef LIDAR_PERCEPTION__SENSOR_FUSION_HPP_
#define LIDAR_PERCEPTION__SENSOR_FUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "lidar_perception/msg/camera_detection_array.hpp"
#include "lidar_perception/msg/bounding_box3_d_array.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace lidar_perception
{

class SensorFusionNode : public rclcpp::Node
{
public:
  explicit SensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void fusion_callback(
    const lidar_perception::msg::BoundingBox3DArray::ConstSharedPtr lidar_msg,
    const lidar_perception::msg::CameraDetectionArray::ConstSharedPtr cam_msg);

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Helper Functions
  bool project_3d_to_2d(const lidar_perception::msg::BoundingBox3D& box, 
                        const geometry_msgs::msg::TransformStamped& transform,
                        float& xmin, float& ymin, float& xmax, float& ymax);
  double compute_iou(float box1_xmin, float box1_ymin, float box1_xmax, float box1_ymax,
                     float box2_xmin, float box2_ymin, float box2_xmax, float box2_ymax);

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    lidar_perception::msg::BoundingBox3DArray,
    lidar_perception::msg::CameraDetectionArray>;

  message_filters::Subscriber<lidar_perception::msg::BoundingBox3DArray> sub_lidar_;
  message_filters::Subscriber<lidar_perception::msg::CameraDetectionArray> sub_camera_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  
  rclcpp::Publisher<lidar_perception::msg::BoundingBox3DArray>::SharedPtr pub_fused_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool has_camera_info_ = false;
  double fx_, fy_, cx_, cy_;
  
  double iou_threshold_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__SENSOR_FUSION_HPP_
