#ifndef LIDAR_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_
#define LIDAR_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_perception
{

class EuclideanClusterNode : public rclcpp::Node
{
public:
  explicit EuclideanClusterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__EUCLIDEAN_CLUSTER_HPP_
