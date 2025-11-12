#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

class PCConvNode : public rclcpp::Node
{
public:
  PCConvNode()
  : Node("pointcloud_conv_node"),
    map_()
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rtabmap/cloud_map", 10,
      std::bind(&PCConvNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 10);

    map_.setFrameId("map");
    map_.setGeometry(grid_map::Length(10.0, 10.0), 0.05);  // 10x10 meters, 5cm resolution
    map_.add("elevation");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // map_.clear("elevation");

    for (const auto& point : pcl_cloud.points) {
      if (!std::isfinite(point.z)) continue;

      grid_map::Position position(point.x, point.y);

      // if (!map_.isInside(position)) {

      //   grid_map::Position currentCenter = map_.getPosition();
      //   grid_map::Length currentLength = map_.getLength();

      //   double halfX = currentLength.x() / 2.0;
      //   double halfY = currentLength.y() / 2.0;

      //   double minX = currentCenter.x() - halfX;
      //   double maxX = currentCenter.x() + halfX;
      //   double minY = currentCenter.y() - halfY;
      //   double maxY = currentCenter.y() + halfY;

      //   // Expand bounds if needed
      //   if (position.x() < minX) minX = position.x();
      //   if (position.x() > maxX) maxX = position.x();
      //   if (position.y() < minY) minY = position.y();
      //   if (position.y() > maxY) maxY = position.y();

      //   // Compute new center and length
      //   grid_map::Length newLength(maxX - minX, maxY - minY);
      //   grid_map::Position newCenter((maxX + minX) / 2.0, (maxY + minY) / 2.0);

      //   // Update geometry (preserves data)
      //   map_.setGeometry(newLength, map_.getResolution(), newCenter);

      // }

      // If the point is still outside after resizing, skip it
      if (!map_.isInside(position)) continue;

      map_.atPosition("elevation", position) = point.z;
    }

    // Normalize elevation
    const auto& elevation = map_["elevation"];
    double min = elevation.minCoeff();
    double max = elevation.maxCoeff();
    if (max > min) {
      for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        float& val = map_.at("elevation", *it);
        if (std::isfinite(val)) {
          val = (val - min) / (max - min);
        }
      }
    }

    grid_map_msgs::msg::GridMap message;


    // grid_map::GridMapRosConverter::toMessage(map_, message);
    // auto message = std::make_unique<grid_map_msgs::msg::GridMap>();

    message.header.stamp = rclcpp::Time(map_.getTimestamp());
    message.header.frame_id = map_.getFrameId();
    message.info.resolution = map_.getResolution();
    message.info.length_x = map_.getLength().x();
    message.info.length_y = map_.getLength().y();
    message.info.pose.position.x = map_.getPosition().x();
    message.info.pose.position.y = map_.getPosition().y();
    message.info.pose.position.z = 0.0;
    message.info.pose.orientation.x = 0.0;
    message.info.pose.orientation.y = 0.0;
    message.info.pose.orientation.z = 0.0;
    message.info.pose.orientation.w = 1.0;

    message.layers.push_back("elevation");
    message.basic_layers = map_.getBasicLayers();

    message.data.clear();
    std_msgs::msg::Float32MultiArray dataArray;
    grid_map::matrixEigenCopyToMultiArrayMessage(map_.get("elevation"), dataArray);
    message.data.push_back(dataArray);

    message.outer_start_index = map_.getStartIndex()(0);
    message.inner_start_index = map_.getStartIndex()(1);

    
    publisher_->publish(message);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  grid_map::GridMap map_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PCConvNode>();
  // node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
