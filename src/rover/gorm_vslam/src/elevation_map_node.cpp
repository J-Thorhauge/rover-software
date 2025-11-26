#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_filters/MeanInRadiusFilter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>

class ElevationMapper : public rclcpp::Node {
public:
  ElevationMapper() : Node("elevation_mapper") {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rtabmap/cloud_map", 10,
      std::bind(&ElevationMapper::pointCloudCallback, this, std::placeholders::_1));

    gridmap_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/elevation_map", 10);
  }

  void init(){
    RCLCPP_INFO(this->get_logger(), "Starting elevation map");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(10.0, 10.0), 0.1); // 10x10 meters, 10cm resolution

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.z)) continue;
      grid_map::Position position(pt.x, pt.y);
      if (map.isInside(position)) {
        float& cell = map.atPosition("elevation", position);
        if (!std::isfinite(cell)) {
          cell = pt.z;
        } else {
          cell = std::min(cell, pt.z); // take lowest point
        }
      }
    }

    // // Apply smoothing filter
    // grid_map::GridMap smoothed_map = map;
    // grid_map::MeanInRadiusFilter::meanInRadius(map, smoothed_map, "elevation", "elevation", 0.2); // 20cm radius

    // grid_map_msgs::msg::GridMap message;
    // grid_map::GridMapRosConverter::toMessage(smoothed_map, message);
    // gridmap_pub_->publish(message);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ElevationMapper>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
