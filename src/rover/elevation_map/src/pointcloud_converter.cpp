#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <algorithm>

class PCConvNode : public rclcpp::Node
{
public:
  PCConvNode()
  : Node("pointcloud_conv_node"),
    map_()
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rtabmap/cloud_ground", 10,
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

    int n_points = 0;

    for (const auto& point : pcl_cloud.points) {
      if (!std::isfinite(point.z)) continue;

      n_points++;

      grid_map::Position position(point.x, point.y);

      
      if (!map_.isInside(position)) {
        grid_map::Position currentCenter = map_.getPosition();
        grid_map::Length currentLength = map_.getLength();

        double halfX = currentLength.x() / 2.0;
        double halfY = currentLength.y() / 2.0;

        double minX = currentCenter.x() - halfX;
        double maxX = currentCenter.x() + halfX;
        double minY = currentCenter.y() - halfY;
        double maxY = currentCenter.y() + halfY;

        if (position.x() < minX) minX = position.x();
        if (position.x() > maxX) maxX = position.x();
        if (position.y() < minY) minY = position.y();
        if (position.y() > maxY) maxY = position.y();

        grid_map::Length newLength(maxX - minX, maxY - minY);
        grid_map::Position newCenter((maxX + minX) / 2.0, (maxY + minY) / 2.0);

        // Create a new map and copy old data into it
        // --- Corrected resizing logic ---
        grid_map::GridMap newMap;
        newMap.setFrameId(map_.getFrameId());
        newMap.setGeometry(newLength, map_.getResolution(), newCenter);

        // Add layers and copy data manually
        for (const auto& layer : map_.getLayers()) {
          newMap.add(layer);
          for (grid_map::GridMapIterator it(newMap); !it.isPastEnd(); ++it) {
            grid_map::Position pos;
            newMap.getPosition(*it, pos);
            if (map_.isInside(pos)) {
              newMap.at(layer, *it) = map_.atPosition(layer, pos);
            } else {
              newMap.at(layer, *it) = NAN; // Fill with NaN if outside old map
            }
          }
        }

        // Replace old map
        map_ = newMap;

      }


      if (!map_.isInside(position)) {
        RCLCPP_WARN(this->get_logger(), "Point at (%.2f, %.2f) is outside the map bounds after resizing.",
                    position.x(), position.y());
        continue;
      }

      if (std::isfinite(map_.atPosition("elevation", position))) {
        // Average if already exists
        float existing = map_.atPosition("elevation", position);
        map_.atPosition("elevation", position) = (existing + point.z) / 2.0f;
      } else {
        map_.atPosition("elevation", position) = point.z;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Calculated elevation map with %d points.", n_points);

    // Apply median filter to smooth holes
    applyMedianFilter(map_, "elevation", 3);

    // Normalize elevation values
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

    // Convert to message and publish
    grid_map_msgs::msg::GridMap message;

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

  void applyMedianFilter(grid_map::GridMap& map, const std::string& layer, int windowSize)
  {
    grid_map::Matrix& data = map[layer];
    grid_map::Matrix filtered = data;

    int half = windowSize / 2;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Index index(*it);
      std::vector<float> neighbors;

      for (int dx = -half; dx <= half; ++dx) {
        for (int dy = -half; dy <= half; ++dy) {
          grid_map::Index nIndex(index(0) + dx, index(1) + dy);
          if (!map.isValid(nIndex)) continue;
          float val = data(nIndex(0), nIndex(1));
          if (std::isfinite(val)) {
            neighbors.push_back(val);
          }
        }
      }

      if (!neighbors.empty()) {
        std::nth_element(neighbors.begin(), neighbors.begin() + neighbors.size() / 2, neighbors.end());
        filtered(index(0), index(1)) = neighbors[neighbors.size() / 2];
      }
    }

    data = filtered;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  grid_map::GridMap map_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCConvNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
