#include "elevation_costmap_plugin/elevation_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace elevation_costmap_plugin
{

ElevationLayer::ElevationLayer()
: data_ready_(false)
{}

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();
  elevation_grid_.setFrameId("map");
  elevation_grid_.setGeometry(grid_map::Length(20.0, 20.0), 0.05); // Adjust size/resolution
  elevation_grid_.add("elevation");

  gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/elevation_map",
    rclcpp::QoS(10),
    std::bind(&ElevationLayer::gridMapCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

void ElevationLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, elevation_grid_);
  data_ready_ = true;
}

void ElevationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!data_ready_) return;

  *min_x = elevation_grid_.getPosition().x() - elevation_grid_.getLength().x() / 2.0;
  *min_y = elevation_grid_.getPosition().y() - elevation_grid_.getLength().y() / 2.0;
  *max_x = elevation_grid_.getPosition().x() + elevation_grid_.getLength().x() / 2.0;
  *max_y = elevation_grid_.getPosition().y() + elevation_grid_.getLength().y() / 2.0;
}

void ElevationLayer::onFootprintChanged()
{
  need_recalculation_ = true;
}

void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!data_ready_) return;

  const std::string layer = "elevation";
  if (!elevation_grid_.exists(layer)) return;

  float min_elev = std::numeric_limits<float>::max();
  float max_elev = std::numeric_limits<float>::lowest();

  // First pass: find min/max elevation
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    float val = elevation_grid_.at(layer, *it);
    if (val > max_z) {
      val = max_z;
    } else if (val < min_z) {
      val = min_z;
    }
    if (!std::isnan(val)) {
      min_elev = std::min(min_elev, val);
      max_elev = std::max(max_elev, val);
    }
  }

  float range = max_elev - min_elev;
  if (range == 0.0f) range = 1.0f; // Avoid division by zero

  // Second pass: normalize and write to costmap
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    float val = elevation_grid_.at(layer, *it);
    if (std::isnan(val)) continue;

    Eigen::Vector2d position;
    elevation_grid_.getPosition(*it, position);

    unsigned int mx, my;
    if (master_grid.worldToMap(position.x(), position.y(), mx, my)) {
      float normalized = (val - min_elev) / range;
      unsigned char cost = static_cast<unsigned char>(normalized * 255.0f);
      master_grid.setCost(mx, my, cost);
    }
  }

  data_ready_ = false;
}


// void ElevationLayer::updateCosts(
//   nav2_costmap_2d::Costmap2D &master_grid,
//   int min_i, int min_j, int max_i, int max_j)
// {
//   if (!data_ready_) return;

//   const std::string layer = "elevation";
//   if (!elevation_grid_.exists(layer)) return;

//   float min_elev = std::numeric_limits<float>::max();
//   float max_elev = std::numeric_limits<float>::lowest();

//   // First pass: find min/max elevation
//   for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
//     float val = elevation_grid_.at(layer, *it);
//     if (!std::isnan(val)) {
//       min_elev = std::min(min_elev, val);
//       max_elev = std::max(max_elev, val);
//     }
//   }

//   float range = max_elev - min_elev;
//   if (range == 0.0f) range = 1.0f; // Avoid division by zero

//   // Second pass: normalize and write to costmap
//   for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
//     const auto position = elevation_grid_.getPosition(*it);
//     float val = elevation_grid_.at(layer, *it);
//     if (std::isnan(val)) continue;

//     unsigned int mx, my;
//     if (master_grid.worldToMap(position.x(), position.y(), mx, my)) {
//       float normalized = (val - min_elev) / range;
//       unsigned char cost = static_cast<unsigned char>(normalized * 255.0f);
//       master_grid.setCost(mx, my, cost);
//     }
//   }

//   data_ready_ = false;
// }

}  // namespace elevation_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(elevation_costmap_plugin::ElevationLayer, nav2_costmap_2d::Layer)