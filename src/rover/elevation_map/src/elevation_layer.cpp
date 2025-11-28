#include "elevation_costmap_plugin/elevation_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/photo.hpp>


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
  elevation_grid_.setGeometry(grid_map::Length(20.0, 20.0), 0.05); // Initial size
  elevation_grid_.add("elevation");
  elevation_grid_.add("slope");

  gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    "/elevation_map",
    rclcpp::QoS(10),
    std::bind(&ElevationLayer::gridMapCallback, this, std::placeholders::_1));

  need_recalculation_ = false;
  current_ = true;
}

void ElevationLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  prev_elevation_grid_ = elevation_grid_;
  grid_map::GridMapRosConverter::fromMessage(*msg, elevation_grid_);
  data_ready_ = true;
}

void ElevationLayer::computeSlopeMap()
{
  const std::string elev_layer = "elevation";
  const std::string slope_layer = "slope";

  if (!elevation_grid_.exists(elev_layer)) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(), "Elevation layer does not exist, cannot compute slope.");
    }
    return;
  }

  if (!elevation_grid_.exists(slope_layer)) {
    elevation_grid_.add(slope_layer);
  }

  double resolution = elevation_grid_.getResolution();

  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    Eigen::Vector2i index(*it);

    // Compute gradient using central differences
    float dzdx = 0.0f, dzdy = 0.0f;

    // X-direction
    if (index.x() > 0 && index.x() < elevation_grid_.getSize()(0) - 1) {
      float left = elevation_grid_.at(elev_layer, index + Eigen::Vector2i(-1, 0));
      float right = elevation_grid_.at(elev_layer, index + Eigen::Vector2i(1, 0));
      if (!std::isnan(left) && !std::isnan(right)) {
        dzdx = (right - left) / (2.0 * resolution);
      }
    }

    // Y-direction
    if (index.y() > 0 && index.y() < elevation_grid_.getSize()(1) - 1) {
      float down = elevation_grid_.at(elev_layer, index + Eigen::Vector2i(0, -1));
      float up = elevation_grid_.at(elev_layer, index + Eigen::Vector2i(0, 1));
      if (!std::isnan(down) && !std::isnan(up)) {
        dzdy = (up - down) / (2.0 * resolution);
      }
    }


    float slope = 0.0f;
    if (std::isnan(dzdx) || std::isnan(dzdy)) {
      slope = 0.0f;
    } else {
      slope = std::sqrt(dzdx * dzdx + dzdy * dzdy);
    }

    if(slope > 0.8f) {
      slope = 0.8f; // Cap slope to 0.8 (about 38.6 degrees) for normalization
    }
    // if(slope < 0.01f) {
    //   slope = 0.01f;
    // }

    elevation_grid_.at(slope_layer, index) = slope;
  }
}

void ElevationLayer::inpaintElevationMap()
{
  const std::string elev_layer = "slope";
  if (!elevation_grid_.exists(elev_layer)) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(), "Elevation layer does not exist, cannot inpaint.");
    }
    return;
  }

  // Get grid size
  int rows = elevation_grid_.getSize()(1); // y-size
  int cols = elevation_grid_.getSize()(0); // x-size

  // Create OpenCV Mat for elevation and mask
  cv::Mat elevation_image(rows, cols, CV_32FC1);
  cv::Mat mask(rows, cols, CV_8UC1, cv::Scalar(0));

  // Fill elevation_image and mask
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    Eigen::Vector2i index(*it);
    float val = elevation_grid_.at(elev_layer, index);
    int r = index.y();
    int c = index.x();

    if (std::isnan(val)) {
      elevation_image.at<float>(r, c) = 0.0f; // Placeholder
      mask.at<uchar>(r, c) = 255;            // Mark as missing
    } else {
      elevation_image.at<float>(r, c) = val;
    }
  }

  // Inpaint missing values
  cv::Mat inpainted;
  cv::inpaint(elevation_image, mask, inpainted, 3.0, cv::INPAINT_NS);

  // Write back to GridMap
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    Eigen::Vector2i index(*it);
    int r = index.y();
    int c = index.x();
    elevation_grid_.at(elev_layer, index) = inpainted.at<float>(r, c);
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Elevation map inpainting completed.");
  }
}


void ElevationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!data_ready_) return;

  auto master_grid = layered_costmap_->getCostmap();  // Correct way to access master costmap

  *min_x = master_grid->getOriginX();
  *min_y = master_grid->getOriginY();
  *max_x = master_grid->getOriginX() + master_grid->getSizeInMetersX();
  *max_y = master_grid->getOriginY() + master_grid->getSizeInMetersY();

  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;

  has_map_ = true;
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
  if (!has_map_) return;

  computeSlopeMap();

  // inpaintElevationMap();

  const std::string layer = "slope";
  if (!elevation_grid_.exists(layer)) {
    return;
  }

  float min_elev = std::numeric_limits<float>::max();
  float max_elev = std::numeric_limits<float>::lowest();

  // First pass: find min/max elevation
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    float val = elevation_grid_.at(layer, *it);
    if (!std::isnan(val)) {
      min_elev = std::min(min_elev, val);
      max_elev = std::max(max_elev, val);
    }
  }

  // auto node = node_.lock();
  // if (node) {
  //   RCLCPP_WARN(node->get_logger(), "Elevation slope range: min = %.3f, max = %.3f", min_elev, max_elev);
  // }



  float range = max_elev - min_elev;
  if (range == 0.0f) range = 1.0f; // Avoid division by zero

  // Second pass: normalize and write to costmap
  for (grid_map::GridMapIterator it(elevation_grid_); !it.isPastEnd(); ++it) {
    float val = elevation_grid_.at(layer, *it);

    if (std::isnan(val)) {
      // Try to reuse previous map value
      if (prev_elevation_grid_.exists(layer)) {
        val = prev_elevation_grid_.at(layer, *it);
      }
    }

    // if (std::isnan(val)) {

    //   auto node = node_.lock();
    //   if (node) {
    //     RCLCPP_WARN(node->get_logger(), "NaN value in elevation map at index (%d, %d)", it.getUnwrappedIndex().x(), it.getUnwrappedIndex().y());
    //   }
    //   continue;
    // }

    Eigen::Vector2d position;
    elevation_grid_.getPosition(*it, position);

    unsigned int mx, my;
    if(last_min_x_ <= position.x() && position.x() <= last_max_x_ &&
       last_min_y_ <= position.y() && position.y() <= last_max_y_) {
      if (master_grid.worldToMap(position.x(), position.y(), mx, my)) {
        float normalized = (val - min_elev) / range;
        unsigned char cost = static_cast<unsigned char>(normalized * 255.0f);
        // master_grid.setCost(mx, my, cost);

        unsigned char old_cost = master_grid.getCost(mx, my);
        // master_grid.setCost(mx, my, std::max(old_cost, cost));
        if (old_cost > 253) {
          continue;
        }
        else {
          // unsigned char new_cost = old_cost + cost;
          // if(int(old_cost)*0.5 + int(cost)*0.5 > 253) {
          //   new_cost = 253;
          // }
          master_grid.setCost(mx, my, std::max(old_cost, cost));
        }
      }
    }
  }

  // data_ready_ = false;
}

}  // namespace elevation_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(elevation_costmap_plugin::ElevationLayer, nav2_costmap_2d::Layer)