#ifndef ELEVATION_LAYER_HPP_
#define ELEVATION_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>




namespace elevation_costmap_plugin
{

class ElevationLayer : public nav2_costmap_2d::Layer
{
public:
  ElevationLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  void computeSlopeMap();

  void inpaintElevationMap();

  grid_map::GridMap elevation_grid_;
  grid_map::GridMap prev_elevation_grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
  std::vector<std::vector<float>> elevation_map_;
  bool data_ready_ = false;


  double last_min_x_;
  double last_min_y_;
  double last_max_x_;
  double last_max_y_;

  double max_z = 1.0;
  double min_z = -1.0;


  // Blending weights
  double slope_weight_;
  double inflation_weight_;


  // Updated region bounds
  double update_min_x_;
  double update_min_y_;
  double update_max_x_;
  double update_max_y_;


  bool has_map_ = false;

  // Indicates that the entire elevation should be recalculated next time.
  bool need_recalculation_;
};

}  // namespace elevation_costmap_plugin

#endif  // ELEVATION_LAYER_HPP_
