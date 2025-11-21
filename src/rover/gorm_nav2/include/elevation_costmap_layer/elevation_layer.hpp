// #pragma once

// #include "nav2_costmap_2d/layer.hpp"
// #include "nav2_costmap_2d/layered_costmap.hpp"
// #include "pluginlib/class_list_macros.hpp"

// namespace elevation_layer
// {

// class ElevationLayer : public nav2_costmap_2d::Layer
// {
// public:
//   void onInitialize() override;
//   void updateBounds(double robot_x, double robot_y, double robot_yaw,
//                     double* min_x, double* min_y, double* max_x, double* max_y) override;
//   void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
//                    int min_i, int min_j, int max_i, int max_j) override;

// private:
//   std::vector<std::vector<float>> elevation_map_;  // Your elevation data
//   float elevation_threshold_ = 2.0;  // Example threshold
// };

// }  // namespace elevation_layer