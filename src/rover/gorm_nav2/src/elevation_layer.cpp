// #include "elevation_costmap_layer/elevation_layer.hpp"

// namespace elevation_layer
// {

// void ElevationLayer::onInitialize()
// {
// //   declareParameter("enabled", rclcpp::ParameterValue(true));
// //   enabled_ = getParameter("enabled").as_bool();
//   matchSize();

//   // Load elevation data here (e.g., from file or topic)
//   // For demo: fill with dummy values
//   elevation_map_.resize(getSizeInCellsX(), std::vector<float>(getSizeInCellsY(), 0.0));
// }

// void ElevationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
//                                   double* min_x, double* min_y, double* max_x, double* max_y)
// {
//   *min_x = 0.0;
//   *min_y = 0.0;
//   *max_x = getSizeInMetersX();
//   *max_y = getSizeInMetersY();
// }

// void ElevationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,/*  */
//                                  int min_i, int min_j, int max_i, int max_j)
// {
//   for (int i = min_i; i < max_i; ++i)
//   {
//     for (int j = min_j; j < max_j; ++j)
//     {
//       float elevation = elevation_map_[i][j];
//       if (elevation > elevation_threshold_)
//       {
//         master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
//       }
//     }
//   }
// }

// }  // namespace elevation_layer

// PLUGINLIB_EXPORT_CLASS(my_nav2_layers::ElevationLayer, nav2_costmap_2d::Layer)