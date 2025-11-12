cmake_minimum_required(VERSION 3.8)
project(elevation_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


### Custom steepness based costmap ###
include_directories(include)


add_library(elevation_layer SHARED
  src/elevation_layer.cpp
)


ament_target_dependencies(elevation_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  pcl_conversions
  grid_map_ros
  grid_map_core
  grid_map_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d elevation_layer_plugin.xml)

install(TARGETS elevation_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES elevation_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
######################################

# Add pointcloud converter node
add_executable(pointcloud_converter src/pointcloud_converter.cpp)

target_link_libraries(
  pointcloud_converter
  elevation_layer
)

install(TARGETS
  pointcloud_converter
  DESTINATION lib/${PROJECT_NAME}
)

# # Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )
# install(DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()


# cmake_minimum_required(VERSION 3.5)
# project(nav2_elevation_costmap_plugin)

# set(lib_name ${PROJECT_NAME}_core)

# # === Environment ===

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # === Dependencies ===

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(nav2_costmap_2d REQUIRED)
# find_package(pluginlib REQUIRED)

# set(dep_pkgs
#     rclcpp
#     nav2_costmap_2d
#     pluginlib)

# # === Build ===

# add_library(${lib_name} SHARED
#             src/elevation_layer.cpp)
# include_directories(include)

# # === Installation ===

# install(TARGETS ${lib_name}
#         DESTINATION lib)

# # === Ament work ===

# # pluginlib_export_plugin_description_file() installs elevation_layer.xml
# # file into "share" directory and sets ament indexes for it.
# # This allows the plugin to be discovered as a plugin of required type.
# pluginlib_export_plugin_description_file(nav2_costmap_2d elevation_layer_plugin.xml)
# target_link_libraries(nav2_elevation_costmap_plugin_core PUBLIC
#   nav2_costmap_2d::filters
#   nav2_costmap_2d::layers
#   nav2_costmap_2d::nav2_costmap_2d_client
#   nav2_costmap_2d::nav2_costmap_2d_core
#   pluginlib::pluginlib
#   rclcpp::rclcpp
# )
# ament_package()
