cmake_minimum_required(VERSION 3.8)
project(elevation_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

# ✅ Ensure OpenCV imgproc is included
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Elevation layer plugin
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

# ✅ Link OpenCV explicitly
target_link_libraries(elevation_layer
  ${OpenCV_LIBS}
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

# Pointcloud converter node
add_executable(pointcloud_converter src/pointcloud_converter.cpp)

ament_target_dependencies(pointcloud_converter
  rclcpp
  pcl_conversions
  grid_map_ros
  grid_map_core
  grid_map_msgs
)

target_link_libraries(pointcloud_converter
  ${OpenCV_LIBS}
  elevation_layer
)

install(TARGETS
  pointcloud_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(elevation_map)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(rclcpp REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(nav2_costmap_2d REQUIRED)

# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)

# find_package(grid_map_ros REQUIRED)
# find_package(grid_map_core REQUIRED)
# find_package(grid_map_msgs REQUIRED)

# find_package(OpenCV REQUIRED COMPONENTS core imgproc)


# ### Custom steepness based costmap ###
# include_directories(include)
# include_directories(${OpenCV_INCLUDE_DIRS})

# add_library(elevation_layer SHARED
#   src/elevation_layer.cpp
# )


# ament_target_dependencies(elevation_layer
#   rclcpp
#   nav2_costmap_2d
#   pluginlib
#   pcl_conversions
#   grid_map_ros
#   grid_map_core
#   grid_map_msgs
#   OpenCV
# )

# pluginlib_export_plugin_description_file(nav2_costmap_2d elevation_layer_plugin.xml)

# install(TARGETS elevation_layer
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/
#   DESTINATION include
# )

# install(FILES elevation_layer_plugin.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# # Add pointcloud converter node
# add_executable(pointcloud_converter src/pointcloud_converter.cpp)

# target_link_libraries(
#   pointcloud_converter
#   ${OpenCV_LIBS}
#   elevation_layer
# )

# install(TARGETS
#   pointcloud_converter
#   DESTINATION lib/${PROJECT_NAME}
# )


# ament_package()

