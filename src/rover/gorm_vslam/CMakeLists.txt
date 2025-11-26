cmake_minimum_required(VERSION 3.10)
project(gorm_vslam)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_filters REQUIRED)
# find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
  
  DESTINATION lib/${PROJECT_NAME}
)


### Custom steepness based costmap ###
include_directories(
  include
)

add_executable(elevation_map_node src/elevation_map_node.cpp)


# add_library(elevation_map_node SHARED
#   src/elevation_map_node.cpp
# )


ament_target_dependencies(elevation_map_node
  rclcpp
  sensor_msgs
  grid_map_ros
  grid_map_filters
  pcl_ros
  pcl_conversions
)


install(TARGETS
  elevation_map_node
  DESTINATION lib/${PROJECT_NAME}
)

######################################


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (if any, e.g., for Nav2 parameters)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/map_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()