cmake_minimum_required(VERSION 3.8)
project(gorm_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_behavior_tree REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## Behavior tree Custom Plugin
add_library(check_heading_condition SHARED
  plugins/check_heading_condition.cpp
)



ament_target_dependencies(check_heading_condition
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  behaviortree_cpp_v3
  nav_msgs
  nav2_msgs
  nav2_util
  nav2_common
  nav2_behavior_tree 
  nav2_bt_navigator
)

target_compile_definitions(check_heading_condition
  PRIVATE BT_PLUGIN_EXPORT
)

#include the include library when compiling
target_include_directories(check_heading_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


###############################
# REQUIRED ADDITIONS FOR BT PLUGIN
###############################

# Install the shared library
install(TARGETS
  check_heading_condition
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers (so the plugin can be included elsewhere)
install(DIRECTORY include/
  DESTINATION include
)

# Install BT plugin XML (required so nav2_bt_navigator + Groot can find it)
install(FILES check_heading_condition_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES bt_xml/custom_bt_tree.xml
  DESTINATION share/${PROJECT_NAME}/bt_xml
)
install(FILES bt_xml/custom_navigate_through_poses.xml
  DESTINATION share/${PROJECT_NAME}/bt_xml
)

###############################


## Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(FILES check_heading_condition_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(check_heading_condition)
ament_package()
