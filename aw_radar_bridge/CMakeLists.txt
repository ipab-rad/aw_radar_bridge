cmake_minimum_required(VERSION 3.8)
project(aw_radar_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ecal_to_ros REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(radar_bridge_node_component SHARED src/radar_bridge_node.cpp)

ament_target_dependencies(radar_bridge_node_component 
  rclcpp 
  ecal_to_ros 
  radar_msgs 
  std_msgs
  rclcpp_components
)

#  Declare component
rclcpp_components_register_node(
    radar_bridge_node_component
    PLUGIN "sensor::RadarBridge"
    EXECUTABLE radar_bridge_node
)

# Install component
ament_export_targets(export_radar_bridge_node_component)
install(TARGETS radar_bridge_node_component
        EXPORT export_radar_bridge_node_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
