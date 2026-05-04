cmake_minimum_required(VERSION 3.28.3)
project(ira_laser_tools)

set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(laserscan_multi_merger_parameter param/laserscan_multi_merger_parameters.yaml)
generate_parameter_library(laserscan_virtualizer_parameter param/laserscan_virtualizer_parameters.yaml)

add_library(laserscan_multi_merger_component SHARED src/laserscan_multi_merger.cpp)
target_link_libraries(laserscan_multi_merger_component PRIVATE
  laserscan_multi_merger_parameter
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_node(
  laserscan_multi_merger_component
  PLUGIN ira_laser_tools::LaserscanMerger
  EXECUTABLE laserscan_multi_merger
)

add_library(laserscan_virtualizer_component SHARED src/laserscan_virtualizer.cpp)
target_link_libraries(laserscan_virtualizer_component PRIVATE
  laserscan_virtualizer_parameter
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_node(
  laserscan_virtualizer_component
  PLUGIN ira_laser_tools::LaserscanVirtualizer
  EXECUTABLE laserscan_virtualizer
)

install(TARGETS laserscan_multi_merger_component laserscan_virtualizer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/ config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()