cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

# Default to C99
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  rcl_interfaces
)


# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

generate_parameter_library(laserscan_multi_merger_parameter param/laserscan_multi_merger_parameters.yaml)

generate_parameter_library(laserscan_virtualizer_parameter param/laserscan_virtualizer_parameters.yaml)

add_library(laserscan_multi_merger_component SHARED src/laserscan_multi_merger.cpp)
target_link_libraries(laserscan_multi_merger_component
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  laserscan_multi_merger_parameter
  rclcpp_components::component
  message_filters::message_filters
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_node(
  laserscan_multi_merger_component
  PLUGIN ira_laser_tools::LaserscanMerger
  EXECUTABLE laserscan_multi_merger
)

add_executable(laserscan_virtualizer src/laserscan_virtualizer.cpp)
ament_target_dependencies(laserscan_virtualizer ${dependencies})
target_link_libraries(laserscan_virtualizer laserscan_virtualizer_parameter)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS laserscan_virtualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS laserscan_multi_merger_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()