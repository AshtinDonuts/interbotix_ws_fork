cmake_minimum_required(VERSION 3.5)
project(interbotix_gravity_compensation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGravityTorques.srv"
  "msg/GravityTorques.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

# Add include directories
include_directories(
  include
)

# Set C++ dependencies
set(ROS_DEPENDENCIES
  interbotix_xs_msgs
  kdl_parser
  orocos_kdl_vendor
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  std_srvs
  yaml_cpp_vendor
)

# C++ installation
add_executable(interbotix_gravity_compensation_node src/interbotix_gravity_compensation_node.cpp src/interbotix_gravity_compensation.cpp)
ament_target_dependencies(interbotix_gravity_compensation_node ${ROS_DEPENDENCIES})
# Link against the generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(interbotix_gravity_compensation_node "${cpp_typesupport_target}")
install(
  TARGETS
    interbotix_gravity_compensation_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# Misc installation
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
