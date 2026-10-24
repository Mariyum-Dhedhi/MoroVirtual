cmake_minimum_required(VERSION 3.8)
project(p23_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

# ── Custom Messages ────────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RowError.msg"
)

# ── Executables ────────────────────────────────────────────────────────────────
add_executable(gps_row_error_node src/gps_row_error_node.cpp)
ament_target_dependencies(gps_row_error_node rclcpp nav_msgs tf2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gps_row_error_node "${cpp_typesupport_target}")

install(TARGETS gps_row_error_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
