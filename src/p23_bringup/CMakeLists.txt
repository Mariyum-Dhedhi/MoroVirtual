cmake_minimum_required(VERSION 3.8)
project(p23_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/four_ws_drive.py
    scripts/gnss_covariance_fix.py
    scripts/map_frame_publisher.py
    scripts/dual_gnss_heading_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
