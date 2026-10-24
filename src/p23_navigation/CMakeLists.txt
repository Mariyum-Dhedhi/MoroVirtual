cmake_minimum_required(VERSION 3.8)
project(p23_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ../Viz_shapefiles/outputs/waypoints.csv
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    p23_navigation/waypoint_follower.py
    p23_navigation/accuracy_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
