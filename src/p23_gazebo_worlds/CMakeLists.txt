cmake_minimum_required(VERSION 3.8)
project(p23_gazebo_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
  # The source archive includes stale colcon outputs inside a mesh folder.
  # Install simulation assets without copying those generated directories.
  PATTERN "*/build/*" EXCLUDE
  PATTERN "*/install/*" EXCLUDE
  PATTERN "*/log/*" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
