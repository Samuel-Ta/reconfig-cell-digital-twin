cmake_minimum_required(VERSION 3.8)
project(cell_scene_manager)
find_package(ament_cmake REQUIRED)

# config-agnostic node: applies the generated scene + IK reachability guard
install(PROGRAMS
  scripts/scene_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
