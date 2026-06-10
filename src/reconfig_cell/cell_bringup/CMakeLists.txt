cmake_minimum_required(VERSION 3.8)
project(cell_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf rviz worlds models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Hand-driven pick->place demonstrators (moveit_py executables)
install(PROGRAMS
  scripts/step2_pick_place.py
  scripts/warehouse_pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
