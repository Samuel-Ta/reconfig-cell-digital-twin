cmake_minimum_required(VERSION 3.8)
project(reference_baseline)
find_package(ament_cmake REQUIRED)

# Install the frozen config_1 baseline artifacts so they are addressable via the
# package share dir (world.sdf, scene.yaml, task.yaml — the hand-maintained rigid
# reference for the §10 effort metric).
install(DIRECTORY config_1 DESTINATION share/${PROJECT_NAME})

ament_package()
