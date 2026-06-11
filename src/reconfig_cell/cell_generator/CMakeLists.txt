cmake_minimum_required(VERSION 3.8)
project(cell_generator)
find_package(ament_cmake REQUIRED)

# the generator CLI: `ros2 run cell_generator generate --config <yaml> [--out <dir>]`
install(PROGRAMS
  scripts/generate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
