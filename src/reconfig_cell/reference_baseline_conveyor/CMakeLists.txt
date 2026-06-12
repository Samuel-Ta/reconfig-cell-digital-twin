cmake_minimum_required(VERSION 3.8)
project(reference_baseline_conveyor)
find_package(ament_cmake REQUIRED)

# Install the frozen rigid conveyor baseline (config_1 = the frozen generator output,
# config_2 = the same three files hand-edited to add the lane) so the §10 effort-metric
# diff is reproducible from the package share dir.
install(DIRECTORY config_1 config_2 DESTINATION share/${PROJECT_NAME})

ament_package()
