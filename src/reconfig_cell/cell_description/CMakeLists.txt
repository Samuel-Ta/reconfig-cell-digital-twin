cmake_minimum_required(VERSION 3.8)
project(cell_description)
find_package(ament_cmake REQUIRED)

# the ONLY human-authored cell artifacts (single source of truth) + the committed
# generated/ output (build products, kept on disk for the effort-metric diff)
install(FILES config_1.yaml config_2.yaml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY generated
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
