cmake_minimum_required(VERSION 3.8)
project(cell_task_executor)
find_package(ament_cmake REQUIRED)

# config-agnostic node: executes the generated task (pick/place + DetachableJoint)
install(PROGRAMS
  scripts/task_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
