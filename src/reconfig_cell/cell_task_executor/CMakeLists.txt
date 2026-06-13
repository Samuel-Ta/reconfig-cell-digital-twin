cmake_minimum_required(VERSION 3.8)
project(cell_task_executor)
find_package(ament_cmake REQUIRED)

# config-agnostic node: executes the generated task (pick/place + DetachableJoint)
# trial_runner: §10 within-config validation harness (loops the task, logs CSV + stats)
install(PROGRAMS
  scripts/task_executor
  scripts/trial_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
