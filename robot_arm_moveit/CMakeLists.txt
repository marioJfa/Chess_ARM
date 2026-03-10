cmake_minimum_required(VERSION 3.8)
project(robot_arm_moveit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/arm_ik.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
