cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

# Install all directories so they're accessible via package:// URIs
install(
  DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
