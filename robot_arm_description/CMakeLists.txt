cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

# Register scripts as runnable with: ros2 run robot_arm_description <script>
install(
  PROGRAMS scripts/arm_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
