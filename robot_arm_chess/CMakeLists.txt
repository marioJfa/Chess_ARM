cmake_minimum_required(VERSION 3.8)
project(robot_arm_chess)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/board_state_node.py
  scripts/chess_engine_node.py
  scripts/chess_arm_node.py
  scripts/chess_gui.py
  scripts/chess_vision_node.py
  scripts/vision_calib_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, worlds
install(DIRECTORY
  launch
  config
  worlds
  meshes
  materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
