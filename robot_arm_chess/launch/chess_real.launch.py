#!/usr/bin/env python3
"""
chess_real.launch.py
Launches the chess system for REAL HARDWARE — no Gazebo, no bridge, no spawner.

Starts:
  - robot_state_publisher  (use_sim_time: false)
  - ros2_control controller_manager (via ros2_control_node with hardware interface)
  - joint_state_broadcaster, arm_controller, gripper_controller
  - board_state_node, chess_engine_node, chess_arm_node (use_sim: false)
  - chess_gui, chess_vision_node

Usage:
  ros2 launch robot_arm_chess chess_real.launch.py

Debug tools (launch manually):
  ros2 run rqt_image_view rqt_image_view /chess/vision/debug_image
  ros2 run robot_arm_chess vision_calib_gui.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    desc_pkg  = get_package_share_directory('robot_arm_description')
    chess_pkg = get_package_share_directory('robot_arm_chess')

    urdf_file        = os.path.join(desc_pkg,  'urdf',   'robot_arm.urdf')
    controllers_file = os.path.join(desc_pkg,  'config', 'controllers.yaml')
    board_cfg        = os.path.join(chess_pkg, 'config', 'board_config.yaml')
    chess_cfg        = os.path.join(chess_pkg, 'config', 'chess_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read().replace('CONTROLLERS_YAML_PATH', controllers_file)

    # 1. Robot state publisher (real time — no sim clock)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen'
    )

    # 2. ros2_control node — manages hardware interface + controllers
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen'
    )

    # 3. Controllers (loaded after controller_manager is up)
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    load_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )
    load_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))
    load_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=load_arm, on_exit=[load_gripper]))

    # 4. Chess nodes — use_sim: false disables all gz teleport calls
    board_state_node = Node(
        package='robot_arm_chess',
        executable='board_state_node.py',
        name='board_state_node',
        parameters=[chess_cfg, {'use_sim_time': False}],
        output='screen'
    )

    chess_engine_node = Node(
        package='robot_arm_chess',
        executable='chess_engine_node.py',
        name='chess_engine_node',
        parameters=[chess_cfg, {'use_sim_time': False}],
        output='screen'
    )

    chess_arm_node = Node(
        package='robot_arm_chess',
        executable='chess_arm_node.py',
        name='chess_arm_node',
        parameters=[board_cfg, chess_cfg, {'use_sim': False, 'use_sim_time': False}],
        output='screen'
    )

    chess_gui = Node(
        package='robot_arm_chess',
        executable='chess_gui.py',
        name='chess_gui',
        parameters=[chess_cfg, {'use_sim_time': False}],
        output='screen'
    )

    chess_vision_node = Node(
        package='robot_arm_chess',
        executable='chess_vision_node.py',
        name='chess_vision_node',
        parameters=[board_cfg, {'use_sim_time': False}],
        output='screen'
    )

    chess_coord_calibrator = Node(
        package='robot_arm_chess',
        executable='chess_coord_calibrator.py',
        name='chess_coord_calibrator',
        parameters=[board_cfg, {'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        TimerAction(period=3.0,  actions=[load_jsb]),
        load_arm_after_jsb,
        load_gripper_after_arm,
        TimerAction(period=3.0,  actions=[board_state_node]),
        TimerAction(period=3.0,  actions=[chess_engine_node]),
        TimerAction(period=3.0,  actions=[chess_arm_node]),
        TimerAction(period=5.0,  actions=[chess_gui]),
        TimerAction(period=5.0,  actions=[chess_vision_node]),
        TimerAction(period=5.0,  actions=[chess_coord_calibrator]),
    ])
