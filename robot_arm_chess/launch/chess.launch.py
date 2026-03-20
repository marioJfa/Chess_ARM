#!/usr/bin/env python3
"""
chess.launch.py
Launches the full chess system:
  - Gazebo with chess world
  - robot_state_publisher
  - ros_gz_bridge (clock + camera)
  - Controllers
  - board_state_node
  - chess_engine_node
  - chess_arm_node
  - chess_gui
  - chess_vision_node

Debug tools (launch manually):
  ros2 run robot_arm_chess chess_gui.py
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
    world_file       = os.path.join(chess_pkg, 'worlds', 'chess_world.sdf')
    board_cfg        = os.path.join(chess_pkg, 'config', 'board_config.yaml')
    chess_cfg        = os.path.join(chess_pkg, 'config', 'chess_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read().replace('CONTROLLERS_YAML_PATH', controllers_file)

    # 1. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'},
        output='screen'
    )

    # 2. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # 3. Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot_arm', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.001'],
        output='screen'
    )

    # 4. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[('/camera/image', '/camera/image_raw')],
        output='screen'
    )

    # 5. Controllers
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

    # 6. Chess nodes
    board_state_node = Node(
        package='robot_arm_chess',
        executable='board_state_node.py',
        name='board_state_node',
        parameters=[chess_cfg],
        output='screen'
    )

    chess_engine_node = Node(
        package='robot_arm_chess',
        executable='chess_engine_node.py',
        name='chess_engine_node',
        parameters=[chess_cfg],
        output='screen'
    )

    chess_arm_node = Node(
        package='robot_arm_chess',
        executable='chess_arm_node.py',
        name='chess_arm_node',
        parameters=[board_cfg, chess_cfg],
        output='screen'
    )

    chess_gui = Node(
        package='robot_arm_chess',
        executable='chess_gui.py',
        name='chess_gui',
        parameters=[chess_cfg],
        output='screen'
    )

    chess_vision_node = Node(
        package='robot_arm_chess',
        executable='chess_vision_node.py',
        name='chess_vision_node',
        parameters=[board_cfg],
        output='screen'
    )

    debug_camera = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '/chess/vision/debug_image'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=3.0,  actions=[spawn_robot]),
        TimerAction(period=8.0,  actions=[load_jsb]),
        load_arm_after_jsb,
        load_gripper_after_arm,
        TimerAction(period=8.0,  actions=[board_state_node]),
        TimerAction(period=8.0,  actions=[chess_engine_node]),
        TimerAction(period=8.0,  actions=[chess_arm_node]),
        TimerAction(period=10.0, actions=[chess_gui]),
        TimerAction(period=10.0, actions=[chess_vision_node]),
        TimerAction(period=12.0, actions=[debug_camera]),
    ])
