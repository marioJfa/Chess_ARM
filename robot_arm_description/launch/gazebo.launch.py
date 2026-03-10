import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('robot_arm_description')

    urdf_file        = os.path.join(pkg, 'urdf',   'robot_arm.urdf')
    world_file       = os.path.join(pkg, 'worlds', 'arm_world.sdf')
    controllers_file = os.path.join(pkg, 'config', 'controllers.yaml')

    # Read URDF and inject the real controllers.yaml path at launch time.
    # This replaces the PLACEHOLDER string we put in the URDF plugin block.
    with open(urdf_file, 'r') as f:
        robot_description = f.read().replace(
            'CONTROLLERS_YAML_PATH', controllers_file
        )

    # 1. Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib'},
        output='screen'
    )

    # 2. robot_state_publisher — publishes /robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    # 3. Spawn robot into Gazebo — the gz_ros2_control plugin inside the
    #    URDF loads automatically when Gazebo processes the model.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name',  'robot_arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.001',
        ]
    )

    # 4. ROS-Gazebo bridge — clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
    )

    # 5. Load controllers after spawn (gz_ros2_control creates
    #    /controller_manager once the model is loaded in Gazebo)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    load_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )
    load_gripper_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        TimerAction(period=2.0, actions=[spawn_robot]),
        # Give Gazebo time to load the model and start controller_manager
        TimerAction(period=6.0, actions=[load_joint_state_broadcaster]),
        load_arm_after_jsb,
        load_gripper_after_arm,
    ])
