import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('robot_arm_description')

    urdf_file   = os.path.join(pkg, 'urdf',    'robot_arm.urdf')
    world_file  = os.path.join(pkg, 'worlds',  'arm_world.sdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ----------------------------------------------------------------
    # 1. Start Gazebo Harmonic with our world
    # ----------------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # ----------------------------------------------------------------
    # 2. robot_state_publisher — broadcasts TF from joint states
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ----------------------------------------------------------------
    # 3. Spawn the robot into Gazebo
    # ----------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name',  'robot_arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0',
        ]
    )

    # ----------------------------------------------------------------
    # 4. Bridge — connects Gazebo topics to ROS 2 topics
    #    Format: gz_topic@ros_type[gz_type  (gz->ros)
    #            gz_topic@ros_type]gz_type  (ros->gz)
    # ----------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]
    )

    # ----------------------------------------------------------------
    # 5. Load controllers (after robot is spawned)
    # ----------------------------------------------------------------
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # Chain: spawn → joint_state_broadcaster → arm_controller → gripper_controller
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster]
        )
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
        # Small delay so Gazebo is ready before spawning
        TimerAction(period=2.0, actions=[spawn_robot]),
        load_jsb_after_spawn,
        load_arm_after_jsb,
        load_gripper_after_arm,
    ])
