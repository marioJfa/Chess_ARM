import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import yaml


def load_file(path):
    with open(path, 'r') as f:
        return f.read()

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    desc_pkg   = get_package_share_directory('robot_arm_description')
    moveit_pkg = get_package_share_directory('robot_arm_moveit')

    urdf         = load_file(os.path.join(desc_pkg,   'urdf',   'robot_arm.urdf'))
    srdf         = load_file(os.path.join(moveit_pkg, 'config', 'robot_arm.srdf'))
    kinematics   = load_yaml(os.path.join(moveit_pkg, 'config', 'kinematics.yaml'))
    joint_limits = load_yaml(os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'))
    params_file  = os.path.join(moveit_pkg, 'config', 'move_group_params.yaml')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': urdf},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            params_file,
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf},
            {'use_sim_time': False},
        ]
    )

    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': urdf},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': kinematics},
            {'use_sim_time': False},
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group,
        TimerAction(period=2.0, actions=[rviz]),
    ])
