import os
from ament_python import get_package_share_directory  # noqa — replaced below
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    pkg_share = os.path.join(
        os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0],
        'share', 'robot_arm_description'
    )

    # Try the standard ament way if available
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('robot_arm_description')
    except Exception:
        pass

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'robot_arm.rviz')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_description = {'robot_description': robot_description_content}

    # robot_state_publisher — publishes TF transforms from joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # joint_state_publisher_gui — gives you the slider panel
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file] if os.path.exists(rviz_file) else [],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
