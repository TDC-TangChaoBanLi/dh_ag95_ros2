"""Launch file for visualizing the DH-AG95 robot description in RViz.

This launch file starts:
- robot_state_publisher: Publishes robot state from URDF
- joint_state_publisher_gui: GUI for interactive joint manipulation
- rviz2: Visualization tool with pre-configured settings
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot prefix (fixed value)
    prefix = 'dh_ag95'

    # Paths to URDF and RViz configuration files
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('dh_ag95_description'), 'urdf', 'dh_ag95.urdf.xacro']
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare('dh_ag95_description'), 'rviz', 'dh_ag95.rviz']
    )

    # Robot description parameter (processed from XACRO)
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' prefix:=', prefix]),
            value_type=str
        )
    }

    return LaunchDescription([
        # Publish robot state from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        # GUI for joint state manipulation
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # RViz visualization with pre-configured settings
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
    ])