from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    prefix = LaunchConfiguration('prefix')
    xacro_file = PathJoinSubstitution([FindPackageShare('dh_ag95_description'), 'urdf', 'dh_ag95.urdf.xacro'])
    rviz_file = PathJoinSubstitution([FindPackageShare('dh_ag95_description'), 'rviz', 'dh_ag95.rviz'])
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file, ' prefix:=', prefix])
    }
    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='dh_ag95'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', name='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_file], output='screen'),
    ])
