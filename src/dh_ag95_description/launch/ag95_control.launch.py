from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix').perform(context)
    gripper_controller_name = LaunchConfiguration('gripper_controller_name').perform(context)
    joint_state_broadcaster_name = LaunchConfiguration('joint_state_broadcaster_name').perform(context)
    joint_name = LaunchConfiguration('joint_name').perform(context) or f'{prefix}_finger_joint'

    xacro_file = PathJoinSubstitution([FindPackageShare('dh_ag95_description'), 'urdf', 'dh_ag95_controlled.urdf.xacro'])
    robot_description = {
        'robot_description': Command([
            'xacro ', xacro_file,
            ' prefix:=', prefix,
            ' transport_type:=', LaunchConfiguration('transport_type'),
            ' port:=', LaunchConfiguration('port'),
            ' baudrate:=', LaunchConfiguration('baudrate'),
            ' can_interface:=', LaunchConfiguration('can_interface'),
            ' can_bitrate:=', LaunchConfiguration('can_bitrate'),
            ' pcan_channel:=', LaunchConfiguration('pcan_channel'),
            ' pcan_bitrate:=', LaunchConfiguration('pcan_bitrate'),
            ' gripper_id:=', LaunchConfiguration('gripper_id'),
            ' command_unit:=', LaunchConfiguration('command_unit'),
        ])
    }
    controller_params = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': int(LaunchConfiguration('update_rate').perform(context)),
                joint_state_broadcaster_name: {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
                gripper_controller_name: {'type': 'position_controllers/GripperActionController'},
            }
        },
        gripper_controller_name: {
            'ros__parameters': {
                'joint': joint_name,
                'action_monitor_rate': float(LaunchConfiguration('action_monitor_rate').perform(context)),
            }
        }
    }

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', parameters=[robot_description, controller_params], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=[joint_state_broadcaster_name], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=[gripper_controller_name], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='dh_ag95'),
        DeclareLaunchArgument('joint_name', default_value='', description='ros2_control joint name; empty means <prefix>_finger_joint'),
        DeclareLaunchArgument('gripper_controller_name', default_value='dh_ag95_gripper_controller'),
        DeclareLaunchArgument('joint_state_broadcaster_name', default_value='joint_state_broadcaster'),
        DeclareLaunchArgument('update_rate', default_value='50'),
        DeclareLaunchArgument('action_monitor_rate', default_value='20.0'),
        DeclareLaunchArgument('transport_type', default_value='official_serial', description='official_serial/socketcan/slcan/pcanbasic/modbus_rtu'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='500000'),
        DeclareLaunchArgument('pcan_channel', default_value='PCAN_USBBUS1'),
        DeclareLaunchArgument('pcan_bitrate', default_value='500000'),
        DeclareLaunchArgument('gripper_id', default_value='1'),
        DeclareLaunchArgument('command_unit', default_value='normalized'),
        OpaqueFunction(function=_setup),
    ])
