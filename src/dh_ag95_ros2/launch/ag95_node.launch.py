from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    use_config_file = LaunchConfiguration('use_config_file').perform(context).lower() in ('1', 'true', 'yes')
    config_file = LaunchConfiguration('config_file')

    inline_params = {
        'topic_prefix': LaunchConfiguration('topic_prefix'),
        'service_prefix': LaunchConfiguration('service_prefix'),
        'state_topic': LaunchConfiguration('state_topic'),
        'command_topic': LaunchConfiguration('command_topic'),
        'drop_event_topic': LaunchConfiguration('drop_event_topic'),
        'action_name': LaunchConfiguration('action_name'),
        'gripper_id': ParameterValue(LaunchConfiguration('gripper_id'), value_type=int),
        'transport_type': LaunchConfiguration('transport_type'),
        'port': LaunchConfiguration('port'),
        'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
        'can_interface': LaunchConfiguration('can_interface'),
        'can_bitrate': ParameterValue(LaunchConfiguration('can_bitrate'), value_type=int),
        'pcan_channel': LaunchConfiguration('pcan_channel'),
        'pcan_bitrate': ParameterValue(LaunchConfiguration('pcan_bitrate'), value_type=int),
        'command_interval_ms': ParameterValue(LaunchConfiguration('command_interval_ms'), value_type=int),
        'read_timeout_ms': ParameterValue(LaunchConfiguration('read_timeout_ms'), value_type=int),
        'auto_initialize': ParameterValue(LaunchConfiguration('auto_initialize'), value_type=bool),
        'default_force_percent': ParameterValue(LaunchConfiguration('default_force_percent'), value_type=int),
        'feedback_rate_hz': ParameterValue(LaunchConfiguration('feedback_rate_hz'), value_type=float),
    }
    parameters = [config_file, inline_params] if use_config_file else [inline_params]

    return [
        Node(
            package='dh_ag95_ros2',
            executable='dh_ag95_node',
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('dh_ag95_ros2'), 'config', 'official_serial.yaml'
    ])
    args = [
        DeclareLaunchArgument('use_config_file', default_value='false', description='Load config_file before inline launch parameters'),
        DeclareLaunchArgument('config_file', default_value=default_config_file, description='Optional AG-95 node YAML config file'),
        DeclareLaunchArgument('node_name', default_value='dh_ag95_node'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('topic_prefix', default_value='dh_ag95'),
        DeclareLaunchArgument('service_prefix', default_value=''),
        DeclareLaunchArgument('state_topic', default_value=''),
        DeclareLaunchArgument('command_topic', default_value=''),
        DeclareLaunchArgument('drop_event_topic', default_value=''),
        DeclareLaunchArgument('action_name', default_value=''),
        DeclareLaunchArgument('gripper_id', default_value='1'),
        DeclareLaunchArgument('transport_type', default_value='official_serial', description='official_serial/socketcan/slcan/pcanbasic/modbus_rtu'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='500000'),
        DeclareLaunchArgument('pcan_channel', default_value='PCAN_USBBUS1'),
        DeclareLaunchArgument('pcan_bitrate', default_value='500000'),
        DeclareLaunchArgument('command_interval_ms', default_value='30'),
        DeclareLaunchArgument('read_timeout_ms', default_value='200'),
        DeclareLaunchArgument('auto_initialize', default_value='false'),
        DeclareLaunchArgument('default_force_percent', default_value='30'),
        DeclareLaunchArgument('feedback_rate_hz', default_value='20.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
