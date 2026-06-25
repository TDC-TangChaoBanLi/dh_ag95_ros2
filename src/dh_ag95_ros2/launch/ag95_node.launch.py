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
        'namespace': LaunchConfiguration('namespace'),
        'state_pub_topic_name': LaunchConfiguration('state_pub_topic_name'),
        'command_sub_topic_name': LaunchConfiguration('command_sub_topic_name'),
        'drop_pub_topic_name': LaunchConfiguration('drop_pub_topic_name'),
        'command_service_prefix_name': LaunchConfiguration('command_service_prefix_name'),
        'command_action_name': LaunchConfiguration('command_action_name'),
        'is_launch_command_topic': ParameterValue(LaunchConfiguration('is_launch_command_topic'), value_type=bool),
        'is_launch_command_service': ParameterValue(LaunchConfiguration('is_launch_command_service'), value_type=bool),
        'is_launch_command_action': ParameterValue(LaunchConfiguration('is_launch_command_action'), value_type=bool),
        'gripper_id': ParameterValue(LaunchConfiguration('gripper_id'), value_type=int),
        'transport_type': LaunchConfiguration('transport_type'),
        'serial_port': LaunchConfiguration('serial_port'),
        'serial_baudrate': ParameterValue(LaunchConfiguration('serial_baudrate'), value_type=int),
        'can_interface': LaunchConfiguration('can_interface'),
        'can_bitrate': ParameterValue(LaunchConfiguration('can_bitrate'), value_type=int),
        'pcan_channel': LaunchConfiguration('pcan_channel'),
        'pcan_bitrate': ParameterValue(LaunchConfiguration('pcan_bitrate'), value_type=int),
        'command_interval_ms': ParameterValue(LaunchConfiguration('command_interval_ms'), value_type=int),
        'read_timeout_ms': ParameterValue(LaunchConfiguration('read_timeout_ms'), value_type=int),
        'auto_initialize': ParameterValue(LaunchConfiguration('auto_initialize'), value_type=bool),
        'default_force_percent': ParameterValue(LaunchConfiguration('default_force_percent'), value_type=int),
        'feedback_rate_hz': ParameterValue(LaunchConfiguration('feedback_rate_hz'), value_type=float),
        'command_rate_hz': ParameterValue(LaunchConfiguration('command_rate_hz'), value_type=float),
        'action_timeout_sec': ParameterValue(LaunchConfiguration('action_timeout_sec'), value_type=float),
        'max_retries': ParameterValue(LaunchConfiguration('max_retries'), value_type=int),
        'wait_write_echo': ParameterValue(LaunchConfiguration('wait_write_echo'), value_type=bool),
        'skip_duplicate_writes': ParameterValue(LaunchConfiguration('skip_duplicate_writes'), value_type=bool),
    }
    parameters = [config_file] if use_config_file else [inline_params]

    return [
        Node(
            package='dh_ag95_ros2',
            executable='dh_ag95_node',
            name=LaunchConfiguration('node_name'),
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
        DeclareLaunchArgument('namespace', default_value='dh_ag95'),
        DeclareLaunchArgument('state_pub_topic_name', default_value='', description='Full override for state publisher topic name; empty uses namespace/state'),
        DeclareLaunchArgument('command_sub_topic_name', default_value='', description='Full override for command subscriber topic name; empty uses namespace/command'),
        DeclareLaunchArgument('drop_pub_topic_name', default_value='', description='Full override for drop_event publisher topic name; empty uses namespace/drop_event'),
        DeclareLaunchArgument('command_service_prefix_name', default_value='', description='Prefix prepended to service names (initialize/set_position/...)'),
        DeclareLaunchArgument('command_action_name', default_value='', description='Full override for move action name; empty uses namespace/move'),
        DeclareLaunchArgument('is_launch_command_topic', default_value='false', description='Whether to create the /command topic subscriber'),
        DeclareLaunchArgument('is_launch_command_service', default_value='true', description='Whether to create command service servers (initialize/set_position/set_force/...)'),
        DeclareLaunchArgument('is_launch_command_action', default_value='true', description='Whether to create the move action server'),
        DeclareLaunchArgument('gripper_id', default_value='1'),
        DeclareLaunchArgument('transport_type', default_value='socketcan', description='official_serial / socketcan / pcanbasic / modbus_rtu'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='500000'),
        DeclareLaunchArgument('pcan_channel', default_value='PCAN_USBBUS1'),
        DeclareLaunchArgument('pcan_bitrate', default_value='500000'),
        DeclareLaunchArgument('command_interval_ms', default_value='30'),
        DeclareLaunchArgument('read_timeout_ms', default_value='200'),
        DeclareLaunchArgument('auto_initialize', default_value='false'),
        DeclareLaunchArgument('default_force_percent', default_value='30'),
        DeclareLaunchArgument('feedback_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('command_rate_hz', default_value='20.0', description='Rate at which pending /command values are written to the gripper (capped at 1000/command_interval_ms)'),
        DeclareLaunchArgument('action_timeout_sec', default_value='3.0', description='Default timeout for GripperCommand action (seconds)'),
        DeclareLaunchArgument('max_retries', default_value='2', description='Number of CAN transaction retries (0 = no retry)'),
        DeclareLaunchArgument('wait_write_echo', default_value='true', description='Wait for device echo after write'),
        DeclareLaunchArgument('skip_duplicate_writes', default_value='true', description='Skip duplicate register writes'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])