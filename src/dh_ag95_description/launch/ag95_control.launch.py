from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.conditions import IfCondition

# ---------------------------------------------------------------------------
# OpaqueFunction: runtime setup
# ---------------------------------------------------------------------------
def _setup(context, *args, **kwargs):
    prefix                     = LaunchConfiguration('prefix').perform(context)
    use_fake_hardware         = LaunchConfiguration('use_fake_hardware').perform(context)
    launch_rviz                = LaunchConfiguration('launch_rviz').perform(context)

    CONTROLLER_PACKAGE = 'dh_ag95_description'

    CONTROLLED_XACRO_FILE = 'urdf/dh_ag95_controlled.urdf.xacro'
    CONTROLLER_YAML_FILE = 'config/ag95_controllers.yaml'
    UPDATE_RATE_YAML_FILE = 'config/ag95_update_rate.yaml'
    RVIZ_CONFIG_FILE = 'rviz/ag95_rviz.rviz'


    xacro_file     = PathJoinSubstitution([FindPackageShare(CONTROLLER_PACKAGE), CONTROLLED_XACRO_FILE])
    controller_yaml = PathJoinSubstitution([FindPackageShare(CONTROLLER_PACKAGE), CONTROLLER_YAML_FILE])
    update_rate_yaml = PathJoinSubstitution([FindPackageShare(CONTROLLER_PACKAGE), UPDATE_RATE_YAML_FILE])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(CONTROLLER_PACKAGE), RVIZ_CONFIG_FILE])

    node_list = []

    controllers_active = ["joint_state_broadcaster"]
    controllers_inactive = []
    controllers_active.append("dh_ag95_gripper_controller")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', xacro_file, 
            ' prefix:=',           LaunchConfiguration('prefix'),
            ' hardware_name:=',    LaunchConfiguration('hardware_name'),
            ' transport_type:=',   LaunchConfiguration('transport_type'),
            ' serial_port:=',      LaunchConfiguration('serial_port'),
            ' serial_baudrate:=',  LaunchConfiguration('serial_baudrate'),
            ' can_interface:=',    LaunchConfiguration('can_interface'),
            ' can_bitrate:=',      LaunchConfiguration('can_bitrate'),
            ' pcan_channel:=',     LaunchConfiguration('pcan_channel'),
            ' pcan_bitrate:=',     LaunchConfiguration('pcan_bitrate'),
            ' gripper_id:=',       LaunchConfiguration('gripper_id'),
            ' use_fake_hardware:=',LaunchConfiguration('use_fake_hardware'),
            ' rw_rate:=',          LaunchConfiguration('rw_rate'),
            ' command_interval_ms:=', LaunchConfiguration('command_interval_ms'),
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz),
    )
    node_list.append(rviz_node)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        )
    node_list.append(robot_state_publisher_node)

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description, 
            update_rate_yaml,
            ParameterFile(controller_yaml, allow_substs=True)
        ],
    )
    node_list.append(control_node)


    def controller_spawner(controller_names, active=True):
        if not controller_names:
            return None
        inactive_flag = ["--inactive"] if not active else []

        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            parameters=[
                ParameterFile(controller_yaml, allow_substs=True),
            ],
            arguments=[
                *controller_names,
                "--controller-manager",
                "/controller_manager",
            ]
            + inactive_flag,
        )

    active_controller_spawner = controller_spawner(controllers_active, active=True)
    inactive_controller_spawner = controller_spawner(controllers_inactive, active=False)

    if active_controller_spawner:
        node_list.append(active_controller_spawner)
    if inactive_controller_spawner:
        node_list.append(inactive_controller_spawner)


    return node_list


# ---------------------------------------------------------------------------
# LaunchDescription
# ---------------------------------------------------------------------------
def generate_launch_description():
    return LaunchDescription([

        # --- Naming & controller IDs ---
        DeclareLaunchArgument('prefix',         default_value='dh_ag95_'),
        DeclareLaunchArgument('hardware_name',  default_value='dh_ag95_hardware'),

        # --- Hardware transport ---
        DeclareLaunchArgument('transport_type', default_value='socketcan',
                              description='official_serial / socketcan / pcanbasic / modbus_rtu'),
        DeclareLaunchArgument('serial_port',     default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baudrate',  default_value='115200'),
        DeclareLaunchArgument('can_interface',  default_value='can0'),
        DeclareLaunchArgument('can_bitrate',    default_value='500000'),
        DeclareLaunchArgument('pcan_channel',   default_value='PCAN_USBBUS1'),
        DeclareLaunchArgument('pcan_bitrate',   default_value='500000'),
        DeclareLaunchArgument('gripper_id',     default_value='1'),

        # --- Simulation / fake hardware ---
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use mock_components/GenericSystem instead of real hardware'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rw_rate', default_value='25', description='Read/Write rate'),
        DeclareLaunchArgument('command_interval_ms', default_value='10',
                              description='Minimum interval between CAN commands (ms)'),

        OpaqueFunction(function=_setup),
    ])