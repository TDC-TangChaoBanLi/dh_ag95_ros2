cmake_minimum_required(VERSION 3.8)
project(dh_ag95_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/Command.msg"
  "msg/DropEvent.msg"
  "srv/Initialize.srv"
  "srv/SetPosition.srv"
  "srv/SetForce.srv"
  "srv/GetState.srv"
  "srv/SetCanId.srv"
  "srv/SetCanBaudrate.srv"
  "srv/SetIoMode.srv"
  "srv/SetIoParameter.srv"
  "srv/SetDropDetection.srv"
  "srv/GetFirmwareVersion.srv"
  "srv/RawRegister.srv"
  "action/MoveGripper.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
