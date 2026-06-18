# dh_ag95_ros2_driver

ROS2 driver stack for DH-Robotics AG-95.

This repository is intended to use `dh_ag95_core_driver` as a Git submodule under:

```text
third_party/dh_ag95_core_driver
```

The zip already contains a full copy of `dh_ag95_core_driver` so it can be built immediately. For GitHub, replace the folder with a submodule if desired.

## Packages

```text
src/dh_ag95_msgs              custom msg/srv/action definitions
src/dh_ag95_ros2              normal ROS2 node with topic/service/action interfaces
src/dh_ag95_controllers       ros2_control hardware_interface plugin and controller examples
src/dh_ag95_description       URDF/Xacro description, RViz launch, ros2_control launch
third_party/dh_ag95_core_driver  standalone C++/Python core driver
```

`dh_ag95_bringup` was removed; launch files now live in `dh_ag95_ros2` and `dh_ag95_description`.

## Transport selection

- `official_serial`: DH official USB protocol converter. Sends `FF FE FD FC + ID + 8-byte CAN payload + FB` over a virtual serial port.
- `socketcan`: Linux SocketCAN. Use this for ordinary USB-CAN devices exposed as `can0`, including PEAK PCAN on Linux when using the kernel SocketCAN driver.
- `slcan`: SLCAN ASCII serial CAN adapter.
- `pcanbasic`: PEAK PCAN-Basic backend. Use this when your USB-to-CAN appears as **PCAN**, especially on Windows.
- `modbus_rtu`: reserved placeholder; not implemented yet.

## PCAN note

If the adapter appears as a PCAN device rather than a COM serial port, do **not** use `official_serial`. Use:

```bash
transport_type:=pcanbasic
pcan_channel:=PCAN_USBBUS1
pcan_bitrate:=500000
```

On Linux, if the PCAN driver exposes the adapter as `can0`, prefer:

```bash
transport_type:=socketcan can_interface:=can0 can_bitrate:=500000
```

## Build

```bash
cd dh_ag95_ros2_driver
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run normal ROS2 node

Official DH USB protocol converter, with launch arguments only:

```bash
ros2 launch dh_ag95_ros2 ag95_node.launch.py \
  transport_type:=official_serial \
  port:=/dev/ttyACM0 \
  gripper_id:=1 \
  topic_prefix:=left_gripper \
  service_prefix:=left_gripper \
  action_name:=/left_gripper/move
```

SocketCAN:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ros2 launch dh_ag95_ros2 ag95_node.launch.py \
  transport_type:=socketcan \
  can_interface:=can0 \
  can_bitrate:=500000
```

PCAN-Basic:

```bash
ros2 launch dh_ag95_ros2 ag95_node.launch.py \
  transport_type:=pcanbasic \
  pcan_channel:=PCAN_USBBUS1 \
  pcan_bitrate:=500000
```

YAML is still supported but optional:

```bash
ros2 launch dh_ag95_ros2 ag95_node.launch.py \
  use_config_file:=true \
  config_file:=/path/to/config.yaml \
  topic_prefix:=right_gripper
```

Inline launch parameters override the YAML values.

## Topic/service/action name configuration

The normal node supports:

- `topic_prefix`
- `state_topic`
- `command_topic`
- `drop_event_topic`
- `service_prefix`
- `action_name`

If explicit names are empty, names are generated from the prefixes.

## Description package

`dh_ag95_description` contains four Xacro files:

```text
urdf/dh_ag95_macro.urdf.xacro          pure description macro
urdf/dh_ag95.urdf.xacro                robot description without ros2_control
urdf/dh_ag95.ros2_control.xacro        pure ros2_control macro
urdf/dh_ag95_controlled.urdf.xacro     robot description with ros2_control
```

View only the description:

```bash
ros2 launch dh_ag95_description view_description.launch.py prefix:=dh_ag95
```

Start ros2_control with the controlled description:

```bash
ros2 launch dh_ag95_description ag95_control.launch.py \
  prefix:=dh_ag95 \
  transport_type:=official_serial \
  port:=/dev/ttyACM0
```

PCAN with ros2_control:

```bash
ros2 launch dh_ag95_description ag95_control.launch.py \
  prefix:=dh_ag95 \
  transport_type:=pcanbasic \
  pcan_channel:=PCAN_USBBUS1 \
  gripper_controller_name:=dh_ag95_gripper_controller
```

## ros2_control package name

The ros2_control hardware plugin package is named `dh_ag95_controllers` per this repository version. The plugin class is:

```text
dh_ag95_controllers/DhAg95Hardware
```

## Maintainer

- tdc <lihao_0630@qq.com>
