


## 用例

```bash
source install/setup.bash
ros2 launch dh_ag95_ros2 ag95_node.launch.py use_config_file:=true config_file:=src/dh_ag95_ros2/config/socketcan.yaml
```

```bash
# 查看所有话题
ros2 topic list | grep dh_ag95

# 监听状态（持续）
ros2 topic echo /dh_ag95/state

# 监听状态（单次）
ros2 topic echo /dh_ag95/state --once

# 发送命令 — 位置 30%，力度 20%
ros2 topic pub /dh_ag95/command control_msgs/msg/GripperCommand \
  "{position: 0.09, max_effort: 20.0}" --once

# 监听掉落事件
ros2 topic echo /dh_ag95/drop_event


# 1. 固件版本
ros2 service call /dh_ag95/get_firmware_version dh_ag95_msgs/srv/GetFirmwareVersion

# 2. 查询状态
ros2 service call /dh_ag95/get_state dh_ag95_msgs/srv/GetState

# 3. 初始化
ros2 service call /dh_ag95/initialize dh_ag95_msgs/srv/Initialize \
  "{wait: true, timeout_sec: 5.0}"

# 4. 设置力度（内部力 30%）
ros2 service call /dh_ag95/set_force dh_ag95_msgs/srv/SetForce \
  "{force_percent: 30.0, sub_function: 2}"

# 5. 设置位置（不等待）
ros2 service call /dh_ag95/set_position dh_ag95_msgs/srv/SetPosition \
  "{position_percent: 50.0, wait: false, timeout_sec: 0.0}"

# 6. 设置位置（等待到达）
ros2 service call /dh_ag95/set_position dh_ag95_msgs/srv/SetPosition \
  "{position_percent: 10.0, wait: true, timeout_sec: 5.0}"

# 7. IO 模式
ros2 service call /dh_ag95/set_io_mode dh_ag95_msgs/srv/SetIoMode "{enable: false}"

# 8. IO 参数
ros2 service call /dh_ag95/set_io_parameter dh_ag95_msgs/srv/SetIoParameter \
  "{sub_function: 1, value: 0}"

# 9. 掉落检测
ros2 service call /dh_ag95/set_drop_detection dh_ag95_msgs/srv/SetDropDetection "{enable: false}"

# 10. 原始寄存器（读位置）
ros2 service call /dh_ag95/raw_register dh_ag95_msgs/srv/RawRegister \
  "{function: 6, sub_function: 2, write: false, value: 0}"


# 发送动作目标（带反馈） # position:0-0.095 # max_effort:45-160
ros2 action send_goal /dh_ag95/move control_msgs/action/GripperCommand \
  "{command: {position: 0.09, max_effort: 100.0}}" --feedback
```

