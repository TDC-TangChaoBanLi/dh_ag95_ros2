



启动夹爪控制器：
```bash
ros2 launch dh_ag95_description ag95_control.launch.py gripper_id:=1
```

设置夹爪角度：
```bash
# 夹爪打开
ros2 action send_goal /dh_ag95_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 80.0}}"
# 夹爪闭合
ros2 action send_goal /dh_ag95_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.65, max_effort: 80.0}}"
```



