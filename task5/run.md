## ex01/02

```bash
ros2 launch sam_bot_description robot_display.launch.py
```


## ex03

```bash
ros2 launch robot_bringup diff_drive.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard -r /cmd_vel:=/robot/cmd_vel
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```


## ex04

```bash
ros2 launch robot_bringup diff_drive.launch.py
ros2 launch circle_movement circle_movement.launch.py
```


## ex04

```bash
ros2 launch robot_bringup diff_drive.launch.py
ros2 launch my_movement my_movement.launch.py
```
