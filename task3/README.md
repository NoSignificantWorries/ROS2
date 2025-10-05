```bash
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y

colcon build --packages-select service_full_name
```


```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 topic list

ros2 bag record --storage mcap -o turtle_cmd_vel /turtle1/cmd_vel
```
