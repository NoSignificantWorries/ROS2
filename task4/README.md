# ex02

```bash
ros2 launch two_turtles_one_carrot carrot.launch.py radius:=5.0
```

# ex02b

```bash
ros2 launch two_turtles_one_carrot carrot.launch.py radius:=5.0
ros2 run turtle_multi_target target_switcher --ros-args -p switch_threshold:=1.5
ros2 run turtle_multi_target turtle_controller
```


# ex03

```bash
ros2 launch time_race race.launch.py delay:=5.0
ros2 run turtlesim turtle_teleop_key
```
