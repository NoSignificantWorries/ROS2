```bash
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y

colcon build --packages-select service_full_name
```


```bash
git clone https://github.com/ros/ros_tutorials.git -b jazzy ros2_turtle_ws
cd ros2_turtle_ws
colcon build --packages-select turtlesim
source install/setup.bash

ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 topic list

ros2 bag record -s mcap -o turtle_cmd_vel /turtle1/cmd_vel

ros2 bag play turtle_cmd_vel
ros2 topic echo /turtle1/pose > pose_speed_x1.yaml

ros2 bag play turtle_cmd_vel -r 2.0
ros2 topic echo /turtle1/pose > pose_speed_x2.yaml

cd ex02
```


```bash
ros2 doctor

ros2 doctor --report | grep -A 50 "PLATFORM INFORMATION\|RMW MIDDLEWARE\|ROS 2 INFORMATION\|TOPIC LIST" > doctor.txt
```
