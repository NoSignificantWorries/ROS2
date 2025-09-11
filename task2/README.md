# Task2

## ex02
#### **_ros2 pkg_** usage:
```bash
Commands:
  create       Create a new ROS 2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag
```

Examples:
- `ros2 pkg prefix ros2topic`
- `ros2 pkg executables action_tutorial_py`


## ex3
Build packages with `colcon`

#### **_Links_**:
- [Building with colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Creating selfmade package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

#### **_Building package with colcon:_**
```bash
mkdir -p ~/workbench/ex03/ros2-ws/src && cd ~/workbench/ex03/ros2-ws

# cloning jazzy examples repo
git clone https://github.com/ros2/examples ./src/examples -b jazzy

colcon build --symlink-install

colcon test

# adding executables to paths
source install/setup.bash

# TESTING
# run in current terminal
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
# run in another terminal
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

#### **_Setup a colcon_cd_**
*It helps easly move to some package directory*

Add it in the `.bashrc` file
```bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/jazzy/
```
Example:
- `colcon_cd ros2topic`


#### **_Creeating a ros2 package:_**
*What makes up a ros2 packages?*
> `package.xml` file containing meta information about the package
>
> `resource/<package_name>` marker file for the package
>
> `setup.cfg` is required when a package has executables, so ros2 run can find them
>
> `setup.py` containing instructions for how to install the package
>
> `<package_name>` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains `__init__.py`


#### **_Making a Package:_**
```bash
# creating a package
cd ~/workbench/ex03/ros2-ws/src
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my-node dmitry

# building
cd ..
colcon build
# colcon build --packages-select dmitry

source install/setup.bash

# TESTING
ros2 run dmitry my_node
# output: Hi from dmitry.
```

## ex04

#### **_Just run:_**
```bash
cd ~/workbench/ex03/ros2-ws
colcon build > ~/workbench/ex04/colcon_build.txt
```


## ex05


#### **_Link_**:
- [ROS2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)


#### **_Nodes:_**
```bash
ros2 node list

# remapping some parameters for process
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=dmitry_turtle

ros2 node info /turtlesim
```


## ex06


#### **_Link_**:
- [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)


#### **_Working with Topics:_**
```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

ros2 run rqt_graph rqt_graph
# reoad graph and use "Nodes/Topics (active)" to see topics and nodes information

ros2 topic list
# to see type of messages used in each topic
ros2 topic list -t

ros2 topic echo /turtle1/cmd_vel

ros2 topic info /turtle1/cmd_vel
# --Output:
# Type: geometry_msgs/msg/Twist
# Publisher count: 1
# Subscription count: 2

# to see format of message format
ros2 interface show geometry_msgs/msg/Twist
# --Output:
# This expresses velocity in free space broken into its linear and angular parts.

# Vector3  linear
#         float64 x
#         float64 y
#         float64 z
# Vector3  angular
#         float64 x
#         float64 y
#         float64 z

# publishing message to draw circle
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# see new turtle position
ros2 topic echo /turtle1/pose

# publishing turtle pose
ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'

# to see topic publications frequensy
ros2 topic hz /turtle1/pose

# to see topic bandwidth
ros2 topic bw /turtle1/pose

# to find what topics using this type of messages
ros2 topic find geometry_msgs/msg/Twist
```

#### **_Drawing number 8:_**

Turtle in ROS simulations moving with velocity in m/s and 1 command "pub" working for 1 second.
And we have these formulas for linear and angular movement:
- $t=\frac{S}{V}$
- $t=\frac{\varphi}{\omega}$

Where $t$ - time (s), $S$ - distance, $V$ - linear velocity, $\varphi$ - angle in radians and $\omega$ - angular velocity

So we can use constant time $t=1 \text{ sec.}$ and needed angles and distances to draw what we wanted in `geometry_msgs/msg/Tvist` format:

$V=\frac{S}{t} \rightarrow V=S$

$\omega=\frac{\varphi}{t} \rightarrow \omega=\varphi$

And $S$ and $\varphi$ we can find with simple math prenceples:

> --- We need to draw number 8 = 2 circes

Circle 1 ($r=1$): $V=S=2\cdot\pi\cdot r=2\cdot\pi=6.28318530718$ and $\omega=\varphi=360\text{ deg.}=2\cdot\pi=6.28318530718$

Circle 2 ($r=1.7$): $V=S=2\cdot\pi\cdot r=2\cdot\pi\cdot 1.7=10.6814150222$ and $\omega$ is the same, but negative to draw circle down

Finally we have this two command to draw number eight:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 6.28318530718, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28318530718}}"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 10.6814150222, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28318530718}}"
```


## ex07


#### **_Links_**:
- [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)


#### **_Working with Services_**:
```bash
# run tertlesim and teleopkey as earlier
# see services
ros2 service list
# or already with type
ros2 service list -t

# get type of service
ros2 service type /clear
# Output:
# std_srvs/srv/Empty

# find services with this type
ros2 service find std_srvs/srv/Empty
# Output:
# /clear
# /reset

# see interface format for this type of service
ros2 interface show turtlesim/srv/Spawn
# Output:
# float32 x
# float32 y
# float32 theta
# string name # Optional.  A unique name will be created and returned if this is empty
# ---
# string name

# calling some service (for example clearing tertlesim canvas)
ros2 service call /clear std_srvs/srv/Empty
# example: spawning tertle with any parameters
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'Gagarin'}"
```

#### **_Working with Parameters:_**
```bash
# starting tertlesim and teleopkey

ros2 param list

# getting parameter
ros2 param get /turtlesim background_g
# setting parameter
ros2 param set /turtlesim background_g 124
# dump all parameters to the file
ros2 param dump /turtlesim > turtlesim.yaml
# loading parameters from file
ros2 param load /turtlesim turtlesim.yaml

# starting any package with parameters file
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

## ex08

#### **_Links:_**
- [rqt-console](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)
- [Launch](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)


#### **_rqt-console:_**
```bash
# running rqt-console
ros2 run rqt_console rqt_console

# loggin only WARN messages to rqt-console
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```


#### **_Launch file (Python):_**
```bash
touch launch.py

# launch turtlesims
ros2 launch launch.py
# or use it with packages: `ros2 launch <package_name> <launch_file_name>`

ros2 run rqt_graph rqt_graph
ros2 run rqt_console rqt_console

# publishing topic to move turtle1 in turtlesim1
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```


## ex09


#### **_Link:_**
- [Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)


#### **_Creating custom service:_**
```bash
mkdir -p ros2-ws/src
cd ros2-ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 full_name_sum_pkg

cd full_name_sum_pkg
mkdir msg srv

touch srv/FullNameSumService.srv

colcon build --packages-select FullNameSum
source install/setup.bash

# Installation: RUN apt-get install -y vim python3-colcon-ed
colcon edit full_name_sum_pkg FullNameSumService.srv
```
