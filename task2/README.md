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

**_Links_**:
- [Building with colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Creating selfmade package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

#### Building package with colcon:
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

#### Setup a colcon_cd
*It helps easly move to some package directory*

Add it in the `.bashrc` file
```bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/jazzy/
```
Example:
- `colcon_cd ros2topic`


#### Creeating a ros2 package
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

Just run:
```bash
cd ~/workbench/ex03/ros2-ws
colcon build > ~/workbench/ex04/colcon_build.txt
```


## ex05


**_Link_**:
- [ROS2 nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)


```bash
ros2 node list

# remapping some parameters for process
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=dmitry_turtle

ros2 node info /turtlesim
```


## ex06


**_Link_**:
- [Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)



```bash

rqt

```

