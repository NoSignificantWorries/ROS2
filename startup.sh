#!/bin/sh

source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 run turtlesim turtlesim_node

