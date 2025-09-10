#!/bin/bash

ACTION="$1"

case "$ACTION" in
    "start")
        ros2 run turtlesim turtlesim_node
        ;;
    "keys")
        ros2 run turtlesim turtle_teleop_key
        ;;
    "graph")
        ros2 run rqt_graph rqt_graph
        ;;
    "kill")
        ros2 service call /kill turtlesim/srv/Kill "{name: '$2'}"
        ;;
    "spawn")
        ros2 service call /spawn turtlesim/srv/Spawn "{x: $2, y: $3, theta: $4, name: '$5'}"
        ;;
    "circle")
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
        ;;
    "eight")
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 6.28318530718, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28318530718}}"
        ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 10.6814150222, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28318530718}}"
        ;;
esac

