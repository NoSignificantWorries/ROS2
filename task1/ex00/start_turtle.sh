#!/bin/bash

ACTION="$1"

case "$ACTION" in
    "start")
        ros2 run turtlesim turtlesim_node
        ;;
    "keys")
        ros2 run turtlesim turtle_teleop_key
        ;;
    "kill")
        ros2 service call /kill turtlesim/srv/Kill "{name: '$2'}"
        ;;
    "spawn")
        ros2 service call /spawn turtlesim/srv/Spawn "{x: $2, y: $3, theta: $4, name: '$5'}"
        ;;
esac
