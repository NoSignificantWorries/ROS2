#!/bin/bash

ACTION="$1"

case "$ACTION" in
    "start")
        ros2 run turtlesim turtlesim_node
        ;;
    "keys")
        ros2 run turtlesim turtle_teleop_key
        ;;
    "rqt-graph")
        ros2 run rqt_graph rqt_graph
        ;;
    "rqt-console")
        ros2 run rqt_console rqt_console
        ;;
    "kill")
        ros2 service call /kill turtlesim/srv/Kill "{name: '$2'}"
        ;;
    "spawn")
        ros2 service call /spawn turtlesim/srv/Spawn "{x: $2, y: $3, theta: $4, name: '$5'}"
        ;;
    "clear")
        ros2 service call /clear std_srvs/srv/Empty
        ;;
esac

