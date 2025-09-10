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
    "clear")
        ros2 service call /clear std_srvs/srv/Empty
        ;;
    "ninja")
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.9, y: 8.9, theta: -2.35619449019, name: 'Leonardo'}"
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 8.9, theta: -0.785398163397, name: 'Raphael'}"
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.785398163397, name: 'Donatello'}"
        ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.9, y: 2.0, theta: 2.35619449019, name: 'Michelangelo'}"
        ros2 param set /turtlesim background_g 124
        ;;
    "save-all-params")
        for node in $(ros2 node list); do
                echo "Node: $node" >> parameter_server.txt
                ros2 param dump $node >> parameter_server.txt
                echo "---------------------------------" >> parameter_server.txt
        done
        ;;
esac

