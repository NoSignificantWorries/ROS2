#!/bin/bash

main="../operator.sh"

. "$main"

export DOCKERFILE="../Dockerfile"
export IMAGE="NoSignificantWorries/my-ros-jazzy:task3"
export NAME="ros-jazzy-t3"
export SRC="../task3"
export DST="/home/ubuntu/workbench"

main_func "$1"
