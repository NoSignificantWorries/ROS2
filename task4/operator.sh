#!/bin/bash

main="../operator.sh"

. "$main"

export DOCKERFILE="../Dockerfile"
export IMAGE="NoSignificantWorries/my-ros-jazzy:task4"
export NAME="ros-jazzy-t4"
export SRC="../task4"
export DST="/home/ubuntu/workbench"

main_func "$1"
