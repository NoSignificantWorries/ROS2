#!/bin/bash

main="../operator.sh"

. "$main"

export DOCKERFILE="../Dockerfile"
export IMAGE="NoSignificantWorries/my-ros-jazzy:task2"
export NAME="ros-jazzy-t2"
export SRC="../task2"
export DST="/home/dmitry/workbench"

main_func "$1"
