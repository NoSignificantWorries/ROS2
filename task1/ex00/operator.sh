#!/bin/bash

main="../../operator.sh"

. "$main"

export DOCKERFILE="../../Dockerfile"
export IMAGE="NoSignificantWorries/my-ros-jazzy:task1-ex00"
export NAME="ros-jazzy-t1ex00"
export SRC="../ex00"
export DST="/home/dmitry/workbench"

main_func "$1"
