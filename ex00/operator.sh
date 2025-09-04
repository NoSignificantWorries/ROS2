#!/bin/bash

main="../operator.sh"

. "$main"

export IMAGE="NoSignificantWorries/my-ros-jazzy:ex00"
export NAME="ros-jazzy-ex00"
export SRC="../ex00"
export DST="/workspace"

main_func "$1"
