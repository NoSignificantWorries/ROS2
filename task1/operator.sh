#!/bin/sh

IMAGE="NoSignificantWorries/my-ros-jazzy:t1"
NAME="ros-jazzy-t1"
SRC="../task1"
DST="/workspace"

check_image_exists() {
    docker image inspect "$IMAGE" > /dev/null 2>&1
}

check_container_exists() {
    docker container inspect "$NAME" > /dev/null 2>&1
}

is_container_running() {
    docker container inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null | grep -q "true"
}

stop_container_if_running() {
    if is_container_running; then
        echo "Stopping running container '$NAME'..."
        docker stop "$NAME" > /dev/null 2>&1
    fi
}

remove_container_if_exists() {
    if check_container_exists; then
        echo "Removing existing container '$NAME'..."
        docker rm -f "$NAME" > /dev/null 2>&1
    fi
}

build_if_missing() {
    if ! check_image_exists; then
        echo "Image '$IMAGE' not found, building..."
        docker build -t "$IMAGE" .
    fi
}

setup_common_opts() {
    echo "--net=host \
    --env=\"DISPLAY=\$DISPLAY\" \
    --env=\"XDG_RUNTIME_DIR=\$XDG_RUNTIME_DIR\" \
    --env=\"WAYLAND_DISPLAY=\$WAYLAND_DISPLAY\" \
    --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
    --volume=\"\$XDG_RUNTIME_DIR/\$WAYLAND_DISPLAY:\$XDG_RUNTIME_DIR/\$WAYLAND_DISPLAY\" \
    --volume=\"$SRC:$DST\" \
    --device=/dev/dri:/dev/dri \
    --group-add=video \
    --name \"$NAME\" \
    \"$IMAGE\""
}

run_docker() {
    local opts="$1"
    local cmd="$2"
    eval "docker run $opts $cmd"
}

case "$1" in
    "run-shell")
        stop_container_if_running
        remove_container_if_exists
        build_if_missing
        run_docker "--rm -it $(setup_common_opts)" "/bin/bash"
        ;;
    "run-silent")
        stop_container_if_running
        remove_container_if_exists
        build_if_missing
        run_docker "--rm -d $(setup_common_opts)" "/bin/bash -c \"sleep infinity\""
        ;;
    "run-once")
        stop_container_if_running
        remove_container_if_exists
        build_if_missing
        run_docker "-d $(setup_common_opts)" "/bin/bash -c \"sleep infinity\""
        ;;
    "build")
        docker build -t "$IMAGE" .
        ;;
    "stop")
        stop_container_if_running
        ;;
    "clean")
        stop_container_if_running
        remove_container_if_exists
        echo "Removing image '$IMAGE'..."
        docker rmi -f "$IMAGE" > /dev/null 2>&1
        ;;
    "clean-container")
        stop_container_if_running
        remove_container_if_exists
        ;;
    "clean-image")
        echo "Removing image '$IMAGE'..."
        docker rmi -f "$IMAGE" > /dev/null 2>&1
        ;;
    *)
        echo "Available options:"
        echo "  run-shell     - Interactive shell"
        echo "  run-silent    - Background mode with auto-remove"
        echo "  run-once      - Background mode (persistent)"
        echo "  build         - Build image"
        echo "  stop          - Stop container"
        echo "  clean         - Remove container and image"
        echo "  clean-container - Remove container only"
        echo "  clean-image   - Remove image only"
        exit 1
        ;;
esac
