FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y tmux

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

WORKDIR /workspace


