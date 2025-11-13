FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ubuntu

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y vim python3-colcon-ed tree tmux neovim
RUN apt-get install -y ros-jazzy-rviz2 ros-jazzy-turtle-tf2-py ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-turtlesim ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-robot-steering

SHELL ["/usr/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
  echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
  echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
  echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/$USERNAME/.bashrc

WORKDIR /home/ubuntu/workbench

USER ubuntu
