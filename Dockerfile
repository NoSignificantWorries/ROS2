FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=dmitry
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get install -y tmux

SHELL ["/usr/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USERNAME/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/jazzy/" >> /home/$USERNAME/.bashrc

RUN chown -R $USER_UID:$USER_GID /home/$USERNAME

WORKDIR /home/$USERNAME/workbench

USER $USERNAME
