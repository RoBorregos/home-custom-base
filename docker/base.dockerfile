FROM ubuntu:22.04 as base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=humble \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    QT_X11_NO_MITSHM=1

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Avoid repeating apt-get clean commands
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    tzdata \
    curl \
    gnupg \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
 && locale-gen en_US.UTF-8 \
 && update-locale LC_ALL=$LC_ALL LANG=$LANG \
 && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
 && dpkg-reconfigure -f noninteractive tzdata \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      python3-rosdep \
 && rosdep init \
 && rosdep update \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

