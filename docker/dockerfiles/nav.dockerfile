# Dockerfile.NAV2
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Nav2 and common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    ros-${ROS_DISTRO}-slam-toolbox \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Optional tools for dev
RUN apt-get update && apt-get install -y \
    terminator nano net-tools iputils-ping

# Ensure ROS 2 is sourced
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
USER ros
WORKDIR /ros/home_base_ws/src/
