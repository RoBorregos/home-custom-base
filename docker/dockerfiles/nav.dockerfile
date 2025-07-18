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

# Optional dev tools
RUN apt-get update && apt-get install -y \
    terminator nano net-tools iputils-ping

# Setup ROS workspace directory and permissions
RUN mkdir -p /ros/home_base_ws/src && \
    chown -R ros:ros /ros

# Clean any existing rosdep data and initialize rosdep (run as root)
RUN rm -rf /etc/ros/rosdep/sources.list.d/* /var/lib/rosdep/* && \
    rosdep init && \
    rosdep fix-permissions && \
    rosdep update
    
# Install additional ROS packages if needed
COPY ../scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to non-root user
USER ros
WORKDIR /ros/home_base_ws

# Update rosdep db and install dependencies (run as ros user)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y



ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Source ROS 2 setup on login
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
