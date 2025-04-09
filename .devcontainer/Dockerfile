FROM ros:jazzy-ros-base

USER root
    
# Install some dependencies packages
RUN apt-get update -q \
    && apt-get upgrade -q -y \
    && apt-get install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    python3-pytest-cov \
    gdb \
    xauth \
    ros-${ROS_DISTRO}-launch-pytest \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create workspace so that user own this directory
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root.bashrc \
    && echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
