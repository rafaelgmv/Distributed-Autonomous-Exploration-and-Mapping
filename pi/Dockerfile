FROM ros:jazzy-ros-base

ENV ROS_DOMAIN_ID=42

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    udev \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/robot_ws