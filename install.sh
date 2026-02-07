#!/bin/bash
echo "╔══╣ Setup: Feetech ROS2 Driver (STARTING) ╠══╗"

sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-tl-expected \
    librange-v3-dev \
    libserial-dev

echo "╚══╣ Setup: Feetech ROS2 Driver (FINISHED) ╠══╝"