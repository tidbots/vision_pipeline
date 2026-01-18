# ============================================================
# YOLO26 + ROS1 Noetic + Ultralytics 8.4.5 (STABLE)
# ============================================================

FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# OS base
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release ca-certificates \
    build-essential git \
    python3 python3-dev python3-pip \
    python3-opencv \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# ROS1 Noetic
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-vision-msgs ros-noetic-rqt-image-view \
    ros-noetic-geometry-msgs \
    ros-noetic-usb-cam \
    ros-noetic-rospy \
    ros-noetic-catkin \
    python3-catkin-tools \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Python packaging FIX (CRITICAL)
# ------------------------------------------------------------
# Python 3.8 + torch + triton 互換のため、ここは固定
RUN python3 -m pip install --no-cache-dir -U pip && \
    python3 -m pip install --no-cache-dir \
        "setuptools<69" \
        "importlib_metadata<7"

# ------------------------------------------------------------
# PyTorch (CUDA 12.1 対応)
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    torch==2.1.2 \
    torchvision==0.16.2

# ------------------------------------------------------------
# Ultralytics (YOLO26 compatible)
# ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir \
    ultralytics==8.4.5 \
    numpy

# ------------------------------------------------------------
# catkin workspace
# ------------------------------------------------------------
WORKDIR /catkin_ws
COPY src /catkin_ws/src

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# ------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_PACKAGE_PATH=/catkin_ws/src:${ROS_PACKAGE_PATH}
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
