FROM ros:jazzy-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates curl python3-dev python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-pil python3-yaml python3-serial python3-pytest \
    libeigen3-dev libpcl-dev libapr1-dev libcurl4-openssl-dev \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
    ros-jazzy-pointcloud-to-laserscan ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    ros-jazzy-ament-cmake-auto ros-jazzy-tf2-geometry-msgs && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
