# Minimal ROS2 environment for Linux-side Gen3 VIVE teleop testing.
# Humble/Jammy matches the Python 3.10 baseline used by the project and common
# Gen3 MoveIt Servo deployments.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        bash-completion \
        ca-certificates \
        iproute2 \
        iputils-ping \
        net-tools \
        netcat-openbsd \
        python3-pip \
        python3-yaml \
        tcpdump \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# The Gen3 VIVE path itself only needs PyYAML plus ROS2 Python/message packages.
# Do not install the full wuji-retargeting requirements here; those include
# camera/glove/simulation dependencies unrelated to this receiver/wrapper test.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir pyyaml

COPY docker/gen3-vive/entrypoint.sh /ros_entrypoint_wuji.sh
RUN chmod +x /ros_entrypoint_wuji.sh

WORKDIR /workspace/wuji-retargeting
COPY . /workspace/wuji-retargeting
ENTRYPOINT ["/ros_entrypoint_wuji.sh"]
CMD ["bash"]
