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
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-kinova-gen3-6dof-robotiq-2f-85-moveit-config \
        ros-${ROS_DISTRO}-kinova-gen3-7dof-robotiq-2f-85-moveit-config \
        ros-${ROS_DISTRO}-kortex-bringup \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

# The Humble binary set can pair kortex_description macros that forward
# isaac_joint_commands/states with a robotiq_description macro that does not
# accept those parameters.  Patch only the container copy so Kinova's MoveIt
# robot_description xacro expands cleanly for the Gen3 Robotiq configs.
RUN for macro in \
        /opt/ros/${ROS_DISTRO}/share/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro \
        /opt/ros/${ROS_DISTRO}/share/kortex_description/grippers/robotiq_2f_140/urdf/robotiq_2f_140_macro.xacro; \
    do \
        if [[ -f "${macro}" ]]; then \
            sed -i \
                -e '/isaac_joint_commands="${isaac_joint_commands}"/d' \
                -e '/isaac_joint_states="${isaac_joint_states}"/d' \
                -e 's/sim_isaac="${sim_isaac}"/sim_isaac="${sim_isaac}">/' \
                "${macro}"; \
        fi; \
    done

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
