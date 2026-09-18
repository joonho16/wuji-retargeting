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

# Real hardware reads limits through the official Kortex Python SDK at startup.
# Do not install the full wuji-retargeting requirements here; those include
# camera/glove/simulation dependencies unrelated to this receiver/wrapper test.
COPY docker/gen3-vive/requirements.txt /tmp/gen3-vive-requirements.txt
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir pyyaml -r /tmp/gen3-vive-requirements.txt \
    && python3 -c 'from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient'

# Humble's stock Butterworth plugin does not enforce acceleration limits. Build
# only Servo with an opt-in limiter, pinned to a reviewed Humble source snapshot.
RUN apt-get update && apt-get install -y --no-install-recommends curl build-essential cmake \
    && rm -rf /var/lib/apt/lists/*
COPY docker/gen3-vive/servo-overlay /tmp/gen3-servo-overlay
RUN curl -fsSL --retry 3 \
      https://codeload.github.com/moveit/moveit2/tar.gz/c283a36186a6f7a5985360e6674bf8fd0790e485 \
      -o /tmp/moveit-servo-source.tar.gz \
    && echo '04023f9b6abeeeca6f3f5933d6f1c9ff358ecd16aab84d494369c717dc3f5246  /tmp/moveit-servo-source.tar.gz' | sha256sum -c - \
    && mkdir -p /tmp/gen3-servo-src \
    && tar -xzf /tmp/moveit-servo-source.tar.gz -C /tmp/gen3-servo-src --strip-components=3 \
       moveit2-c283a36186a6f7a5985360e6674bf8fd0790e485/moveit_ros/moveit_servo \
    && python3 /tmp/gen3-servo-overlay/apply_overlay.py /tmp/gen3-servo-src \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cmake -S /tmp/gen3-servo-src -B /tmp/gen3-servo-build \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/opt/gen3_servo_overlay \
    && cmake --build /tmp/gen3-servo-build -j2 \
    && cmake --install /tmp/gen3-servo-build \
    && rm -rf /tmp/gen3-servo-src /tmp/gen3-servo-build /tmp/gen3-servo-overlay /tmp/moveit-servo-source.tar.gz

# Keep the installed Kortex 0.2.3 protocol/SDK version, but contain cyclic
# exceptions and stop reusing a failed session. No robot connection during build.
COPY docker/gen3-vive/kortex-overlay /tmp/gen3-kortex-overlay
RUN curl -fsSL --retry 3 \
      https://codeload.github.com/Kinovarobotics/ros2_kortex/tar.gz/50114b618e50321d9eb60a7406ffeb891d5fb175 \
      -o /tmp/gen3-kortex-source.tar.gz \
    && echo 'e4b6ef3981f17713ce1574f6b64a9b3fdab92f1430e6d31191ab27a4122313b6  /tmp/gen3-kortex-source.tar.gz' | sha256sum -c - \
    && mkdir -p /tmp/gen3-kortex-src \
    && tar -xzf /tmp/gen3-kortex-source.tar.gz -C /tmp/gen3-kortex-src --strip-components=2 \
       ros2_kortex-50114b618e50321d9eb60a7406ffeb891d5fb175/kortex_driver \
    && python3 /tmp/gen3-kortex-overlay/apply_overlay.py /tmp/gen3-kortex-src \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cmake -S /tmp/gen3-kortex-src -B /tmp/gen3-kortex-build \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/opt/gen3_kortex_overlay \
    && cmake --build /tmp/gen3-kortex-build -j2 \
    && cmake --install /tmp/gen3-kortex-build \
    && rm -rf /tmp/gen3-kortex-src /tmp/gen3-kortex-build /tmp/gen3-kortex-overlay /tmp/gen3-kortex-source.tar.gz

# Stop stale native commands inside ros2_control even when the Python publisher
# is killed. Keep the tested driver units/frame contract of PickNik 0.0.3.
COPY docker/gen3-vive/twist-overlay /tmp/gen3-twist-overlay
RUN curl -fsSL --retry 3 \
      https://codeload.github.com/PickNikRobotics/picknik_controllers/tar.gz/refs/tags/0.0.3 \
      -o /tmp/gen3-twist-source.tar.gz \
    && echo 'df5960f346d76e04b01a796991482af526c84bb6b16e3ce94ff45ebfc51631e5  /tmp/gen3-twist-source.tar.gz' | sha256sum -c - \
    && mkdir -p /tmp/gen3-twist-src \
    && tar -xzf /tmp/gen3-twist-source.tar.gz -C /tmp/gen3-twist-src --strip-components=2 \
       picknik_controllers-0.0.3/picknik_twist_controller \
    && python3 /tmp/gen3-twist-overlay/apply_overlay.py /tmp/gen3-twist-src \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cmake -S /tmp/gen3-twist-src -B /tmp/gen3-twist-build \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/opt/gen3_twist_overlay \
    && cmake --build /tmp/gen3-twist-build -j2 \
    && cmake --install /tmp/gen3-twist-build \
    && rm -rf /tmp/gen3-twist-src /tmp/gen3-twist-build /tmp/gen3-twist-overlay /tmp/gen3-twist-source.tar.gz

# Focused A/B hold window, plus a virtual display for hardware-free UI tests.
RUN apt-get update && apt-get install -y --no-install-recommends python3-tk xvfb xauth libxtst6 \
    && rm -rf /var/lib/apt/lists/*

COPY docker/gen3-vive/entrypoint.sh /ros_entrypoint_wuji.sh
RUN chmod +x /ros_entrypoint_wuji.sh

WORKDIR /workspace/wuji-retargeting
COPY . /workspace/wuji-retargeting
ENTRYPOINT ["/ros_entrypoint_wuji.sh"]
CMD ["bash"]
