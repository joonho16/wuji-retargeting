# syntax=docker/dockerfile:1
# Ubuntu x86_64 desktop host target; Python 3.11 has Linux wheels for
# pyrealsense2, MuJoCo, Pinocchio, and the Wuji packages used here.
FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MUJOCO_GL=glfw

# MANUS Linux SDK prerequisites are installed here, not vendored into the
# repository. Official MANUS docs list separate Core Integrated and Core Remote
# dependency sets; this image includes the superset so the SDK examples and a
# tracked MANUS->Wuji UDP bridge can be built/run from a mounted SDK directory.
# See: https://docs.manus-meta.com/latest/Plugins/SDK/Linux/
ARG MANUS_BUILD_GRPC=1
ARG MANUS_GRPC_RELEASE_TAG=v1.28.1
ENV MANUS_SDK_ROOT=/opt/manus/ManusSDK_v3.1.1 \
    MANUS_GRPC_RELEASE_TAG=${MANUS_GRPC_RELEASE_TAG} \
    LD_LIBRARY_PATH=/opt/manus/ManusSDK_v3.1.1/SDKMinimalClient_Linux/ManusSDK/lib:/opt/manus/ManusSDK_v3.1.1/SDKClient_Linux/ManusSDK/lib:/opt/manus/ManusSDK_v3.1.1/ROS2/ManusSDK/lib

RUN apt-get update && apt-get install -y --no-install-recommends \
        bash \
        build-essential \
        ca-certificates \
        cmake \
        gcc-11 \
        g++-11 \
        git \
        iptables \
        pkg-config \
        libtool \
        libzmq3-dev \
        libusb-1.0-0-dev \
        zlib1g-dev \
        libudev-dev \
        libncurses5-dev \
        gdb \
        openssh-server \
        autoconf \
        automake \
        curl \
        libdbus-1-3 \
        libegl1 \
        libfontconfig1 \
        libfreetype6 \
        libgl1 \
        libgl1-mesa-dri \
        libgles2 \
        libglib2.0-0 \
        libglfw3 \
        libglvnd0 \
        libgomp1 \
        libice6 \
        libsm6 \
        libudev1 \
        libusb-1.0-0 \
        libx11-6 \
        libxcb1 \
        libxcb-xinerama0 \
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxkbcommon0 \
        libxkbcommon-x11-0 \
        libxrandr2 \
        libxrender1 \
        udev \
        usbutils \
        v4l-utils \
    && mkdir -p /var/run/sshd /opt/manus \
    && echo '# HIDAPI/libusb' > /etc/udev/rules.d/70-manus-hid.rules \
    && echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"' >> /etc/udev/rules.d/70-manus-hid.rules \
    && echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"' >> /etc/udev/rules.d/70-manus-hid.rules \
    && echo '# HIDAPI/hidraw' >> /etc/udev/rules.d/70-manus-hid.rules \
    && echo 'KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"' >> /etc/udev/rules.d/70-manus-hid.rules \
    && if [ "$MANUS_BUILD_GRPC" = "1" ]; then \
         export CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11 && \
         mkdir -p /var/local/git && \
         git clone -b "$MANUS_GRPC_RELEASE_TAG" https://github.com/grpc/grpc /var/local/git/grpc && \
         cd /var/local/git/grpc && \
         git submodule update --init --recursive && \
         cd /var/local/git/grpc/third_party/protobuf && \
         ./autogen.sh && ./configure --enable-shared && \
         make -j"$(nproc)" && make install && make clean && ldconfig && \
         cd /var/local/git/grpc && \
         make -j"$(nproc)" && make install && make clean && ldconfig && \
         rm -rf /var/local/git/grpc; \
       else \
         echo "MANUS_BUILD_GRPC=0: installed apt-level MANUS deps; set MANUS_BUILD_GRPC=1 for official gRPC ${MANUS_GRPC_RELEASE_TAG} source build."; \
       fi \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/wuji-retargeting
COPY . .

# This repo's RealSense path still calls mp.solutions.hands. Newer mediapipe
# wheels removed that legacy API, while mediapipe 0.10.21's declared numpy<2
# dependency conflicts with pin/cmeel-boost on Python 3.11. Keep the resolved
# robotics stack intact and replace only the mediapipe wheel plus its minimal
# legacy import-time dependencies. grpcio-tools is kept on a protobuf<5-compatible
# release because avp_stream declares it in metadata.
RUN python -m pip install --upgrade pip setuptools wheel \
    && python -m pip install \
        "wujihandpy==1.8.0" \
        "avp_stream==2.51" \
        -e ".[realsense,tuning]" \
    && python -m pip install \
        "attrs>=19.1.0" \
        "protobuf>=4.25.0,<5" \
        "grpcio==1.62.3" \
        "grpcio-tools==1.62.3" \
    && python -m pip install --force-reinstall --no-deps "mediapipe==0.10.21" \
    && python scripts/docker_smoke.py

CMD ["python", "scripts/docker_smoke.py"]
