ARG BASE_IMAGE=px4io/px4-sitl-gazebo-ros2:latest
FROM ${BASE_IMAGE}

LABEL org.opencontainers.image.title="neurovector-sim"
LABEL org.opencontainers.image.description="PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy + RViz2 + Foxglove"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Use the nearby Ubuntu mirror for the headless simulation runtime.
RUN find -L /etc/apt -type f \( -name '*.sources' -o -name '*.list' \) -exec sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|https://kr.archive.ubuntu.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|https://kr.archive.ubuntu.com/ubuntu|g' \
    -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.aliyun.com/ros2/ubuntu|g' \
    -e '/^deb-src/d' -e 's/Types: deb deb-src/Types: deb/g' {} +

# 기성 이미지에 없는 시각화/브릿지 도구 추가
RUN apt-get -o Acquire::Languages=none -o Acquire::IndexTargets::deb::DEP-11::DefaultEnabled=false update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-tf2-tools \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-xacro \
        jq \
        tmux \
        vim \
        iproute2 \
        iputils-ping \
        net-tools \
        python3-pip \
        python3-opencv \
        x11vnc \
        novnc \
        websockify \
        xterm \
        fluxbox \
        wmctrl \
        xvfb \
        xauth \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# 사용자 ROS 2 워크스페이스 (호스트에서 마운트)

RUN mkdir -p /ros2_ws/src

# 대화형 셸 편의 설정
COPY bashrc.append /tmp/bashrc.append
RUN cat /tmp/bashrc.append >> /root/.bashrc && rm /tmp/bashrc.append

COPY entrypoint.sh /usr/local/bin/nv-entrypoint.sh
RUN chmod +x /usr/local/bin/nv-entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/usr/local/bin/nv-entrypoint.sh"]
CMD ["bash", "-l"]
