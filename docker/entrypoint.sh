#!/usr/bin/env bash
# NeuroVector 시뮬레이션 컨테이너 엔트리포인트.
# 기성 이미지의 ros2-entrypoint.sh 와 동일하게 ROS/워크스페이스를 소싱한 뒤,
# 호스트에서 마운트한 /ros2_ws 오버레이가 있으면 그 위에 덮어쓴다.
set -e

source /usr/local/lib/px4/network.sh
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/local_setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /ros2_ws/install/setup.bash
fi

exec "$@"
