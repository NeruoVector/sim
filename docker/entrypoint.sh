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

# 호스트의 sim/models 를 Gazebo 리소스 경로에 추가 (기존 경로는 유지)
if [ -d /models ]; then
    model_paths="/models"
    for dir in /models/drones /models/environments /models/props; do
        [ -d "$dir" ] && model_paths="$model_paths:$dir"
    done
    export GZ_SIM_RESOURCE_PATH="$model_paths${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
fi

exec "$@"
