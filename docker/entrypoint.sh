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
    for dir in /models/drones /models/environments /models/props /worlds; do
        [ -d "$dir" ] && model_paths="$model_paths:$dir"
    done
    export GZ_SIM_RESOURCE_PATH="$model_paths${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
fi

if [ "${NV_SCHOOL_SCENE:-0}" = 1 ]; then
    # Reuse the existing PX4 aircraft and the same fixture as the generated scene.
    school_launch="$(python3 - <<'PY'
import json
with open('/fixtures/school.json', encoding='utf-8') as file:
    data = json.load(file)
print(data['drone']['sim_model'], ','.join(str(v) for v in [*data['takeoff_enu'], 0, 0, 0]))
PY
)"
    read -r PX4_SIM_MODEL PX4_GZ_MODEL_POSE <<< "$school_launch"
    export PX4_SIM_MODEL PX4_GZ_MODEL_POSE
    # The packaged px4-gazebo wrapper resets PX4_GZ_WORLDS on every launch.
    px4_install="$(dirname "$(dirname "$(readlink -f "$(command -v px4-gazebo)")")")"
    cp /worlds/school.sdf "$px4_install/share/gz/worlds/school.sdf"
    # School software-rendering profile; preserve the stock sensor geometry/FOV.
    mkdir -p /tmp/nv-school-models
    cp -a "$px4_install/share/gz/models/OakD-Lite" /tmp/nv-school-models/
    python3 - <<'PY'
import xml.etree.ElementTree as ET
path = '/tmp/nv-school-models/OakD-Lite/model.sdf'
tree = ET.parse(path)
for sensor in tree.findall('.//sensor'):
    sensor.find('update_rate').text = '5'
    if sensor.get('type') == 'camera':
        sensor.find('camera/image/width').text = '640'
        sensor.find('camera/image/height').text = '360'
tree.write(path, encoding='utf-8', xml_declaration=True)
PY
    # x500_depth lacks the ground-truth publisher present in x500_vision.
    python3 - "$px4_install/share/gz/models/x500_depth/model.sdf" <<'PY'
import sys
import xml.etree.ElementTree as ET
path = sys.argv[1]
tree = ET.parse(path)
model = tree.find('model')
if model.find("plugin[@name='gz::sim::systems::OdometryPublisher']") is None:
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-odometry-publisher-system', name='gz::sim::systems::OdometryPublisher')
    for name, value in [('dimensions', '3'), ('odom_frame', 'map'), ('robot_base_frame', 'x500_depth_0'), ('odometry_publish_frequency', '20')]:
        ET.SubElement(plugin, name).text = value
    tree.write(path, encoding='utf-8', xml_declaration=True)
PY
    export GZ_SIM_RESOURCE_PATH="/tmp/nv-school-models:$GZ_SIM_RESOURCE_PATH"
    # Both PX4 and ROS use simulated time; wall-time synchronization stalls slow SITL.
    export PX4_PARAM_UXRCE_DDS_SYNCT=0
fi

exec "$@"
