#!/usr/bin/env bash
set -euo pipefail
trap 'kill $(jobs -pr) 2>/dev/null || true' EXIT
restart_worker() {
  while true; do
    if "$@"; then
      echo "worker $1 exited; restarting" >&2
    else
      echo "worker $1 failed; restarting in 2 seconds" >&2
    fi
    sleep 2
  done
}
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99
chmod 1777 /tmp/.X11-unix
Xvfb :99 -screen 0 1600x900x24 -ac &
sleep 2
fluxbox &
restart_worker MicroXRCEAgent udp4 -p 8888 &
export PX4_PARAM_SDLOG_BACKEND=0
camera_fps="${NV_SCHOOL_CAMERA_FPS:-15.0}"
sed -i "/<sensor name=\"IMX214\"/,/<\/sensor>/s|<update_rate>[^<]*</update_rate>|<update_rate>${camera_fps}</update_rate>|" \
  /tmp/nv-school-models/OakD-Lite/model.sdf
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/px4-gazebo/lib/gz/plugins${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
gz sim --headless-rendering -r -s /worlds/school.sdf &
PX4_GZ_STANDALONE=1 px4-gazebo -d &
sleep 15
restart_worker ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/config/school_bridge.yaml &
python3 /scripts/school-live-node.py --ros-args -p use_sim_time:=true &
restart_worker python3 /scripts/school-depth-node.py --ros-args -p use_sim_time:=true &
restart_worker python3 /ros2_ws/src/nv_telemetry/nv_telemetry/camera_stream.py --ros-args -p drone_id:=school-drone-001 -p srt_host:=mediamtx -p frame_timeout_s:=3.0 -p fps:="$camera_fps" &
sed -e 's|<camera_pose>.*</camera_pose>|<camera_pose>18 -22 16 0 0.5 2.1</camera_pose>|' \
    /opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config > /tmp/school-gui.config
LIBGL_ALWAYS_SOFTWARE=1 nice -n 10 gz sim -g --gui-config /tmp/school-gui.config &
x11vnc -display :99 -noxdamage -forever -shared -viewonly -nopw -localhost -rfbport 5903 &
websockify --web=/usr/share/novnc 0.0.0.0:6083 localhost:5903 &
sleep 10
wmctrl -r Gazebo -e 0,0,0,1000,850 || true
(
  until [ -f /runtime/camera.jpg ]; do sleep 2; done
  ffmpeg -loglevel error -re -f image2 -pattern_type none -loop 1 -framerate 2 -i /runtime/camera.jpg -c:v copy -f mjpeg pipe:1 | \
    ffplay -loglevel warning -window_title 'Live Gazebo RGB camera' -left 1010 -top 20 -x 580 -y 350 -f mjpeg -
) &
xterm -fa 'DejaVu Sans Mono' -fs 10 -title 'Live simulator state' -geometry 78x25+1010+410 -e watch -n 1 'python3 -m json.tool /runtime/state.json; python3 -m json.tool /runtime/inference.json' &
wait -n
