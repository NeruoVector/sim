#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/local_setup.bash
source /ros2_ws/install/local_setup.bash
set -u
package_prefix="$(ros2 pkg prefix nv_safety_bridge)"
"$package_prefix/lib/nv_safety_bridge/safety_bridge" >/tmp/nv-safety-sitl.log 2>&1 &
bridge_pid=$!
trap 'kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' EXIT
if ! "$package_prefix/lib/nv_safety_bridge/sitl_probe"; then
    cat /tmp/nv-safety-sitl.log
    exit 1
fi
