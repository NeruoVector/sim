#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/local_setup.bash
source /ros2_ws/install/local_setup.bash
set -u
package_prefix="$(ros2 pkg prefix nv_safety_bridge)"
# These domains contain only test publishers. Never inject fake PX4 state into SITL.
for scenario in accepted denied replay; do
    freeze=false
    if [ "$scenario" = accepted ]; then
        export ROS_DOMAIN_ID=73
        reject=false
    elif [ "$scenario" = denied ]; then
        export ROS_DOMAIN_ID=74
        reject=true
    else
        export ROS_DOMAIN_ID=75
        reject=false
        freeze=true
    fi
    bridge_log="/tmp/nv-safety-bridge-$scenario.log"
    "$package_prefix/lib/nv_safety_bridge/safety_bridge" >"$bridge_log" 2>&1 &
    bridge_pid=$!
    trap 'kill "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' EXIT
    if ! "$package_prefix/lib/nv_safety_bridge/bridge_integration_test" --ros-args -p reject_commands:="$reject" -p freeze_pose:="$freeze"; then
        cat "$bridge_log"
        exit 1
    fi
    kill "$bridge_pid"
    wait "$bridge_pid" || true
    trap - EXIT
done
