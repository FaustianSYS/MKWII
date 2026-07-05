#!/usr/bin/env bash
# Launch UnrealEditor with the FlightSimUE project and ROS 2 libraries on the path.
# The FlightSimROS2 plugin will auto-start its spin thread once a level is loaded.
#
# Usage: ./scripts/launch_ue5_ros2.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UE5_ROOT="${UE5_ROOT:-$HOME/UnrealEngine}"
PROJECT="$REPO_ROOT/ue5/FlightSimUE/FlightSimUE.uproject"
ROS2_LIB="/opt/ros/jazzy/lib"
BRIDGE_LIB="$REPO_ROOT/ros2/install/debug/flightsim_ue5_ros2_bridge/lib"
FMSGS_LIB="$REPO_ROOT/ros2/install/debug/flightsim_msgs/lib"

# Source ROS 2 env so RMW middleware is discoverable.
# ROS 2 setup.bash uses AMENT_TRACE_SETUP_FILES before setting it; disable -u.
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$REPO_ROOT/ros2/install/debug/setup.bash" ]] && \
    source "$REPO_ROOT/ros2/install/debug/setup.bash"
set -u

export LD_LIBRARY_PATH="$ROS2_LIB:$BRIDGE_LIB:$FMSGS_LIB:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "LD_LIBRARY_PATH includes ROS 2: $ROS2_LIB"
echo "LD_LIBRARY_PATH includes FlightSim ROS libs: $BRIDGE_LIB:$FMSGS_LIB"
echo "Launching: $UE5_ROOT/Engine/Binaries/Linux/UnrealEditor"

exec "$UE5_ROOT/Engine/Binaries/Linux/UnrealEditor" "$PROJECT" "$@"
