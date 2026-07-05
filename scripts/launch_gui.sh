#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FLIGHTSIM_BUILD_PROFILE="${FLIGHTSIM_BUILD_PROFILE:-debug}" \
FLIGHTSIM_BUILD_GUI=ON \
  "${ROOT}/scripts/build_gui.sh"

# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"
flightsim_resolve_build_profile

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

set +u
source "${FLIGHTSIM_ROS2_INSTALL_DIR}/setup.bash"
set -u

echo ""
echo "Profile: ${FLIGHTSIM_BUILD_PROFILE}"
echo "Press Ctrl+C to stop"
echo ""

ros2 launch flightsim_ros engagement_gui_full.launch.py
