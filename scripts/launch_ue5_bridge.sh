#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"
flightsim_resolve_build_profile

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${FLIGHTSIM_ROS2_INSTALL_DIR}/setup.bash"
set -u

exec ros2 launch flightsim_ros air_defense_ue5.launch.py "$@"
