#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"

flightsim_resolve_build_profile

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}" >&2
  exit 1
fi

EXTRA_CMAKE_ARGS=("-DFLIGHTSIM_BUILD_VISION=ON")

echo "==> Building FlightSim core [${FLIGHTSIM_BUILD_PROFILE}] (${FLIGHTSIM_BUILD_DIR})"
cmake --preset "${FLIGHTSIM_CMAKE_PRESET}" -B "${FLIGHTSIM_BUILD_DIR}" "${EXTRA_CMAKE_ARGS[@]}"
cmake --build "${FLIGHTSIM_BUILD_DIR}"

echo "==> Building ROS 2 packages [${FLIGHTSIM_BUILD_PROFILE}]"
cd "${ROOT}/ros2"

export PATH="/usr/bin:/opt/ros/${ROS_DISTRO}/bin:${PATH}"
PYTHON_EXEC="${PYTHON_EXECUTABLE:-/usr/bin/python3}"

colcon build \
  --build-base "${FLIGHTSIM_ROS2_BUILD_DIR}" \
  --install-base "${FLIGHTSIM_ROS2_INSTALL_DIR}" \
  --cmake-args \
    "-DFLIGHTSIM_BUILD_DIR=${FLIGHTSIM_BUILD_DIR}" \
    "-DFLIGHTSIM_BUILD_PROFILE=${FLIGHTSIM_BUILD_PROFILE}" \
    "-DFLIGHTSIM_BUILD_VISION=ON" \
    "-DPython3_EXECUTABLE=${PYTHON_EXEC}" \
  --symlink-install

echo ""
echo "Profile:       ${FLIGHTSIM_BUILD_PROFILE}"
echo "Core build:    ${FLIGHTSIM_BUILD_DIR}"
echo "ROS 2 overlay: source ${FLIGHTSIM_ROS2_INSTALL_DIR}/setup.bash"
echo "Run:"
echo "  ros2 launch flightsim_ros air_defense_ue5.launch.py use_vision_seeker:=true"
