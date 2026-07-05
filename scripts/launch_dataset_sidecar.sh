#!/usr/bin/env bash
# Phase 0 Mode A — FlightSim truth publisher + JSON frame logger for Isaac dataset sidecar.
#
# Usage:
#   ./scripts/launch_dataset_sidecar.sh
#   OUTPUT_DIR=datasets/my_run ./scripts/launch_dataset_sidecar.sh
#
# Terminal 2 (when Isaac Sim is installed):
#   python3 tools/isaac/isaac_dataset_sidecar.py --output "$OUTPUT_DIR"

set -euo pipefail

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

OUTPUT_DIR="${OUTPUT_DIR:-${ROOT}/datasets/run_$(date +%Y%m%d_%H%M%S)}"
export FLIGHTSIM_ROOT="${ROOT}"

mkdir -p "${OUTPUT_DIR}/frames"

cleanup() {
  if [[ -n "${LOGGER_PID:-}" ]] && kill -0 "${LOGGER_PID}" 2>/dev/null; then
    kill "${LOGGER_PID}" 2>/dev/null || true
    wait "${LOGGER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "==> Dataset sidecar (Mode A)"
echo "    Truth topics: /flightsim/scene_state, missile_state, target_state"
echo "    Output:       ${OUTPUT_DIR}"
echo ""

PYTHON_EXEC="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
"${PYTHON_EXEC}" "${ROOT}/tools/isaac/truth_logger.py" \
  --output "${OUTPUT_DIR}" \
  --wait-complete &
LOGGER_PID=$!

sleep 0.5

echo "==> Starting engagement truth publisher..."
ros2 launch flightsim_ros air_defense_dataset.launch.py &
LAUNCH_PID=$!

wait "${LOGGER_PID}"
echo "==> Truth logger finished (${OUTPUT_DIR})"

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
fi
