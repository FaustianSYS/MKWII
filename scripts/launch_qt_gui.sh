#!/usr/bin/env bash
# Launch FlightSim engagement (dataset/truth mode) + C++ Qt 3D tactical GUI.
#
# Usage:
#   ./scripts/launch_qt_gui.sh
#   SCENARIO=air_defense ./scripts/launch_qt_gui.sh
#   USE_PYTHON_GUI=1 ./scripts/launch_qt_gui.sh   # fallback PyQt5 2D map

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/build_profile.sh"
flightsim_resolve_build_profile

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PYTHON_EXEC="${PYTHON_EXECUTABLE:-/usr/bin/python3}"
USE_PYTHON_GUI="${USE_PYTHON_GUI:-0}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${FLIGHTSIM_ROS2_INSTALL_DIR}/setup.bash"
set -u

cleanup() {
  if [[ -n "${ENG_PID:-}" ]] && kill -0 "${ENG_PID}" 2>/dev/null; then
    kill "${ENG_PID}" 2>/dev/null || true
    wait "${ENG_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

export QT_LOGGING_RULES="*.debug=false"

if [[ "${USE_PYTHON_GUI}" == "1" ]]; then
  if ! "${PYTHON_EXEC}" -c "from PyQt5.QtWidgets import QApplication" 2>/dev/null; then
    echo "PyQt5 not found for ${PYTHON_EXEC}." >&2
    echo "Install: sudo apt install python3-pyqt5" >&2
    exit 1
  fi
  echo "==> Qt tactical GUI (Python 2D fallback)"
  echo "    Engagement: air_defense_dataset.launch.py"
  echo "    GUI:        tools/qt_gui/tactical_gui.py"
  echo ""
  ros2 launch flightsim_ros air_defense_dataset.launch.py \
    scenario:="${SCENARIO:-air_defense}" &
  ENG_PID=$!
  sleep 0.5
  exec "${PYTHON_EXEC}" "${ROOT}/tools/qt_gui/tactical_gui.py"
fi

if ! command -v flightsim_qt_tactical_gui >/dev/null 2>&1 && \
   [[ ! -x "${FLIGHTSIM_ROS2_INSTALL_DIR}/flightsim_qt_gui/lib/flightsim_qt_gui/flightsim_qt_tactical_gui" ]]; then
  echo "C++ Qt GUI not found in ROS overlay." >&2
  echo "Build with: ./scripts/build_ros2.sh" >&2
  echo "Deps: sudo apt install qtbase5-dev libqt5opengl5-dev" >&2
  exit 1
fi

echo "==> Qt 3D tactical GUI (C++)"
echo "    Launch: flightsim_qt_gui qt_tactical.launch.py"
echo "    Orbit: LMB | Pan: RMB | Zoom: wheel | Follow missile toggle in panel"
echo ""

# Launch file starts engagement + GUI; do not use ENG_PID trap for the whole tree —
# ros2 launch owns child processes.
trap - EXIT INT TERM
exec ros2 launch flightsim_qt_gui qt_tactical.launch.py \
  scenario:="${SCENARIO:-air_defense}" \
  start_engagement:=true
