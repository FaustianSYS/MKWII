#!/usr/bin/env bash
# Build the FlightSimROS2 UE5 plugin.
# Step 1: build the C wrapper library (system toolchain, full RTTI).
# Step 2: build the UE5 plugin (UE5 clang, no RTTI, only C header from step 1).
#
# Usage: ./scripts/build_ue5_plugin.sh [--launch]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UE5_ROOT="${UE5_ROOT:-$HOME/UnrealEngine}"
PROJECT="$REPO_ROOT/ue5/FlightSimUE/FlightSimUE.uproject"
ROS2_WS="$REPO_ROOT/ros2"
PYTHON_EXEC="${PYTHON_EXECUTABLE:-/usr/bin/python3}"

# ROS 2 setup.bash uses AMENT_TRACE_SETUP_FILES before initialising it.
set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$ROS2_WS/install/debug/setup.bash" ]] && source "$ROS2_WS/install/debug/setup.bash"
set -u

# Keep Anaconda or other user Python environments from shadowing ROS 2's
# Python packages during colcon/ament configuration.
export PATH="/usr/bin:/opt/ros/jazzy/bin:${PATH}"

# ── Step 1: build the C wrapper library ───────────────────────────────────
echo "=== Step 1: Building C wrapper library (flightsim_ue5_ros2_bridge) ==="

cd "$ROS2_WS"
colcon build \
    --packages-select flightsim_ue5_ros2_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release "-DPython3_EXECUTABLE=${PYTHON_EXEC}" \
    --install-base install/debug

echo "Wrapper library built."
echo ""

# Source overlay so UBT can resolve the library path if needed.
set +u
source "$ROS2_WS/install/debug/setup.bash"
set -u

# ── Step 2: build the UE5 plugin ──────────────────────────────────────────
echo "=== Step 2: Building UE5 plugin (FlightSimROS2) ==="

cd "$UE5_ROOT"
"$UE5_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" \
    FlightSimUEEditor \
    Linux \
    Development \
    "$PROJECT" \
    -plugin="$REPO_ROOT/ue5/FlightSimUE/Plugins/FlightSimROS2/FlightSimROS2.uplugin" \
    -WaitMutex \
    -FromMsBuild

echo ""
echo "=== Build complete ==="

if [[ "${1:-}" == "--launch" ]]; then
    echo "=== Launching UnrealEditor ==="
    BRIDGE_LIB="$ROS2_WS/install/debug/flightsim_ue5_ros2_bridge/lib"
    FMSGS_LIB="$ROS2_WS/install/debug/flightsim_msgs/lib"
    LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$BRIDGE_LIB:$FMSGS_LIB:${LD_LIBRARY_PATH:-}" \
    "$UE5_ROOT/Engine/Binaries/Linux/UnrealEditor" "$PROJECT" &
    echo "UnrealEditor launched (PID $!)"
fi
