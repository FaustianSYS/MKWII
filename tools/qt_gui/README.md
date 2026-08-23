# FlightSim Qt tactical GUI

C++ Qt5 + OpenGL app for **3D track visualization and interactive testing** of engagement truth topics. A PyQt5 2D fallback remains under `tools/qt_gui/`.

## Role

| UI | Purpose |
|----|---------|
| **C++ Qt 3D GUI** (`flightsim_qt_gui`) | Local OpenGL tracks (missile/target), LOS, metrics |
| PyQt5 GUI (`tools/qt_gui`) | Lightweight 2D NED map fallback |
| Web GUI (`gui/`) | Browser tactical display via WebSocket bridge |
| UE5 | Photoreal monitoring + seeker camera |

## Requirements

- ROS 2 Jazzy overlay built (`./scripts/build_ros2.sh`)
- `sudo apt install qtbase5-dev libqt5opengl5-dev`
- Display available (`DISPLAY` or Wayland)

## Run

```bash
# One-shot: engagement truth publisher + C++ Qt 3D window
./scripts/launch_qt_gui.sh

# Attach GUI only to an already-running stack
source ros2/install/debug/setup.bash   # or your FLIGHTSIM profile install
ros2 launch flightsim_qt_gui qt_tactical.launch.py start_engagement:=false

# Python 2D fallback
USE_PYTHON_GUI=1 ./scripts/launch_qt_gui.sh
```

## Controls

- **LMB drag** — orbit camera
- **RMB / MMB drag** — pan (disables follow)
- **Wheel** — zoom
- **Follow missile** — keep look-at on the interceptor
- **Clear trails / Reset camera / Mark event** — panel actions

Display frame: **+X North, +Y East, +Z Up** (NED Down flipped for viewing).

## Topics

- `/flightsim/scene_state`
- `/flightsim/missile_state`
- `/flightsim/target_state`
- `/flightsim/engagement_status`
