# UE5 ROS 2 bridge contract

FlightSim publishes scene truth for rendering; UE5 publishes seeker camera frames; the C++ vision node returns track estimates for guidance.

## Coordinate frame

- All FlightSim topics use **NED** (`frame_id: ned`).
- North = +X, East = +Y, Down = +Z (altitude is `-z`).
- UE5 actors should apply a fixed transform from NED to Unreal world space (left-handed Z-up). Document the mapping in the UE plugin; a common mapping is:
  - UE X = NED North
  - UE Y = NED East
  - UE Z = -NED Down

Attitudes are body-to-NED quaternions `[w, x, y, z]` (`SceneEntity.attitude_wxyz`).

## Topics

| Topic | Type | Direction | Notes |
|-------|------|-----------|-------|
| `/flightsim/scene_state` | `flightsim_msgs/SceneState` | FlightSim → UE5 | Spawn/update missile + shahed + depot each sim step |
| `/flightsim/missile_state` | `flightsim_msgs/MissileState` | FlightSim → UE5 | Debug overlay, attitude for camera mount |
| `/flightsim/target_state` | `flightsim_msgs/TargetState` | FlightSim → UE5 | Truth **Shahed** state (missile guidance target) |
| `/flightsim/seeker_camera/image` | `sensor_msgs/Image` | UE5 → vision node | `mono8` or `8UC1`, boresight aligned with missile body +X |
| `/flightsim/seeker_camera/camera_info` | `sensor_msgs/CameraInfo` | UE5 → vision node | Optional; intrinsics should match FOV below |
| `/flightsim/seeker_track` | `flightsim_msgs/SeekerTrack` | vision node → FlightSim | Estimated LOS in NED for guidance |

Launch with:

```bash
ros2 launch flightsim_ros air_defense_ue5.launch.py use_vision_seeker:=true
```

This starts engagement, vision, and `flightsim_ue5_bridge_node` (software stand-in). For a real Unreal process, stop the bridge node and run your UE5 ROS plugin instead — same topics and message types.

Alternate (engagement + vision only; you must supply camera frames from UE5):

```bash
ros2 launch flightsim_ros air_defense_vision.launch.py use_vision_seeker:=true
```

## Camera model

Default seeker parameters (override via ROS params on `flightsim_vision_node`):

- Resolution: 640 × 480
- FOV: 0.52 rad azimuth and elevation (match `MissileAttributes.optical_window`)
- Boresight: camera optical axis aligned with missile body +X
- Pixel mapping: linear angle from image center (see `CameraModel::pixel_to_bearing_rad`)

`CameraInfo.K` should be consistent with:

```
fx = (width / 2) / tan(fov_az / 2)
fy = (height / 2) / tan(fov_el / 2)
cx = width / 2
cy = height / 2
```

## UE5 responsibilities

1. Subscribe to `/flightsim/scene_state` and place/update actors for each `SceneEntity`.
2. Parent an EO/IR camera to the missile actor using `attitude_wxyz`.
3. Publish grayscale frames on `/flightsim/seeker_camera/image` at sim rate (100 Hz default).
4. Publish matching `CameraInfo` when intrinsics differ from the defaults above.

### Shahed-136 (inbound threat)

Import from [`assets/ue5/models/shahed_136/`](../../assets/ue5/models/shahed_136/README.md):

- **Primary:** `source/shahed_136.glb` (CC BY 4.0, ~3.5 m, nose +X)
- **Refresh:** `./scripts/fetch_shahed_136_model.sh`

Map `SceneEntity` with `type=shahed` / `id=shahed` / `model=shahed_136` to `BP_Shahed136`.

`TargetState` on `/flightsim/target_state` carries the same truth (`id=shahed`, `model=shahed_136`) for missile guidance and debug overlay.

### Depot (ground objective)

Static ground target the Shahed flies toward. See [`assets/ue5/models/depot/`](../../assets/ue5/models/depot/README.md).

Map `SceneEntity` with `type=depot` / `id=depot` / `model=depot` to `BP_Depot` (building/pad mesh at terrain height).

### Missile mesh (generic interceptor)

Import the low-poly interceptor mesh from [`assets/ue5/models/generic_missile/`](../../assets/ue5/models/generic_missile/README.md):

- **Primary:** `source/generic_missile.glb` (CC BY 3.0, Jarlan Perez / Google Poly, scaled 3.66 m, nose +X)
- **Alternate:** `source/generic_missile.obj`
- **Refresh:** `./scripts/fetch_generic_missile_model.sh`

Map `SceneEntity` with `type=missile` / `id=missile` to your interceptor Blueprint.

## Vision pipeline

`flightsim_vision_node` thresholds bright blobs, finds centroid, converts pixel offset to LOS in NED using missile attitude from `/flightsim/missile_state`, and publishes `/flightsim/seeker_track`.

When `use_vision_seeker:=true`, `flightsim_engagement_node` uses the track for proportional navigation; truth target state remains available for intercept scoring only.

## Dependencies

- ROS 2 Jazzy
- `sensor_msgs` for camera images
- Core FlightSim built with `-DFLIGHTSIM_BUILD_VISION=ON`

Native OpenCV is optional; the default processor uses a lightweight threshold/centroid pass suitable for CI synthetic frames.
