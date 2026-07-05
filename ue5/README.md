# UE5 co-simulation bridge

FlightSim ships a **software UE5 bridge** so the full vision loop works without Unreal Engine installed. When you have UE5, swap the bridge process for a Unreal project that implements the same ROS 2 contract.

## Software bridge (default)

`flightsim_ue5_bridge_node` is a ROS 2 process that:

1. Subscribes to `/flightsim/scene_state`
2. Syncs missile + Shahed entity poses (same contract as UE5)
3. Renders synthetic seeker frames and publishes `/flightsim/seeker_camera/image` + `camera_info`

Launch the full stack (engagement + vision + bridge):

```bash
./scripts/build_ros2.sh
./scripts/launch_ue5_bridge.sh
```

Or explicitly:

```bash
source ros2/install/debug/setup.bash
ros2 launch flightsim_ros air_defense_ue5.launch.py use_vision_seeker:=true
```

Verify:

```bash
ros2 topic hz /flightsim/seeker_camera/image
ros2 topic echo /flightsim/seeker_track --once
```

## Real UE5 process

Replace `flightsim_ue5_bridge_node` with Unreal Engine implementing [`../docs/ue5/ros2_bridge_contract.md`](../docs/ue5/ros2_bridge_contract.md):

| Responsibility | Topic |
|----------------|-------|
| Subscribe scene truth | `/flightsim/scene_state` |
| Publish seeker camera | `/flightsim/seeker_camera/image` (`mono8`, 640×480, 0.52 rad FOV) |
| Optional intrinsics | `/flightsim/seeker_camera/camera_info` |

Import meshes from [`../assets/ue5/models/`](../assets/ue5/models/).

NED → Unreal mapping (from contract):

- UE X = NED North
- UE Y = NED East
- UE Z = −NED Down

Attitudes are body→NED quaternions `[w, x, y, z]`.

## GUI

With ROS tactical GUI:

```bash
ros2 launch flightsim_ros tactical_gui.launch.py
# in another terminal, start the bridge stack:
./scripts/launch_ue5_bridge.sh
```

The **UE5 BRIDGE** panel shows **UE5 BRIDGE LIVE** when camera frames are flowing.
