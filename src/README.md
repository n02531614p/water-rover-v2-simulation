# Waterdroid ROS 2 workspace packages

| Package | Contents |
|---------|----------|
| `waterdroid_description` | `urdf/waterdroid.urdf.xacro` — 4WD two-deck rover, 3-stage treatment train, 4 pumps, SG90 arm, Gazebo systems; `launch/display.launch.py`; RViz config |
| `waterdroid_gazebo` | `worlds/v2_village.sdf` (river bank x=10, offload bays (6,3) (6,-3) (-3,-3)) and `worlds/village.sdf` (5 homesteads) |
| `waterdroid_control` | `mission_navigator_gt` (default, ground truth, full cycle) · `mission_navigator` (wheel odom waypoints) · `water_processor` (simulated pumps / tanks / dosing) · `arm_controller` · `status_monitor` (+ MQTT bridge to Node-RED) |
| `waterdroid_bringup` | `launch/simulation.launch.py`, `config/params.yaml` |

Legacy (not used by the launch files, kept for reference): `water_robot_sim/` (first prototype with an
on-board boiler), `urdf/v2_rover.urdf.xacro` and `scripts/v2_fsm.py` (2WD telescopic-arm concept),
`waterdroid_description/urdf/waterdroid.urdf.xacro.2wd.bak`.

Topic map: `../docs/TOPICS.md`.
