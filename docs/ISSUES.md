# Issue → files checklist

Audit of every open/closed issue against the repo, with what was missing and what now covers it.
"Firmware core" = pure C++ in `firmware/waterdroid_esp32/src/` (unit-tested on a PC) used by the
integrated sketch and by the XOD node.

| Issue | Owner | Hardware | Firmware / XOD | Node-RED | Simulation (ROS 2 / Gazebo) |
|-------|-------|----------|----------------|----------|-----------------------------|
| **#1** Intake & raw water | @meekness2006 | intake pump, pre-filter, hose, Stage 1 | core `IntakeController` (max-run cutoff, dry-run check, overflow block) · XOD `xod/waterdroid/intake-controller` — **was missing** | `river_pump_flow.json` (+ Manual Water Intake button, fill gauge — **were missing**) · central dashboard *Water train* tab | `water_processor` CH1 (dry-run when not at the river, auto-stop at batch) · URDF intake pump + hose + pre-filter |
| **#3** Chemical dosing | | alum + chlorine pumps | core `DosingPump` (mL from real relay-ON time) · XOD `dosing-pump` — **was missing** | `chemical_dosing_flow.json` fixed: Raspberry-Pi GPIO → ESP32-metered `rover/dosing/cmd`, 200 L → 400 mL scale, dosing panel with mL totals added | `water_processor` CH2/CH3 · URDF dosing pumps |
| **#5** Filtration & flow | Dann | Stage 2 column, **one water-flow sensor** (no level sensors) → `FlowMeter` + `VolumeModel` estimate the tanks (XOD `flow-meter`, `volume-model`) | `filtration_monitor-1.ino` (pins aligned) · core `FiltrationMonitor` · XOD `level-monitor` — **XOD was missing** | `flow.json` (unchanged, already on `rover/filtration/*`) · central dashboard | `water_processor` gravity model + overflow latch · URDF column |
| **#6** Servo arm → now the **intake** arm (delivery nozzle fixed) | Tapiwa | SG90 lowers / raises the intake hose + pump | `robotic_arm.ino` (ESP32Servo/GPIO13 added; pin 9 is flash on ESP32) · core `ServoArm` · XOD `servo-arm` — **XOD was missing** | `robotic_arm_flow.json`: commands only reached a debug node → now MQTT `rover/arm/cmd` + live angle gauge | `arm_controller` + revolute `arm_servo_joint` with JointPositionController — **arm was not modelled** |
| **#7** Navigation | Tafara | GPS / compass / obstacle sensors **not available** → **fixed route of fixed coordinates, dead reckoning** | core `RouteNavigator` in `src/nav_cores.h` (route table, calibration, re-sync on "Arrived") · XOD `route-navigator` | *Drive & Map* tab: route on the map, Auto/Manual, calibrate buttons | `mission:=route` (dead reckoning from /cmd_vel, drift logged vs ground truth) |
| **#8** Clean water delivery | | Stage 3 + delivery pump, **fixed nozzle** | core `DeliveryController` (at-homestead interlock) · XOD `delivery-controller` — **was missing** | central dashboard *Arm & Delivery* tab — **was missing** | `water_processor` CH4 with arm interlock · URDF delivery pump |
| **#9** 4WD propulsion | | 4 × TT motors + L298N | core `DriveController` (skid steer, slew, min duty, dead-man, inhibit) · XOD `l298n-drive` — **was missing** | `rover_teleop_flow.json`: missing ui_group added, buttons → MQTT `rover/drive/cmd` (were debug only) | URDF now **4WD** with 65 mm wheels (was 2WD + caster), DiffDrive on 4 joints |
| **#10** Power | | power bank + 4-ch relay | `power-management/` (complete) — relay map corrected: **CH2/CH3 were "left/right motor"**, but the motors are on the L298N; the 4 relays are the 4 pumps | `power_management_flow.json` relabelled | battery/rail estimate in `status_monitor` |
| **#11** Central controller (closed) | | ESP32 | core `MissionCore` + `waterdroid_esp32.ino` serial bridge — **firmware was missing** · XOD `mission-fsm` | `esp32_central_controller_serial_bridge.json` (works with the new firmware's serial frames) | `mission_navigator_gt` publishes the same state names |
| **#12** Comms & dashboard | Tadiwa | ESP32 Wi-Fi | MQTT client in `waterdroid_esp32.ino` · XOD `telemetry-json` — **was missing** | `node-red/waterdroid_dashboard.json` central SCADA: all feeds, system E-STOP (retained), CSV mission log — **was missing** | `status_monitor` mirrors the sim onto the same MQTT topics |

## Repo-wide problems found and fixed

* `waterdroid_control` had no `setup.cfg`, so `ros2 run` could not find any node; `water_processor.py`,
  `status_monitor.py` and `params.yaml` were empty files.
* `simulation.launch.py` loaded the world from `/home/tyler/water_robot_ws/...` and passed raw xacro to
  robot_state_publisher; `waterdroid_gazebo` was not a ROS package; package.xml files had no run dependencies.
* `mission_navigator_gt.py` listened to `/ground_truth/odom`, which nothing published, and was not an executable.
* `mission_navigator.py` used waypoints from an older world and nested quotes in f-strings (Python ≥ 3.12 only).
* Each member sketch claimed its own pins on the one ESP32 (relay CH1 on GPIO26 in two sketches, motors on
  relays, servo on a flash pin). Now one pin map: `firmware/waterdroid_esp32/src/rover_pins.h`.
* MQTT namespaces differed (`rover/...` vs `water/...`); now all `rover/...` (docs/TOPICS.md).
* `chemical_dosing_flow.json.json` / `rover-teleop.json.json` double extensions removed.

## Still open (needs the team)

1. **Measure the real route** and put it in `DEFAULT_ROUTE`, then calibrate `NAV_SPEED_MPS` / `NAV_TURN_DPS`
   (docs/HARDWARE.md). Wheel encoders would cut the drift a lot if the TT motor kit has the slotted discs.
2. **Flow sensor:** fit it on the intake line with the 10k/20k divider, calibrate pulses per litre; time the
   filter column for `FILTER_ML_PER_S`.
3. **Calibrate** level sensors, pump rates and the rail divider (docs/HARDWARE.md).
4. **Flash and bench test** `waterdroid_esp32.ino` — it was syntax-checked against stubs and its logic is unit
   tested, but it has not run on the physical board yet.
5. **Open the XOD project in XOD IDE** to confirm the generated nodes load (checked with a C++ stub only).
