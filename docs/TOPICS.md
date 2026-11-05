# Topic map — ESP32 ⇄ Node-RED ⇄ ROS 2 simulation

One MQTT namespace, `rover/...`, is shared by the ESP32 firmware (`firmware/waterdroid_esp32`), every
Node-RED flow and the Gazebo simulation bridge (`status_monitor` with `mqtt:=true`). The dashboard
therefore works the same against the real rover and the simulation. Status payloads are JSON.

## MQTT

| Topic | Dir (rover view) | Payload | Issue |
|-------|------------------|---------|-------|
| `rover/estop` | in + out, **retained** | `1` = E-STOP (all pumps + motors off, arm frozen), `0` = release | #12 |
| `rover/online` | out, retained (last-will `0`) | `1` / `0` | #12 |
| `rover/telemetry` | out, 1 Hz | full frame: state, state_id, battery_v, s1/s2/s3, relays, arm, drive, alum_ml, chlorine_ml, delivered_ml, health, power, rssi | #12 |
| `rover/state` | out, 1 Hz | `{"state":"STATE_PUMPING","state_id":2,"battery_v":5.02}` | #11 |
| `rover/mission/cmd` | in | `start` `pause` `resume` `abort` `arrived_river` `arrived_home` `arrived_depot` | #11 |
| `rover/intake/status` | out | `{"pump":true,"state":"PUMPING","level_pct":42,"run_s":12.3,"flow_ml_s":21.5,"batch_ml":260,"target_ml":950,"total_ml":1210,"fault":""}` (flow sensor) | #1 |
| `rover/intake/cmd` | in | `start` `stop` `reset` (clears DRY_RUN / TIMEOUT) | #1 |
| `rover/dosing/status` | out | `{"alum_on":false,"chlorine_on":true,"alum_total_ml":3.0,"chlorine_total_ml":0.6,"alum_stock_ml":247,...}` | #3 |
| `rover/dosing/cmd` | in | `{"pump":"alum","ml":3}` · `{"pump":"chlorine","ml":0.6}` · `stop` · `refill alum` | #3 |
| `rover/filtration/status` | out | `{"s1":40,"s2":20,"s3":10,"intake":true,"flow":0.02,"health":"HEALTHY"}` | #5 |
| `rover/filtration/cmd` | in | `pause` · `resume` · `auto` | #5 |
| `rover/arm/status` | out | `{"angle":90,"target":90,"moving":false,"deployed":true,"stowed":false}` | #6 |
| `rover/arm/cmd` | in | `EXTEND` · `RETRACT` · `ANGLE:0-90` | #6 |
| `rover/nav/pose` | out | `{"x":6.0,"y":3.0,"yaw":3.14,"target":[6,3],"next":"HOME","phase":"DRIVE","leg_left":1.2,"point":2,"auto":true,"source":"dead_reckoning"}` | #7 |
| `rover/nav/cmd` | in | `auto` (follow the fixed route) · `manual` (D-pad + Arrived buttons) · `reset` (pose = depot) · `calib_forward` · `calib_turn` | #7 |
| `rover/delivery/status` | out | `{"pump":false,"state":"DONE","delivered_ml":317,"delivered_total_ml":951,"deliveries":3,"clean_ml":0}` | #8 |
| `rover/delivery/cmd` | in | `{"ml":300}` · `stop` (pumps only with the arm deployed) | #8 |
| `rover/drive/status` | out | `{"left":200,"right":200,"inhibit":false}` | #9 |
| `rover/drive/cmd` | in | `forward` `reverse` `left` `right` `stop` or `{"l":-255..255,"r":-255..255}`; 0.6 s dead-man | #9 |
| `rover/power/status` | out | `{"mode":"ACTIVE","rail_v":5.02,"rail":"OK","ch":[1,0,0,0],"want":[1,0,0,0],"brownouts":0,...}` | #10 |
| `rover/power/cmd` | in | `active` `sleep` `emergency` `reset` | #10 |
| `rover/power/req/1..4` | in | `1` / `0` (standalone `power_management.ino` only) | #10 |

### Serial bridge (Issue #11 flow `esp32_central_controller_serial_bridge.json`)

The firmware prints one line per second at 115200 baud:
`{"state_id":2,"state":"STATE_PUMPING","battery_v":5.02,"estop":0}` and accepts
`{"cmd":"SET_STATE","state":1}` (start), `{"cmd":"ESTOP","state":5}` and the plain words
`start pause resume abort estop reset arrived_river arrived_home arrived_depot nav_auto nav_manual nav_reset calib_forward calib_turn EXTEND RETRACT ANGLE:n`.

Mission state ids: `0 IDLE · 1 NAV_RIVER · 2 INTAKE_DOSE · 3 NAV_HOMESTEAD · 4 DELIVER_WATER · 5 EMERGENCY_STOP`.

## ROS 2 (simulation)

| Topic | Type | From → To |
|-------|------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | mission / teleop → Gazebo DiffDrive (4 wheels) |
| `/odom` | nav_msgs/Odometry | DiffDrive wheel odometry (drifts on skid steer) |
| `/ground_truth/odom` | nav_msgs/Odometry | Gazebo OdometryPublisher → mission, water_processor, status_monitor |
| `/tf`, `/joint_states` | tf2_msgs/TFMessage, sensor_msgs/JointState | Gazebo → robot_state_publisher / RViz |
| `/arm_servo/cmd_pos` | std_msgs/Float64 (rad) | arm_controller → Gazebo JointPositionController |
| `/waterdroid/mission_state` | std_msgs/String (JSON) | mission_navigator(_gt) |
| `/waterdroid/mission/cmd` | std_msgs/String | `start` `pause` `resume` `abort` |
| `/waterdroid/cmd` | std_msgs/String | `intake:start` `intake:stop` `dose_alum[:ml]` `dose_chlorine[:ml]` `deliver:ml` `stop` `reset` |
| `/waterdroid/process_state` | std_msgs/String (JSON) | water_processor: tank volumes, relays, totals, health |
| `/waterdroid/arm/cmd`, `/waterdroid/arm/state` | std_msgs/String | `EXTEND` `RETRACT` `ANGLE:n` / JSON |
| `/waterdroid/estop` | std_msgs/Bool | dashboard → everything |
| `/waterdroid/status` | std_msgs/String (JSON) | status_monitor 1 Hz frame |
