# Firmware

`waterdroid_esp32/` is the integrated sketch for the rover's single ESP32 (all subsystems, MQTT, serial
bridge). Logic lives in pure C++ headers under `waterdroid_esp32/src/` so it can be unit-tested on a PC
and reused by the XOD nodes:

| Header | Contents |
|--------|----------|
| `rover_pins.h` | the one pin map |
| `power_core.h` | #10 relay sequencer (copy of `power-management/.../core/power_core.h`, kept identical by the tests) |
| `pump_cores.h` | #1 `IntakeController`, #3 `DosingPump`, #8 `DeliveryController` |
| `motion_cores.h` | #5 `FiltrationMonitor`, #6 `ServoArm`, #9 `DriveController` |
| `mission_core.h` | #11 `MissionCore` state machine |

`bash tests/run_tests.sh` runs the unit tests and syntax-checks every sketch in the repo against
Arduino/ESP32 stubs (`tests/stubs`). Wiring and calibration: `../docs/HARDWARE.md`.
