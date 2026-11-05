# XOD nodes — Water Rover v2

C++ nodes (`not-implemented-in-xod`) for each subsystem. The C++ is **generated** from the firmware cores
(`firmware/waterdroid_esp32/src/*.h`) so XOD, the ESP32 sketch and the unit tests share identical logic:

```bash
python3 xod/waterdroid/build_xod.py   # regenerate after editing a core
python3 xod/waterdroid/check_xod.py   # C++ syntax check of every node (incl. power-management/xod)
```

Open the `xod/waterdroid` folder as a project in XOD IDE (File → Open Project). Power sequencing is the
`relay-sequencer` node in `power-management/power-management/xod`.

| Node | Issue | Wire it like this (pins from docs/HARDWARE.md) |
|------|-------|-----------------------------------------------|
| `intake-controller` | #1 | `WANT` → relay-sequencer `REQ1`; `PIN1` → `xod/gpio/digital-write` GPIO23; relay ON state (`NOT PIN1`) → `RON`; `analog-sensor` GPIO34 ×100 → `LVL`; level-monitor `NOT ALLW` → `BLK` |
| `dosing-pump` ×2 | #3 | alum: `WANT` → `REQ2` (GPIO19), chlorine: `WANT` → `REQ3` (GPIO18); `TOT` → watch / telemetry |
| `level-monitor` | #5 | three `analog-sensor` (GPIO34/35/36) ×100 → `S1 S2 S3`; `ALLW` gates the intake |
| `servo-arm` | #6 | intake boom: `VAL` → `xod-dev/servo/rotate-servo` on GPIO13; `DEP` → mission `DEP`, `NOT DEP` → intake `BLK`, `NOT STW` → drive `INH` |
| `delivery-controller` | #8 | `WANT` → `REQ4` (GPIO4); `HOME` from the mission (state = DISPENSING); `ARM` stays true (fixed nozzle) |
| `l298n-drive` | #9 | `IN1..IN4` → digital-write 27/26/25/33; `ENA`/`ENB` → `pwm-write` 14/32; mission `NOT DRV` → `INH` |
| `mission-fsm` | #11 | pulses `INTK ALUM CHLR EXT RET DLV STOP` → the matching node inputs; busy flags back in |
| `flow-meter` | #1 #5 | `PORT` 34 (flow sensor via divider); `BAT` / target ×100 → intake `LVL`; `TOT` → volume-model `IN` |
| `volume-model` | #5 #8 | `S1 S2 S3` → level-monitor and mission-fsm; `CML` → mission `CML`; `S3` → delivery `S3` |
| `route-navigator` | #7 | `GO` = mission travelling; `L R UPD` → l298n-drive; `RIV HOM DEP` → mission `ARIV AHOM ADEP` |
| `telemetry-json` | #12 | `OUT` → serial / Wi-Fi to Node-RED (`rover/telemetry` keys) |

Pulses: XOD pulse outputs fire for one transaction; number outputs like `ALML` / `DLML` carry the volume
that goes with the pulse.
