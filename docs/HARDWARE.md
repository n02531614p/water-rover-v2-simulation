# Water Rover v2 — hardware, pin map and wiring

This is the as-built rover from the design drawing (two wooden decks, 4WD, three-stage treatment train,
servo delivery arm). Everything in the repo — Gazebo model, ESP32 firmware, XOD nodes, Node-RED flows —
uses the numbers on this page.

## Parts list (what we have)

| # | Part | Subsystem / issue | Notes |
|---|------|-------------------|-------|
| 1 | ESP32 DevKit V1 (30-pin) | #11 "autonomous brain", #12 Wi-Fi | one board runs everything |
| 1 | 10,000 mAh 5 V USB power bank (2 outputs preferred) | #10 | see power notes |
| 1 | 4-channel 5 V relay module (active LOW, opto-isolated) | #10 | CH1–CH4 = the four pumps |
| 1 | L298N double H-bridge | #9 | left motor pair on A, right pair on B |
| 4 | 5 V TT geared DC motors + 65 mm off-road wheels | #9 | skid steer, 2 motors in parallel per side |
| 1 | 5 V submersible intake pump + coarse mesh pre-filter + 40–50 cm flexible hose | #1 | relay CH1 |
| 1 | Stage 1 transparent raw water tank (500–1000 mL) | #1, #3 | |
| 1 | 5 V alum dosing pump (peristaltic) | #3 | relay CH2 |
| 1 | Stage 2 clear gravity-fed filtration column | #5 | gravel / sand / charcoal |
| 1 | Stage 3 transparent disinfection & clean tank (500–1000 mL) | #3, #8 | |
| 1 | 5 V chlorine dosing pump | #3 | relay CH3 |
| 1 | 5 V delivery pump | #8 | relay CH4 |
| 1 | SG90 servo + wooden delivery arm + nozzle | #6 | 0° stowed, 90° deployed |
| 1 | Water-flow sensor **YF-S401** (hall turbine, 0.3-6 L/min, ~5880 pulses/L) | #1, #5 | on the intake line — measures the batch and detects dry-running |

### How the rover works without level sensors or GPS (current build)

* **Flow sensor instead of level sensors.** The one flow sensor sits on the intake hose between the pre-filter
  and Stage 1. The intake stops when it has counted the batch (950 mL minus any water still on board) and
  trips **DRY_RUN** if the pump runs 4 s without enough pulses. Stage 1 / column / Stage 3 volumes are then
  *estimated* (`VolumeModel`): + intake, + alum, + chlorine, − delivered, with gravity filtration at a
  calibrated mL/s. Set `USE_LEVEL_SENSORS 1` in the sketch if level sensors are added later (S2 on GPIO35,
  S3 on GPIO36; Stage 1 stays on the flow sensor).
* **Fixed route instead of path finding.** The route is a fixed list of coordinates in
  `firmware/waterdroid_esp32/src/nav_cores.h` (`DEFAULT_ROUTE`): river → back off 0.6 m → homestead 1 → 2 →
  3 → depot. The rover pivots to face the next point, then drives straight; its position is estimated from
  how long it drove/turned (dead reckoning, `RouteNavigator`). Arrival triggers pumping / delivery
  automatically. In Gazebo, `mission:=route` runs exactly this and logs the drift against the true position:
  straight legs are accurate (river: 0.05 m after 9.6 m) but every pivot adds heading error, so without
  correction the miss grows to ~0.5 m at homestead 1, ~1.4 m at homestead 2 and ~2.8 m at homestead 3.
  So: start every trip at the same marked spot facing the river, press **Arrived** at each stop if the rover
  is off (it snaps the estimate back), and keep the route to few turns. The single most useful add-on is a
  cheap **compass (QMC5883L, ~US$3, I²C pins 21/22 already reserved)** — it removes the heading error that
  causes almost all of the drift; wheel encoders fix the distance part.

#### Setting up the route for your site

1. Mark the depot spot and the direction the rover faces at the start (towards the river = +x).
2. Measure each stop with a tape: x metres ahead, y metres to the left (right = negative).
3. Edit `DEFAULT_ROUTE` (keep the `RA_REVERSE` step after the river so the hose clears the bank).
4. Calibrate (Drive & Map tab, mission paused, rover on the real ground):
   **Calibrate: 5 s forward** → measure metres ÷ 5 = `NAV_SPEED_MPS`;
   **Calibrate: 5 s pivot** → measure degrees ÷ 5 = `NAV_TURN_DPS`. Repeat 3× and average; redo when the
   ground or the power bank charge changes a lot.

### Not in the drawing but needed / recommended

| Part | Needed for | Why |
|------|-----------|-----|
| 2 × analog water-level sensors (optional) | #5 column overflow, #8 Stage 3 empty | the flow sensor + volume estimate covers this now; sensors make it measured instead of estimated |
| 10k + 20k resistors | flow sensor | divider from the sensor's 5 V pulse output to GPIO34 (3.3 V max) |
| 2 × 10 kΩ resistors + 100 nF capacitor | #10 battery / 5 V rail meter | voltage divider into GPIO39 |
| 4 × 1N5819 (or 1N4007) diodes | #10 | flyback protection across each pump motor |
| 470–1000 µF electrolytic capacitor | #9/#10 | across the L298N supply, absorbs motor inrush |
| Wheel encoders (LM393 slot sensor + 20-slot disc; often sold with TT motor kits) | #7 | cheapest big improvement to the fixed route: counts wheel turns instead of timing |
| GPS (e.g. NEO-6M), compass/IMU (QMC5883L or MPU-6050), 1–3 × HC-SR04 | #7 | not available for now — the fixed route works without them; pins stay reserved below |

## ESP32 pin map (single source: `firmware/waterdroid_esp32/src/rover_pins.h`)

| GPIO | Connects to | Issue |
|------|-------------|-------|
| 14 | L298N **ENA** (left PWM — remove ENA jumper) | #9 |
| 27 / 26 | L298N **IN1 / IN2** (left direction) | #9 |
| 25 / 33 | L298N **IN3 / IN4** (right direction) | #9 |
| 32 | L298N **ENB** (right PWM — remove ENB jumper) | #9 |
| 23 | Relay **IN1** → intake pump | #1 |
| 19 | Relay **IN2** → alum dosing pump | #3 |
| 18 | Relay **IN3** → chlorine dosing pump | #3 |
| 4 | Relay **IN4** → delivery pump | #8 |
| 13 | SG90 signal (orange) | #6 |
| 34 | **Water-flow sensor** pulse (yellow) via 10k/20k divider | #1 #5 |
| 35 | optional Stage 2 column level sensor (analog) | #5 |
| 36 (VP) | optional Stage 3 level sensor (analog) | #5 #8 |
| 39 (VN) | 5 V rail divider midpoint | #10 |
| 2 | on-board LED (mission running / fast blink = E-STOP) | #12 |
| 16 / 17 | reserved: GPS UART2 RX / TX | #7 |
| 21 / 22 | reserved: I²C SDA / SCL (compass / IMU) | #7 |
| 5 / 15 | reserved: HC-SR04 TRIG / ECHO (echo through a 5 V→3.3 V divider) | #7 |

Pins 34–39 are input-only ADC1 pins, which keep working while Wi-Fi is on (ADC2 pins do not).
GPIO 6–11 are the SPI flash — never use them (the old `robotic_arm.ino` used pin 9, which only works on an
Arduino Uno; it now uses GPIO13 on the ESP32).

**Pin changes vs. the first member sketches:** `filtration_monitor-1.ino` moved S3 from GPIO32 to GPIO36 and
the intake relay from GPIO26 to GPIO23; `power_management.ino` moved the relays from 26/27/25/33 to
23/19/18/4 — those pins are now the L298N. All sketches in the repo are updated.

## Wiring and power notes (Issue #10)

Full wiring diagrams: `docs/wiring/power.svg`, `drive.svg`, `relays.svg`, `sensors.svg` (regenerate with
`python3 docs/wiring/build_wiring.py`).

```
 Power bank USB-A #1 ──► ESP32 USB (logic only: ESP32, relay coil side via JD-VCC, level sensors)
 Flow sensor: red → 5 V, black → GND, yellow → 10k → GPIO34 → 20k → GND
 Power bank USB-A #2 ──►┬─► L298N +12V (Vs) AND L298N +5V terminal (remove the 5V-EN jumper) ─► 4 × TT motors
                        ├─► relay COM terminals ─► NO ─► pumps (+ flyback diode across each pump)
                        ├─► SG90 red (+100 µF close to the servo)
                        └─► 470–1000 µF across the rail, and the 10k/10k divider ─► GPIO39
 All grounds common (ESP32 GND, L298N GND, relay GND, power bank GND).
```

* **Two outputs = isolation.** Motors and pumps on one USB output, the ESP32 on the other, so motor inrush
  cannot brown-out the ESP32. With a single-output bank, keep the big capacitor and rely on the firmware
  sequencer (loads start 600 ms apart, brownout load-shedding, E-STOP below 4.2 V).
* **L298N on 5 V:** the L298N drops ~1.5–2 V, so the TT motors see only ~3–3.5 V and run slowly. The firmware
  uses a minimum PWM of 90/255 to stop stall-hum. If the rover is too slow, a TB6612FNG (MOSFET, ~0.2 V drop)
  is a drop-in swap for the same pins (PWMA/PWMB, AIN1/2, BIN1/2).
* **Relay board:** remove the JD-VCC jumper, feed JD-VCC from the 5 V rail and VCC from the ESP32 3.3 V pin
  so the 3.3 V GPIOs can drive the opto-couplers. The board is active-LOW: the firmware drives all four
  inputs HIGH before anything else at boot so no pump clicks on.
* **Drive and pumps never run together.** The mission state machine inhibits the motors while any pump relay
  is on (pumping, dosing, delivering), which is also what keeps the power bank from shutting down.
* **Power-bank auto-off:** some banks switch off below ~100 mA. The ESP32 with Wi-Fi draws more than that, so
  this is only an issue with Wi-Fi disabled.

## Calibration you must do on the real rover

| What | Where | How |
|------|-------|-----|
| Flow sensor pulses per litre | `FLOW_PULSES_PER_L` | pump exactly 1 L into a jug, read `total_ml` on the dashboard, scale |
| Filter column throughput | `FILTER_ML_PER_S` | time how long 500 mL takes to pass the column |
| Route speed / pivot rate | `NAV_SPEED_MPS`, `NAV_TURN_DPS` | dashboard calibrate buttons (above) |
| Dosing pump rate (mL/s) | `alum.rateMlPerS`, `chlorine.rateMlPerS` | run 60 s into a measuring cylinder |
| Delivery pump flow (mL/s) | `delivery.flowMlPerS` | time filling a 500 mL jug |
| 5 V rail reading | `VCAL` | multimeter reading ÷ sketch reading |
| Servo end stops | `servo.attach(PIN_SERVO, 500, 2400)` | adjust if the SG90 buzzes at 0° or 90° |

## Treatment chemistry used for the dose numbers (Issue #3)

| Stock | Make it | Dose | Gives |
|-------|---------|------|-------|
| Alum 1 % | 10 g aluminium sulphate per litre of water | 3 mL per litre treated | ~30 mg/L (typical 10–50 mg/L for turbid river water) |
| Chlorine 0.35 % | household bleach (3.5 %) diluted 1 : 10 | 0.6 mL per litre treated | ~2 mg/L free chlorine |

Allow ≥ 30 min chlorine contact time before delivery (the firmware default; the simulation shortens it to
5 s). Jar-test the alum dose for your river water — turbidity changes it a lot. Never mix alum and bleach
stocks; label both bottles.
