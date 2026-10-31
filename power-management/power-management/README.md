# Issue #10 — Power Distribution & Load Management

Hardware: 10,000 mAh+ 5 V power bank, 4-channel 5 V relay module, ESP32.
Roles: **XOD node** = multi-channel relay sequencing (no simultaneous motor/pump start-up). **Node-RED** = battery/rail meter and power-mode toggle (Active / Sleep / Emergency Shutdown).

```
power-management/
├── core/power_core.h                    the sequencing + brownout logic (single source of truth)
├── xod/power-management/                XOD project: relay-sequencer native node
├── esp32/power_management/              ESP32 sketch (mirrors the XOD node, adds MQTT)
├── node-red/power_management_flow.json  dashboard flow (import into Node-RED)
└── tests/                               host unit tests (C++ and JavaScript)
```

## 1. How it behaves

| Rule | Behaviour |
|---|---|
| Boot | Mode **SLEEP**: all relays off until `active` is received. |
| Staggered start | Loads switch ON **one at a time**, 600 ms apart, in rank order: CH2 left motor → CH3 right motor → CH1 intake pump → CH4 aux pump. Switching OFF is immediate. |
| Rest time | A channel that was switched off cannot restart for 1 s (relay and motor protection). |
| Start gate | No load starts while the rail is below 4.75 V. |
| Brownout shed | Rail below 4.50 V for 150 ms → the least important running load is switched off (aux, then pump, then motors) and new starts are blocked for 5 s. At most one shed per second. |
| Escalation | 3 sheds without 30 s of healthy rail, or rail below 4.20 V for 1 s → **EMERGENCY** (latched). |
| Emergency | All relays off. Only `reset` clears the latch, and it returns to SLEEP, never straight to ACTIVE. |
| Comms loss | ESP32 loses MQTT for 15 s while ACTIVE → SLEEP. |
| Invalid rail reading | If the divider is disconnected the timers still stagger starts and the dashboard shows `UNKNOWN`. |

Thresholds are defaults in `PowerConfig` (`core/power_core.h`). Tune them after measuring the real rover.

## 2. Hardware

**Pins** (CH1 matches `filtration_monitor-1.ino`; none of them are boot-strapping pins):

| Relay | GPIO | Load |
|---|---|---|
| IN1 | 26 | Intake pump |
| IN2 | 27 | Left motor |
| IN3 | 25 | Right motor |
| IN4 | 33 | Aux pump |
| — | 39 | 5 V rail sense (divider) |

Change `RELAY_PIN` and `rank` if your channel assignment differs.

**Power rails and isolation**
1. Load wiring: bank 5 V → relay COM, relay NO → load, load → bank GND. Fit a flyback diode (1N5819/1N4007) across every motor and pump, cathode to +5 V.
2. Relay board: remove the **JD-VCC jumper**. Feed JD-VCC (coil supply) from the load rail; feed the board's VCC (optocoupler side) from the ESP32 **3V3** pin, not 5 V. With VCC at 5 V and a 3.3 V GPIO, active-low inputs can leave the relay half-on. Check your board's silkscreen, since layouts differ.
3. ESP32 supply: power VIN through its own 470–1000 µF low-ESR capacitor plus 100 nF, close to the board, so motor inrush dips do not reset it. Join all grounds at one star point at the bank.
4. Rail sense: bank 5 V → 10 kΩ → GPIO39 → 10 kΩ → GND, with 100 nF from GPIO39 to GND. Compare with a multimeter and set `VCAL` in the sketch.
5. One bank shares one ground, so this gives noise and current isolation, not galvanic isolation. Use a second bank if galvanic isolation is required.
6. The bank's output limit (often 2–3 A total) is why loads are staggered and shed. Some banks switch off below about 50–100 mA, so if the bank cuts out in SLEEP, add a small keep-alive load.

**The meter measures the 5 V rail, not the charge.** A bank holds 5 V until nearly empty, so voltage only shows sag under load. The dashboard therefore also estimates remaining charge by integrating load current (assumed per-channel currents at the top of the estimator node). Replace them with figures measured on a USB power meter, and press *Power bank recharged* after charging.

## 3. Setup

### ESP32 (Arduino IDE)
1. Install board package **esp32 by Espressif ≥ 2.0** and library **PubSubClient**.
2. Open `esp32/power_management/power_management.ino` (keep `power_core.h` in the same folder).
3. Set `WIFI_SSID`, `WIFI_PASS`, `MQTT_HOST`, upload at 115200 baud.

### Node-RED
1. Install `node-red-dashboard` (Menu ▸ Manage palette). Run an MQTT broker on `localhost:1883` (same as `flow.json`).
2. Menu ▸ Import ▸ paste `node-red/power_management_flow.json` ▸ Deploy.
3. Open `http://localhost:1880/ui` ▸ tab **Power**.
4. No hardware yet: switch on **Simulate ESP32**, press **ACTIVE**, then switch on the four channel requests. Loads start in sequence; **Inject brownout** shows load shedding.

Manual MQTT test:
```bash
mosquitto_pub -t rover/power/cmd -m active
mosquitto_pub -t rover/power/req/2 -m 1     # left motor
mosquitto_sub -t rover/power/status
```
The mission controller should request loads on `rover/power/req/1..4` instead of switching relays directly.

### XOD
1. Copy `xod/power-management` into your XOD workspace and open it (File ▸ Switch Workspace).
2. In a new `main` patch place **relay-sequencer**. Connect `REQ1..4` (boolean), `VBAT` (rail volts), and `PIN1..4` to `digital-output` nodes on the pins above. Defaults: `MODE` 1 (SLEEP), `ACTLOW` true, `STAG` 600 ms.
3. `MODE`: 0 ACTIVE, 1 SLEEP, 2 EMERGENCY; pulse `RST` clears the latch. `RAIL`: 0 unknown, 1 OK, 2 sag, 3 critical.

## 4. Tests

```bash
bash tests/run_tests.sh              # 15 C++ tests for the core logic (needs g++)
node tests/node_red_functions_test.js   # Node-RED function nodes and simulator parity (needs Node.js)
```

## 5. Known limits
- The XOD node was written to XOD's native-node format but not opened in the XOD IDE; the ESP32 sketch is the same logic and is the fallback.
- Relays switch loads fully on or off; no PWM speed control.
- ADC readings on the ESP32 are about ±2–5 % until calibrated with `VCAL`.
