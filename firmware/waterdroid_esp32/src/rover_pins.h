// ============================================================================
// rover_pins.h - THE single pin map for the one ESP32 "autonomous brain".
// Every sketch / XOD patch in this repo must use these pins (docs/HARDWARE.md).
// Board: ESP32 DevKit V1 (30-pin). Avoids strapping pins 0/2*/12/15 for outputs
// that must be quiet at boot, and keeps ADC1 pins for analog (works with Wi-Fi).
// ============================================================================
#ifndef ROVER_PINS_H
#define ROVER_PINS_H

// ---- Issue #9  L298N double H-bridge (4x TT motors: 2 per side, in parallel) ----
#define PIN_L298_ENA   14   // left side PWM   (remove the ENA jumper)
#define PIN_L298_IN1   27   // left  dir A
#define PIN_L298_IN2   26   // left  dir B
#define PIN_L298_IN3   25   // right dir A
#define PIN_L298_IN4   33   // right dir B
#define PIN_L298_ENB   32   // right side PWM  (remove the ENB jumper)

// ---- Issue #10 4-channel 5 V relay module (active LOW) = the four pumps ----
#define PIN_RELAY_CH1  23   // CH1 intake pump      (Issue #1)
#define PIN_RELAY_CH2  19   // CH2 alum dosing pump (Issue #3)
#define PIN_RELAY_CH3  18   // CH3 chlorine pump    (Issue #3)
#define PIN_RELAY_CH4   4   // CH4 delivery pump    (Issue #8)
#define RELAY_ACTIVE_LOW 1

// ---- Issue #6  SG90 delivery-arm servo ----
#define PIN_SERVO      13

// ---- Issue #5  level sensors (analog, ADC1 input-only pins) ----
#define PIN_LEVEL_S1   34   // Stage 1 raw tank
#define PIN_LEVEL_S2   35   // Stage 2 filtration column
#define PIN_LEVEL_S3   36   // Stage 3 clean tank (VP)

// ---- Issue #10 5 V rail sense: rail -> 10k -> GPIO39 -> 10k -> GND (+100 nF) ----
#define PIN_RAIL_SENSE 39   // VN

// ---- status LED ----
#define PIN_LED         2

// ---- Issue #7  reserved for navigation sensors (NOT yet in the parts list) ----
#define PIN_GPS_RX     16   // UART2 RX  <- GPS TX
#define PIN_GPS_TX     17   // UART2 TX  -> GPS RX
#define PIN_I2C_SDA    21   // compass / IMU
#define PIN_I2C_SCL    22
#define PIN_SONAR_TRIG  5   // HC-SR04 trigger (strapping pin: output only after boot)
#define PIN_SONAR_ECHO 15   // via 5V->3.3V divider

#endif
