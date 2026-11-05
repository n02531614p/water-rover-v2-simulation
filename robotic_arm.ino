// Servo INTAKE arm (Issue #6) - bench test sketch: EXTEND lowers the intake into the water, RETRACT raises it.
// (The delivery nozzle is fixed; the servo moves the intake hose + pump.)
// Runs on the rover ESP32 (ESP32Servo library, GPIO13 per firmware/waterdroid_esp32/src/rover_pins.h)
// or on an Arduino Uno for bench testing (Servo library, pin 9).
// The integrated rover firmware uses the same logic in src/motion_cores.h (ServoArm).
#if defined(ESP32)
#include <ESP32Servo.h>
const int SERVO_PIN = 13;   // GPIO9 is the SPI-flash line on ESP32 - never use it
#else
#include <Servo.h>
const int SERVO_PIN = 9;
#endif

Servo armServo;

const int STOW_ANGLE = 0;
const int DEPLOY_ANGLE = 90;

int currentAngle = STOW_ANGLE;
int targetAngle = STOW_ANGLE;

unsigned long previousMoveTime = 0;
const unsigned long MOVE_INTERVAL = 20;

String inputCommand = "";

void readSerialCommand();
void moveServoSmoothly();

void setup() {
  Serial.begin(9600);
#if defined(ESP32)
  armServo.setPeriodHertz(50);
  armServo.attach(SERVO_PIN, 500, 2400);   // SG90 pulse range
#else
  armServo.attach(SERVO_PIN);
#endif
  armServo.write(STOW_ANGLE);

  Serial.println("Robotic Arm Controller Ready");
  Serial.println("Commands: EXTEND, RETRACT, ANGLE:0-90");
}

void loop() {
  readSerialCommand();
  moveServoSmoothly();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n') {
      inputCommand.trim();
      inputCommand.toUpperCase();

      if (inputCommand == "EXTEND") {
        targetAngle = DEPLOY_ANGLE;
        Serial.println("Extending arm");
      } else if (inputCommand == "RETRACT") {
        targetAngle = STOW_ANGLE;
        Serial.println("Retracting arm");
      } else if (inputCommand.startsWith("ANGLE:")) {
        String value = inputCommand.substring(6);
        bool validNumber = value.length() > 0;

        for (unsigned int i = 0; i < value.length(); i++) {
          if (!isDigit(value.charAt(i))) {
            validNumber = false;
            break;
          }
        }

        if (validNumber) {
          int angle = value.toInt();

          if (angle >= STOW_ANGLE && angle <= DEPLOY_ANGLE) {
            targetAngle = angle;
            Serial.print("Target angle: ");
            Serial.println(targetAngle);
          } else {
            Serial.println("Error: angle must be between 0 and 90");
          }
        } else {
          Serial.println("Error: invalid angle command");
        }
      } else if (inputCommand.length() > 0) {
        Serial.println("Unknown command");
      }

      inputCommand = "";
    } else if (incoming != '\r') {
      if (inputCommand.length() < 32) {
        inputCommand += incoming;
      } else {
        inputCommand = "";
        Serial.println("Error: command too long");
      }
    }
  }
}

void moveServoSmoothly() {
  unsigned long now = millis();

  if (now - previousMoveTime >= MOVE_INTERVAL) {
    previousMoveTime = now;

    if (currentAngle < targetAngle) {
      currentAngle++;
      armServo.write(currentAngle);
    } else if (currentAngle > targetAngle) {
      currentAngle--;
      armServo.write(currentAngle);
    }
  }
}
