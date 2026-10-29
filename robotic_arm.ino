#include <Servo.h>

Servo armServo;

const int SERVO_PIN = 9;
const int STOW_ANGLE = 0;
const int DEPLOY_ANGLE = 90;

int currentAngle = STOW_ANGLE;
int targetAngle = STOW_ANGLE;

unsigned long previousMoveTime = 0;
const unsigned long MOVE_INTERVAL = 20;

String inputCommand = "";

void setup() {
  Serial.begin(9600);
  armServo.attach(SERVO_PIN);
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
