#include <Servo.h>

/*
  car_control.ino

  Arduino actuator controller for UGV car.

  New serial protocol from Jetson:

      D,<servo_angle>,<throttle_pwm>\n
      S\n

  Examples:

      D,90,25     -> steer center, drive forward PWM 25
      D,60,35     -> max left steering, drive forward PWM 35
      D,120,35    -> max right steering, drive forward PWM 35
      D,90,0      -> centered steering, motor stopped
      S           -> stop motor and center steering

  Control split:

      Jetson / UGV_Navigation.py:
          - reads GPS heading
          - computes steering angle
          - computes throttle PWM
          - sends D commands continuously

      Arduino:
          - applies steering angle and throttle PWM
          - stops if command stream times out
*/

// =====================================================
// Pin assignments
// =====================================================

const int dirPin = 4;       // Direction pin to Cytron MD30C
const int pwmPin = 5;       // PWM speed pin to Cytron MD30C
const int servoPin = 10;    // Steering servo signal pin


// =====================================================
// Steering limits
// =====================================================

const int SERVO_CENTER = 90;
const int SERVO_LEFT_LIMIT = 60;
const int SERVO_RIGHT_LIMIT = 120;


// =====================================================
// Motor limits
// =====================================================

const int PWM_STOP = 0;
const int PWM_MIN_ALLOWED = 0;
const int PWM_MAX_ALLOWED = 255;

// For your current setup, movement begins around PWM 20.
// The Arduino will still allow lower values in case you intentionally
// want to test them, but Navigation should normally command either 0
// or >= 20.
const int PWM_MIN_MOVING_RECOMMENDED = 20;


// =====================================================
// Direction configuration
// =====================================================

// Your old code used LOW as forward.
const int FORWARD_DIR = LOW;


// =====================================================
// Serial / failsafe settings
// =====================================================

const unsigned long SERIAL_BAUD = 115200;

// Since Jetson will send continuous commands, stop quickly if the stream dies.
const unsigned long COMMAND_TIMEOUT_MS = 1500;

// Max length of one incoming serial line.
const int LINE_BUF_SIZE = 48;


// =====================================================
// Globals
// =====================================================

Servo steering;

char lineBuf[LINE_BUF_SIZE];
int lineLen = 0;

unsigned long lastCommandTime = 0;

int currentServoAngle = SERVO_CENTER;
int currentThrottlePWM = 0;


// =====================================================
// Helper functions
// =====================================================

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}


void applyActuators(int servoAngle, int throttlePWM) {
  servoAngle = clampInt(servoAngle, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
  throttlePWM = clampInt(throttlePWM, PWM_MIN_ALLOWED, PWM_MAX_ALLOWED);

  currentServoAngle = servoAngle;
  currentThrottlePWM = throttlePWM;

  steering.write(currentServoAngle);

  if (currentThrottlePWM <= 0) {
    analogWrite(pwmPin, PWM_STOP);
    return;
  }

  digitalWrite(dirPin, FORWARD_DIR);
  analogWrite(pwmPin, currentThrottlePWM);
}


void stopCar() {
  currentThrottlePWM = 0;
  currentServoAngle = SERVO_CENTER;

  analogWrite(pwmPin, PWM_STOP);
  steering.write(SERVO_CENTER);
}


void sendAck(const char *msg) {
  Serial.println(msg);
}


// =====================================================
// Command parsing
// =====================================================

void handleDriveCommand(char *line) {
  // Expected:
  // D,<servo_angle>,<throttle_pwm>
  //
  // strtok modifies the input buffer, which is okay here.

  char *token = strtok(line, ",");
  if (token == NULL) {
    sendAck("ERR,EMPTY_DRIVE");
    return;
  }

  // First token should be "D"
  if (token[0] != 'D') {
    sendAck("ERR,NOT_DRIVE");
    return;
  }

  token = strtok(NULL, ",");
  if (token == NULL) {
    sendAck("ERR,MISSING_SERVO");
    return;
  }
  int servoAngle = atoi(token);

  token = strtok(NULL, ",");
  if (token == NULL) {
    sendAck("ERR,MISSING_PWM");
    return;
  }
  int throttlePWM = atoi(token);

  servoAngle = clampInt(servoAngle, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
  throttlePWM = clampInt(throttlePWM, PWM_MIN_ALLOWED, PWM_MAX_ALLOWED);

  applyActuators(servoAngle, throttlePWM);
  lastCommandTime = millis();

  Serial.print("OK,D,");
  Serial.print(servoAngle);
  Serial.print(",");
  Serial.println(throttlePWM);
}


void handleCommand(char *line) {
  // Ignore empty lines.
  if (line[0] == '\0') {
    return;
  }

  // Stop command.
  if (line[0] == 'S') {
    stopCar();
    lastCommandTime = millis();
    sendAck("OK,S");
    return;
  }

  // Drive command.
  if (line[0] == 'D') {
    handleDriveCommand(line);
    return;
  }

  // Optional backward compatibility with old burst commands.
  // This lets your old Navigation code still do something if accidentally run.
  if (line[0] == 'F') {
    applyActuators(SERVO_CENTER, 25);
    lastCommandTime = millis();
    sendAck("OK,F_COMPAT");
    return;
  }

  if (line[0] == 'L') {
    applyActuators(SERVO_LEFT_LIMIT, 25);
    lastCommandTime = millis();
    sendAck("OK,L_COMPAT");
    return;
  }

  if (line[0] == 'R') {
    applyActuators(SERVO_RIGHT_LIMIT, 25);
    lastCommandTime = millis();
    sendAck("OK,R_COMPAT");
    return;
  }

  sendAck("ERR,UNKNOWN_CMD");
}


void processSerialByte(char c) {
  if (c == '\r') {
    return;
  }

  if (c == '\n') {
    lineBuf[lineLen] = '\0';
    handleCommand(lineBuf);
    lineLen = 0;
    return;
  }

  if (lineLen < LINE_BUF_SIZE - 1) {
    lineBuf[lineLen] = c;
    lineLen++;
  } else {
    // Buffer overflow: reset line and stop for safety.
    lineLen = 0;
    stopCar();
    sendAck("ERR,LINE_TOO_LONG");
  }
}


// =====================================================
// Arduino setup / loop
// =====================================================

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  digitalWrite(dirPin, FORWARD_DIR);
  analogWrite(pwmPin, PWM_STOP);

  steering.attach(servoPin);
  steering.write(SERVO_CENTER);

  Serial.begin(SERIAL_BAUD);

  stopCar();
  lastCommandTime = millis();

  Serial.println("READY,car_control,D_angle_pwm");
}


void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    processSerialByte(c);
  }

  // Failsafe: stop if Jetson command stream disappears.
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopCar();
  }

  delay(5);
}