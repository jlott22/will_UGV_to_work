#include <Servo.h>

// === Pin assignments (same as your original motor wiring) ===
const int dirPin  = 4;   // Direction to Cytron MD30C
const int pwmPin  = 5;   // PWM speed to Cytron
const int servoPin = 10; // Steering servo signal (white wire)

// === Steering servo ===
Servo steering;

// === Motor speed settings (safe, low-power for testing) ===
// You can increase these later once everything works.
const int forwardPWM = 25;   // gentle forward speed (0–255)
const int turnPWM    = 20;   // even slower when turning

// === Steering angles (you can tweak these if needed) ===
const int servoCenter = 90;  // center
const int servoLeft   = 60;  // left turn
const int servoRight  = 120; // right turn

// === Failsafe and command tracking ===
unsigned long lastCmdTime = 0;
const unsigned long timeoutMs = 3000;   // stop if no command for 3 seconds

char lastCmd = 'S';  // last command: 'F', 'L', 'R', 'S'


// --------- Motor/steering helper functions ---------

void stopMotor() {
  analogWrite(pwmPin, 0);  // motor OFF
}

void forward() {
  digitalWrite(dirPin, LOW);           // LOW = forward (same as your old code)
  analogWrite(pwmPin, forwardPWM);     // gentle throttle
  steering.write(servoCenter);         // steer straight
}

void turnLeft() {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, turnPWM);        // slower while turning
  steering.write(servoLeft);
}

void turnRight() {
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, turnPWM);
  steering.write(servoRight);
}

void applyCommand() {
  if (lastCmd == 'F') {
    forward();
  } else if (lastCmd == 'L') {
    turnLeft();
  } else if (lastCmd == 'R') {
    turnRight();
  } else { // 'S' or anything else
    stopMotor();
    steering.write(servoCenter);
  }
}


// --------- Arduino standard setup/loop ---------

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  steering.attach(servoPin);
  steering.write(servoCenter);

  stopMotor();

  Serial.begin(115200);   // Jetson will use 115200 baud

  lastCmdTime = millis();
}

void loop() {
  // Read any incoming serial command from Jetson
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    lastCmd = cmd;
    lastCmdTime = millis();
  }

  // Apply the most recent command
  applyCommand();

  // Failsafe: if no command for timeoutMs, stop
  if (millis() - lastCmdTime > timeoutMs) {
    lastCmd = 'S';
    stopMotor();
    steering.write(servoCenter);
  }

  delay(5);
}
