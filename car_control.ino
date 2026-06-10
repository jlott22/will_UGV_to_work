#include <Servo.h>

const int dirPin = 4;
const int pwmPin = 5;
const int servoPin = 10;

// Buzzer pins
const int buzzerPin = 9;
const int buzzerGndPin = 7;

const int SERVO_CENTER = 90;
const int SERVO_LEFT_LIMIT = 60;
const int SERVO_RIGHT_LIMIT = 120;

const int PWM_STOP = 0;
const int PWM_MIN_ALLOWED = 0;
const int PWM_MAX_ALLOWED = 255;
const int FORWARD_DIR = LOW;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long COMMAND_TIMEOUT_MS = 1500;
const int LINE_BUF_SIZE = 48;

// Minimum forced silence between separate buzzer codes.
// This makes B1, B2, B3, etc. audibly separate even if Python sends them quickly.
const unsigned long BUZZER_CODE_GAP_MS = 1500;

Servo steering;

char lineBuf[LINE_BUF_SIZE];
int lineLen = 0;

unsigned long lastCommandTime = 0;
unsigned long lastBuzzerCodeEndMs = 0;

int currentServoAngle = SERVO_CENTER;
int currentThrottlePWM = 0;

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
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

void waitForBuzzerGap() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastBuzzerCodeEndMs;

  if (lastBuzzerCodeEndMs > 0 && elapsed < BUZZER_CODE_GAP_MS) {
    delay(BUZZER_CODE_GAP_MS - elapsed);
  }
}

void beep(int freq, int durMs) {
  tone(buzzerPin, freq, durMs);
  delay(durMs);
  noTone(buzzerPin);
  delay(140);  // gap between notes inside one code
}

void playBuzzerCode(const char *code) {
  waitForBuzzerGap();

  if (strcmp(code, "B1") == 0) {
    // Navigation started / Arduino connected: one medium beep
    beep(900, 300);

  } else if (strcmp(code, "B2") == 0) {
    // MQTT connected: two rising beeps
    beep(1200, 300);

  } else if (strcmp(code, "B3") == 0) {
    // GPS communication verified: three rising beeps
    beep(1500, 300);

  } else if (strcmp(code, "B4") == 0) {
    // RTK fixed: one long high beep
    beep(2000, 1000);

  } else if (strcmp(code, "B5") == 0) {
    // Heading acquired: low-high-low-high
    beep(2000, 160);

  } else if (strcmp(code, "B6") == 0) {
    // Startup ACK received: three fast high beeps
    beep(900, 120);

  } else if (strcmp(code, "B7") == 0) {
    // Mission started: rising melody
    beep(1200, 800);

  } else if (strcmp(code, "B8") == 0) {
    // GPS improving
    beep(1600, 180);

  } else if (strcmp(code, "B9") == 0) {
    // corrections availble to rover gpt
    beep(2300, 180);

  } else if (strcmp(code, "E1") == 0) {
    // GPS failure: two long low beeps
    beep(400, 1000);

  } else if (strcmp(code, "E2") == 0) {
    // Heading acquisition failed: one very long low beep
    beep(300, 1400);

  } else if (strcmp(code, "E3") == 0) {
    // Emergency stop / obstacle / stop command: rapid low alarm
    beep(500, 300);

  } else if (strcmp(code, "E9") == 0) {
    // Emergency stop / obstacle / stop command: rapid low alarm
    beep(500, 600);

  } else {
    sendAck("ERR,UNKNOWN_BUZZER_CODE");
    return;
  }

  lastBuzzerCodeEndMs = millis();

  Serial.print("OK,");
  Serial.println(code);
}

void handleDriveCommand(char *line) {
  char *token = strtok(line, ",");
  if (token == NULL) {
    sendAck("ERR,EMPTY_DRIVE");
    return;
  }

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
  if (line[0] == '\0') return;

  if (line[0] == 'S') {
    stopCar();
    lastCommandTime = millis();
    sendAck("OK,S");
    return;
  }

  if (line[0] == 'D') {
    handleDriveCommand(line);
    return;
  }

  if (line[0] == 'B' || line[0] == 'E') {
    playBuzzerCode(line);
    return;
  }

  // Compatibility commands, only for older/testing code.
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
  if (c == '\r') return;

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
    lineLen = 0;
    stopCar();
    sendAck("ERR,LINE_TOO_LONG");
  }
}

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buzzerGndPin, OUTPUT);

  digitalWrite(buzzerGndPin, LOW);
  digitalWrite(dirPin, FORWARD_DIR);
  analogWrite(pwmPin, PWM_STOP);

  steering.attach(servoPin);
  steering.write(SERVO_CENTER);

  Serial.begin(SERIAL_BAUD);

  stopCar();
  lastCommandTime = millis();

  Serial.println("READY,car_control,D_angle_pwm,buzzer");

  // Arduino boot chirp. This is not B1. B1 comes from Navigation.
  beep(700, 100);
  beep(1000, 100);
  lastBuzzerCodeEndMs = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    processSerialByte(c);
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopCar();
  }

  delay(5);
}
