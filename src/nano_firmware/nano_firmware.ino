#include <Wire.h>

// --- MOTOR PINS ---
const int PWM_PIN = 10;
const int LEFT_BWD = A0;
const int LEFT_FWD = A1;
const int RIGHT_BWD = 11;
const int RIGHT_FWD = 12;

// --- ENCODER PINS & VARIABLES ---
const int ENC_FL_PIN = 2; // Supports Hardware Interrupt
const int ENC_FR_PIN = 3; // Supports Hardware Interrupt
const int ENC_RL_PIN = 4; // Fast-polled in loop
const int ENC_RR_PIN = 5; // Fast-polled in loop

volatile long ticksFL = 0;
volatile long ticksFR = 0;
long ticksRL = 0;
long ticksRR = 0;

int lastStateRL = LOW;
int lastStateRR = LOW;
int leftDir = 1;
int rightDir = 1;

// --- ULTRASONIC PINS ---
const int TRIG_PIN = A2;
const int ECHO_PIN = A3;

// --- NON-BLOCKING SERIAL VARIABLES ---
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// --- TIMING & IMU VARIABLES ---
const int MPU_ADDR = 0x68;
unsigned long previousMillis = 0;
const long interval = 50; // 50ms = 20Hz update rate

// --- NEW: LIGHTWEIGHT DEADMAN SWITCH ---
unsigned long lastCmdTime = 0;
bool isMoving = false;

void setup() {
  Serial.begin(115200);

  // 1. Motor Setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT); pinMode(LEFT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT); pinMode(RIGHT_FWD, OUTPUT);

  // 2. Encoder Setup
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_PIN, INPUT_PULLUP);

  // Attach hardware interrupts for front wheels
  attachInterrupt(digitalPinToInterrupt(ENC_FL_PIN), countFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_PIN), countFR, RISING);

  // Read initial states for rear wheels
  lastStateRL = digitalRead(ENC_RL_PIN);
  lastStateRR = digitalRead(ENC_RR_PIN);

  // 3. Ultrasonic Setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 4. IMU Setup
  delay(200); // Give the MPU6050 time to stabilize power
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up the MPU-6050
  Wire.endTransmission(true);
}

void loop() {
  recvWithEndMarker();
  parseData();
  
  // --- NEW: PASSIVE FAILSAFE ---
  // If moving, and 500ms has passed since the last command, stop.
  if (isMoving && (millis() - lastCmdTime > 500)) {
    setMotors(0, 0);
    isMoving = false;
  }

  pollRearEncoders(); // Catch rear wheel ticks instantly
  publishData();      // Send Encoders, IMU, and Ultrasonic every 50ms
}

// --- INTERRUPTS & POLLING ---
void countFL() { ticksFL += leftDir; }
void countFR() { ticksFR += rightDir; }

void pollRearEncoders() {
  int stateRL = digitalRead(ENC_RL_PIN);
  if (stateRL == HIGH && lastStateRL == LOW) { ticksRL += leftDir; }
  lastStateRL = stateRL;

  int stateRR = digitalRead(ENC_RR_PIN);
  if (stateRR == HIGH && lastStateRR == LOW) { ticksRR += rightDir; }
  lastStateRR = stateRR;
}

// --- SERIAL COMMUNICATION ---
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) { ndx = numChars - 1; }
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }
}

void parseData() {
  if (newData == true) {
    if (receivedChars[0] == 'M' && receivedChars[1] == ':') {
      char *strTokIndx = strtok(receivedChars + 2, ",");
      if (strTokIndx != NULL) {
        int leftSpeed = atoi(strTokIndx);
        strTokIndx = strtok(NULL, ",");
        if (strTokIndx != NULL) {
          int rightSpeed = atoi(strTokIndx);
          setMotors(leftSpeed, rightSpeed);
          
          // --- NEW: Reset failsafe timer on valid command ---
          lastCmdTime = millis();
          isMoving = true;
        }
      }
    }
    newData = false;
  }
}

// --- MOTOR DRIVER ---
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  leftDir = (leftSpeed >= 0) ? 1 : -1;
  rightDir = (rightSpeed >= 0) ? 1 : -1;

  int absSpeed = (abs(leftSpeed) + abs(rightSpeed)) / 2;
  analogWrite(PWM_PIN, absSpeed);

  if (leftSpeed > 0) { digitalWrite(LEFT_FWD, HIGH); digitalWrite(LEFT_BWD, LOW); }
  else if (leftSpeed < 0) { digitalWrite(LEFT_FWD, LOW); digitalWrite(LEFT_BWD, HIGH); }
  else { digitalWrite(LEFT_FWD, LOW); digitalWrite(LEFT_BWD, LOW); }

  if (rightSpeed > 0) { digitalWrite(RIGHT_FWD, HIGH); digitalWrite(RIGHT_BWD, LOW); }
  else if (rightSpeed < 0) { digitalWrite(RIGHT_FWD, LOW); digitalWrite(RIGHT_BWD, HIGH); }
  else { digitalWrite(RIGHT_FWD, LOW); digitalWrite(RIGHT_BWD, LOW); }
}

// --- DATA PUBLISHER (20Hz) ---
void publishData() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 1. PUBLISH ENCODERS
    Serial.print("T:");
    Serial.print(ticksFL); Serial.print(",");
    Serial.print(ticksFR); Serial.print(",");
    Serial.print(ticksRL); Serial.print(",");
    Serial.println(ticksRR);

    // 2. PUBLISH IMU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting register for Accel Readings
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    if(Wire.available() >= 14){
      int16_t AcX = Wire.read()<<8|Wire.read();
      int16_t AcY = Wire.read()<<8|Wire.read();
      int16_t AcZ = Wire.read()<<8|Wire.read();
      int16_t Temp = Wire.read()<<8|Wire.read(); // Skip temp
      int16_t GyX = Wire.read()<<8|Wire.read();
      int16_t GyY = Wire.read()<<8|Wire.read();
      int16_t GyZ = Wire.read()<<8|Wire.read();

      float ax = AcX / 16384.0 * 9.81;
      float ay = AcY / 16384.0 * 9.81;
      float az = AcZ / 16384.0 * 9.81;
      float gx = GyX / 131.0 * (PI / 180.0);
      float gy = GyY / 131.0 * (PI / 180.0);
      float gz = GyZ / 131.0 * (PI / 180.0);

      Serial.print("I:");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);
    }

    // 3. PUBLISH ULTRASONIC
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 15000 microsecond timeout limits the block to ~2.5 meters.
    long duration = pulseIn(ECHO_PIN, HIGH, 15000);

    if (duration == 0) {
      Serial.println("U:MAX");
    } else {
      float distance_cm = duration * 0.0343 / 2.0;
      Serial.print("U:");
      Serial.println(distance_cm);
    }
  }
}

