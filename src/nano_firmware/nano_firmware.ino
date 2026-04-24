#include <Wire.h>

// --- MOTOR PINS ---
const int PWM_PIN = 10;
const int LEFT_BWD = A0;
const int LEFT_FWD = A1;
const int RIGHT_BWD = 11;
const int RIGHT_FWD = 12;

// --- QUADRATURE ENCODER PINS ---
const int ENC_FL_A = 2;
const int ENC_FL_B = 3; 
const int ENC_FR_A = 4;
const int ENC_FR_B = 5; 
const int ENC_RL_A = 6;
const int ENC_RL_B = 7; 
const int ENC_RR_A = 8;
const int ENC_RR_B = 9; 

// --- ENCODER VARIABLES ---
volatile long ticksFL = 0;
long ticksFR = 0;
long ticksRL = 0;
long ticksRR = 0;

int lastStateFR_A = LOW;
int lastStateRL_A = LOW;
int lastStateRR_A = LOW;

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

// --- LIGHTWEIGHT DEADMAN SWITCH ---
unsigned long lastCmdTime = 0;
bool isMoving = false;

void setup() {
  Serial.begin(115200);

  // 1. Motor Setup
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT); pinMode(LEFT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT); pinMode(RIGHT_FWD, OUTPUT);

  // 2. Encoder Setup (All 8 pins as inputs with pullups)
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  // Attach hardware interrupt for Front Left ONLY (Pin 2)
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), countFL, RISING);

  // Read initial states for the polled wheels
  lastStateFR_A = digitalRead(ENC_FR_A);
  lastStateRL_A = digitalRead(ENC_RL_A);
  lastStateRR_A = digitalRead(ENC_RR_A);

  // 3. Ultrasonic Setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 4. IMU Setup
  delay(200); 
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up the MPU-6050
  Wire.endTransmission(true);
}

void loop() {
  recvWithEndMarker();
  parseData();
  
  // PASSIVE FAILSAFE
  if (isMoving && (millis() - lastCmdTime > 500)) {
    setMotors(0, 0);
    isMoving = false;
  }

  pollEncoders();     // Catch D4, D6, D8 wheel ticks instantly
  publishData();      // Send Encoders, IMU, and Ultrasonic every 50ms
}

// --- INTERRUPTS & POLLING (QUADRATURE LOGIC) ---

// Hardware Interrupt for Front Left
void countFL() { 
  if (digitalRead(ENC_FL_B) == LOW) { ticksFL--; } 
  else { ticksFL++; } 
}

// Fast polling for the other 3 wheels
void pollEncoders() {
  // Front Right
  int stateFR_A = digitalRead(ENC_FR_A);
  if (stateFR_A == HIGH && lastStateFR_A == LOW) { 
    if (digitalRead(ENC_FR_B) == LOW) { ticksFR++; } else { ticksFR--; }
  }
  lastStateFR_A = stateFR_A;

  // Rear Left
  int stateRL_A = digitalRead(ENC_RL_A);
  if (stateRL_A == HIGH && lastStateRL_A == LOW) { 
    if (digitalRead(ENC_RL_B) == LOW) { ticksRL--; } else { ticksRL++; }
  }
  lastStateRL_A = stateRL_A;

  // Rear Right
  int stateRR_A = digitalRead(ENC_RR_A);
  if (stateRR_A == HIGH && lastStateRR_A == LOW) { 
    if (digitalRead(ENC_RR_B) == LOW) { ticksRR++; } else { ticksRR--; }
  }
  lastStateRR_A = stateRR_A;
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

  int MIN_PWM = 130; 
  
  if (leftSpeed > 0 && leftSpeed < MIN_PWM) leftSpeed = MIN_PWM;
  if (leftSpeed < 0 && leftSpeed > -MIN_PWM) leftSpeed = -MIN_PWM;
  
  if (rightSpeed > 0 && rightSpeed < MIN_PWM) rightSpeed = MIN_PWM;
  if (rightSpeed < 0 && rightSpeed > -MIN_PWM) rightSpeed = -MIN_PWM;

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
    Wire.write(0x3B); 
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
