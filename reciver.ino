#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// -------------- LED PINS ---------------
int ledRedOne = 3;   // LINK / timeout error
int ledRedTwo = 4;   // heartbeat
int ledYellow = 7;   // RX blink
int ledGreen  = 8;   // OK (receiving)

// ---------------- RADIO ----------------
RF24 radio(9, 10);           // CE, CSN
const byte address[6] = "IMU01";

// MUST MATCH TRANSMITTER
struct IMUData {
  float pitch;
  float yaw;
};

IMUData imuData;

// ---------------- SERVO (PITCH) ----------------
Servo pitchServo;
#define SERVO_PIN 6

const int SERVO_CENTER = 90;
const int SERVO_MIN    = 30;
const int SERVO_MAX    = 150;

// ---------------- STEPPER (YAW) ----------------
#define IN1 A0
#define IN2 A1
#define IN3 A2
#define IN4 A3

const int stepSeq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

int stepIndex = 0;

// 28BYJ-48 ≈ 4096 steps / 360°
const float STEPS_PER_DEG = 4096.0 / 360.0;

long currentStepPos = 0;
long targetStepPos  = 0;

// ---------------- SIGNAL TIMEOUT ----------------
unsigned long lastRXTime = 0;
const unsigned long SIGNAL_TIMEOUT = 500; // ms

// ---------------- LED TIMERS ----------------
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_MS = 700;

unsigned long yellowOffAt = 0;         // time to turn yellow off after blink
const unsigned long YELLOW_BLINK_MS = 30;

// ---------------- STEPPER FUNCTIONS ----------------
void setStepper(int idx) {
  digitalWrite(IN1, stepSeq[idx][0]);
  digitalWrite(IN2, stepSeq[idx][1]);
  digitalWrite(IN3, stepSeq[idx][2]);
  digitalWrite(IN4, stepSeq[idx][3]);
}

void stepOnce(int dir) {
  stepIndex += dir;
  if (stepIndex > 7) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 7;
  setStepper(stepIndex);
  currentStepPos += dir;
}

void stepMultiple(int steps) {
  int dir = (steps >= 0) ? 1 : -1;
  steps = abs(steps);

  for (int i = 0; i < steps; i++) {
    stepOnce(dir);
    delay(3);   // step speed (safe)
  }
}

void updateStepper() {
  if (currentStepPos < targetStepPos) {
    stepOnce(1);
  } else if (currentStepPos > targetStepPos) {
    stepOnce(-1);
  }
}

// ---------------- STARTUP MOTOR TEST ----------------
void motorSelfTest() {
  Serial.println(F("=== MOTOR SELF TEST START ==="));

  // ---- SERVO TEST ----
  Serial.println(F("Servo test: 0 -> 180 -> center"));
  pitchServo.write(0);   delay(600);
  pitchServo.write(180); delay(600);
  pitchServo.write(SERVO_CENTER); delay(600);

  // ---- STEPPER TEST ----
  Serial.println(F("Stepper test: 180 CW"));
  int steps180 = (int)(180.0 * STEPS_PER_DEG);
  stepMultiple(steps180);
  delay(500);

  Serial.println(F("Stepper test: 180 CCW"));
  stepMultiple(-steps180);
  delay(500);

  // Ensure centered
  targetStepPos = 0;
  currentStepPos = 0;

  Serial.println(F("=== MOTOR SELF TEST DONE ==="));
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  Serial.println(F("RX booting"));

  // LED pins
  pinMode(ledRedOne, OUTPUT);
  pinMode(ledRedTwo, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  digitalWrite(ledRedOne, LOW);
  digitalWrite(ledRedTwo, LOW);
  digitalWrite(ledYellow, LOW);
  digitalWrite(ledGreen, LOW);

  // Stepper pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  setStepper(0);

  // Servo
  pitchServo.attach(SERVO_PIN);
  pitchServo.write(SERVO_CENTER);
  delay(500);

  // ---- OPTIONAL SELF TEST ----
  // motorSelfTest();

  // ---- RADIO INIT ----
  if (!radio.begin()) {
    Serial.println(F("NRF24 init FAILED"));
    // Solid Red1 if radio init fails
    digitalWrite(ledRedOne, HIGH);
    while (1) {
      // heartbeat to show MCU alive even if radio dead
      if (millis() - lastHeartbeat > HEARTBEAT_MS) {
        lastHeartbeat = millis();
        digitalWrite(ledRedTwo, !digitalRead(ledRedTwo));
      }
    }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, address);
  radio.startListening();

  lastRXTime = millis();
  lastHeartbeat = millis();
  Serial.println(F("RX ready & listening"));
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  // ---- Heartbeat (Red2) ----
  if (now - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = now;
    digitalWrite(ledRedTwo, !digitalRead(ledRedTwo));
  }

  // ---- Turn off yellow after blink window ----
  if (yellowOffAt != 0 && now >= yellowOffAt) {
    digitalWrite(ledYellow, LOW);
    yellowOffAt = 0;
  }

  if (radio.available()) {
    radio.read(&imuData, sizeof(imuData));
    lastRXTime = now;

    // RX indicator (Yellow quick blink)
    digitalWrite(ledYellow, HIGH);
    yellowOffAt = now + YELLOW_BLINK_MS;

    // ---- SERVO (PITCH) ----
    float pitch = constrain(imuData.pitch, -60.0, 60.0);
    int servoAngle = map((int)pitch, -60, 60, SERVO_MIN, SERVO_MAX);
    pitchServo.write(servoAngle);

    // ---- STEPPER (YAW) ----
    float yaw = constrain(imuData.yaw, -60.0, 60.0);
    targetStepPos = (long)(yaw * STEPS_PER_DEG);
  }

  // ---- LINK OK / LOST LOGIC ----
  bool linkOK = (now - lastRXTime <= SIGNAL_TIMEOUT);

  if (linkOK) {
    digitalWrite(ledGreen, HIGH);   // receiving recently
    digitalWrite(ledRedOne, LOW);   // no error
  } else {
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledRedOne, HIGH);  // timeout / no packets

    // safety reset when signal lost
    targetStepPos = 0;
    pitchServo.write(SERVO_CENTER);
  }

  updateStepper();
  delay(2);
}
