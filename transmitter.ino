#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050.h>

// ---------------- PIN DEFINITIONS ----------------
#define LED_ERROR   3   // Red #1 - radio / system error
#define LED_CALIB   4   // Red #2 - calibration status
#define LED_READY   7   // Green   - system OK / calibrated
#define LED_TX      8   // Blue    - TX activity

#define BTN_CALIB   2   // Calibration button (external pull-down: 10k to GND, button to 5V)

// ---------------- RADIO ----------------
RF24 radio(9, 10);      // CE, CSN
const byte address[6] = "IMU01";

struct IMUData {
  float pitch;
  float yaw;
};

IMUData imuData;

// ---------------- MPU ----------------
MPU6050 mpu;

float pitchOffset = 0.0f;
float gyroZOffset = 0.0f;
float yawAngle    = 0.0f;

// Simple low-pass filter (smoothing)
float filteredPitch = 0.0f;
float filteredYaw   = 0.0f;
const float filterAlpha = 0.9f;   // 0..1 (higher = smoother, slower response)

unsigned long lastTime = 0;

// -------------------------------------------------
//              CALIBRATION FUNCTION
// -------------------------------------------------
void calibrateMPU() {
  Serial.println(F("Starting calibration... keep the board still."));

  digitalWrite(LED_CALIB, HIGH);
  digitalWrite(LED_READY, LOW);

  const int samples = 400;
  long pitchSum = 0;
  long gzSum    = 0;

  for (int i = 0; i < samples; i++) {
    // blink calib LED during calibration
    digitalWrite(LED_CALIB, (i % 20 < 10));

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float pitch = atan2(-(float)ax,
                        sqrt((float)ay * ay + (float)az * az)) * 57.3f;

    pitchSum += (long)pitch;
    gzSum    += (long)gz;

    delay(5);
  }

  pitchOffset = (float)pitchSum / (float)samples;
  gyroZOffset = (float)gzSum    / (float)samples;

  // Reset yaw reference to 0 at calibration moment
  yawAngle = 0.0f;

  Serial.print(F("Pitch offset: "));
  Serial.println(pitchOffset);
  Serial.print(F("Gyro Z offset: "));
  Serial.println(gyroZOffset);

  digitalWrite(LED_CALIB, LOW);
  digitalWrite(LED_READY, HIGH);

  Serial.println(F("Calibration DONE ✔"));
}

// -------------------------------------------------
//                      SETUP
// -------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println(F("TX: IMU + NRF24 starting..."));

  // LEDs
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_CALIB, OUTPUT);
  pinMode(LED_READY, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_CALIB, LOW);
  digitalWrite(LED_READY, LOW);
  digitalWrite(LED_TX, LOW);

  // Button (external pull-down: LOW = not pressed, HIGH = pressed)
  pinMode(BTN_CALIB, INPUT);

  // I2C & MPU init
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println(F("MPU6050 detected ✔"));
  } else {
    Serial.println(F("MPU6050 NOT detected ❌"));
    digitalWrite(LED_ERROR, HIGH);
  }

  // Radio init
  if (!radio.begin()) {
    Serial.println(F("NRF24 init FAILED ❌"));
    digitalWrite(LED_ERROR, HIGH);
    while (1) {
      delay(500);
    }
  }

  // Radio configuration
  radio.setPALevel(RF24_PA_LOW);        // can go to RF24_PA_MAX later
  radio.setDataRate(RF24_250KBPS);      // more robust
  radio.openWritingPipe(address);
  radio.stopListening();                // TX mode
  radio.setAutoAck(true);               // expect ACK from receiver

  Serial.println(F("NRF24 init OK ✔"));

  // Initial calibration at startup
  calibrateMPU();

  lastTime = millis();
}

// -------------------------------------------------
//                      LOOP
// -------------------------------------------------
void loop() {
  // ---- Button: recalibrate when pressed ----
  if (digitalRead(BTN_CALIB) == HIGH) {   // active HIGH (button to 5V)
    Serial.println(F("Button pressed → recalibrating..."));
    calibrateMPU();
    delay(300); // simple debounce & avoid repeated triggers
  }

  // ---- Read MPU ----
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Pitch from accelerometer (in degrees)
  float rawPitch = atan2(-(float)ax,
                         sqrt((float)ay * ay + (float)az * az)) * 57.3f;
  float pitch = rawPitch - pitchOffset;

  // Yaw from gyro Z integration (deg/s * dt)
  float gzCorrected = (float)gz - gyroZOffset;
  float yawRate = gzCorrected / 131.0f;   // 131 LSB/(deg/s) for ±250°/s
  yawAngle += yawRate * dt;

  // Keep yaw in -180..180 range
  if (yawAngle > 180.0f)  yawAngle -= 360.0f;
  if (yawAngle < -180.0f) yawAngle += 360.0f;

  // ---- Simple low-pass filter for smoothness ----
  filteredPitch = filterAlpha * filteredPitch + (1.0f - filterAlpha) * pitch;
  filteredYaw   = filterAlpha * filteredYaw   + (1.0f - filterAlpha) * yawAngle;

  imuData.pitch = filteredPitch;
  imuData.yaw   = filteredYaw;

  // ---- Send over radio ----
  bool ok = radio.write(&imuData, sizeof(imuData));

  // TX LED: short blink each send
  digitalWrite(LED_TX, HIGH);
  delay(3);
  digitalWrite(LED_TX, LOW);

  // Error LED: ON if radio write failed (no ACK), OFF if OK
  if (!ok) {
    digitalWrite(LED_ERROR, HIGH);
  } else {
    digitalWrite(LED_ERROR, LOW);
  }

  // ---- Debug Serial ----
  Serial.print(F("P: "));
  Serial.print(imuData.pitch);
  Serial.print(F("  Y: "));
  Serial.print(imuData.yaw);
  Serial.print(F(" | radio: "));
  Serial.println(ok ? F("OK") : F("FAIL"));

  delay(25);   // ~40 Hz update rate
}
