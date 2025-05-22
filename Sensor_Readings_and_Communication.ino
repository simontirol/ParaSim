#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>
#include <HX711.h>
#include <math.h>

// --- Load Cells ---
HX711 load1;
HX711 load2;

#define LOAD1_DOUT 22
#define LOAD1_SCK  23
#define LOAD2_DOUT 26
#define LOAD2_SCK  27

float minValue1 = 0;
float minValue2 = 0;
float maxValue1 = 1000;
float maxValue2 = 1000;

bool invert1 = false;
bool invert2 = false;

// --- ToF Sensors ---
VL53L1X tof1;
VL53L1X tof2;

#define TOF1_XSHUT 8

bool tofAvailable = false;
//#define TOF2_XSHUT 19

// --- MPU6050 ---
MPU6050 mpu;

float alpha = 0.96;
float roll = 0;

float rollOffset = 0.0;
bool mpuAvailable = false;

unsigned long lastTime = 0;

const float accelScale = 1.0 / 16384.0;
const float gyroScale = 1.0 / 131.0;

// --- UART ---
#define BAUDRATE 9600

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 50;

unsigned long lastToFResetTime = 0;
const unsigned long tofResetCooldown = 3000;

void setupToFSensors() {
  pinMode(TOF1_XSHUT, OUTPUT);
  digitalWrite(TOF1_XSHUT, HIGH);
  if (tof1.init()) {
    tofAvailable = true;
    tof1.setDistanceMode(VL53L1X::Short);
    tof1.setTimeout(500);
    tof1.startContinuous(50);
    Serial.println("VL53L1X_1 initialized.");
  } else {
    Serial.println("VL53L1X_1 not found — skipping ToF.");
    tofAvailable = false;
  }
}

void resetToF1() {
  Serial.println("Resetting VL53L1X sensor...");
  digitalWrite(TOF1_XSHUT, LOW);
  delay(10);
  digitalWrite(TOF1_XSHUT, HIGH);
  delay(100);

  if (tof1.init()) {
    tof1.setDistanceMode(VL53L1X::Short);
    tof1.setTimeout(500);
    tof1.startContinuous(50);
    tofAvailable = true;
    Serial.println("VL53L1X reset successful.");
  } else {
    Serial.println("VL53L1X reset failed.");
    tofAvailable = false;
  }
}

int readToFDistanceMapped() {
  if (!tofAvailable) return -1;

  uint16_t d1 = tof1.read();
  if (tof1.timeoutOccurred()) {
    unsigned long now = millis();
    if (now - lastToFResetTime > tofResetCooldown) {
      Serial.println("ToF1 timeout – resetting sensor...");
      resetToF1();
      lastToFResetTime = now;
    } else {
      Serial.println("ToF1 timeout – waiting before next reset.");
    }
    return -1;
  }

  int mapped = map(d1, 100, 250, 255, 0);
  return constrain(mapped, 0, 255);
}

void setupMPUwithOffset() {
  mpu.initialize();
  mpuAvailable = mpu.testConnection();

  if (!mpuAvailable) {
    Serial.println("MPU6050 not responding!");
    return;
  }

  // --- Average gyro over N samples
  const int samples = 100;
  float gyroSum = 0.0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gyroSum += gy * gyroScale;
    delay(2); // ~500 Hz sampling
  }

  float gyroBias = gyroSum / samples;  // degrees/second

  Serial.print("Gyro bias (deg/s): ");
  Serial.println(gyroBias);

  // --- Get current acceleration-based roll
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float axg = ax * accelScale;
  float ayg = ay * accelScale;
  float azg = az * accelScale;

  float denominator = sqrt(axg * axg + azg * azg);
  if (denominator != 0) {
    rollOffset = atan2(ayg, denominator) * 180.0 / PI;
  }

  // --- Integrate gyro drift for short init duration
  const float initTime = samples * 0.002; // seconds
  float estimatedDrift = gyroBias * initTime;

  // --- Final initial roll estimate: accel + gyro drift
  roll = rollOffset + estimatedDrift;

  Serial.print("Initial roll (accel): ");
  Serial.println(rollOffset);
  Serial.print("Estimated drift (gyro): ");
  Serial.println(estimatedDrift);
  Serial.print("Final roll: ");
  Serial.println(roll);

  lastTime = micros();
  Serial.println("MPU initialized with accel+gyro estimation.");
}


void resetMPU() {
  mpu.initialize();
  delay(10);

  if (mpu.testConnection()) {
    mpuAvailable = true;
    Serial.println("MPU6050 reset successful.");
  } else {
    mpuAvailable = false;
    Serial.println("MPU6050 still not responding!");
  }
}

int readMappedRoll() {
  if (!mpuAvailable) return 0;

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 disconnected! Attempting reset...");
    resetMPU();
    return 0;
  }

  unsigned long lastTime = micros();
  float dt = (micros() - lastTime) / 1000000.0; // seconds


  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // --- Berechnung aus Beschleunigung
  float axg = ax * accelScale;
  float ayg = ay * accelScale;
  float azg = az * accelScale;

  float accDenom = sqrt(axg * axg + azg * azg);
  if (accDenom == 0) return 0;

  float accRoll = atan2(ayg, accDenom) * 180.0 / PI;

  // --- Berechnung aus Gyro
  float gyroRate = gy * gyroScale; // °/s bei ±250°/s
  float gyroRoll = roll + gyroRate * dt;

  // --- Komplementärfilter
  roll = alpha * gyroRoll + (1.0 - alpha) * accRoll;

  // --- Offset entfernen
  float relativeRoll = roll - rollOffset;

  // --- Mapping auf PWM oder andere Werte
  int mappedRoll = map(relativeRoll, -10, 10, -255, 255);
  return constrain(mappedRoll, -255, 255);
}



void readLoadCellsMapped(int &mapped1, int &mapped2) {
  float w1 = load1.get_units();
  float w2 = load2.get_units();

  mapped1 = constrain(map(w1, minValue1, maxValue1, 0, 255), 0, 255);
  mapped2 = constrain(map(w2, minValue2, maxValue2, 0, 255), 0, 255);
}

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();
  //Wire.setClock(50000);

  Serial.println("I2C-Device-Scan:");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  gefunden: 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scan finished.\n");

  load1.begin(LOAD1_DOUT, LOAD1_SCK);
  load2.begin(LOAD2_DOUT, LOAD2_SCK);
  delay(2000);
  minValue1 = load1.get_units();
  minValue2 = load2.get_units();
  Serial.println("Calibrating brakes - maximal Pull...");
  delay(3000);
  maxValue1 = load1.get_units();
  maxValue2 = load2.get_units();

  setupToFSensors();

  Serial.println("Setting up MPU - sit still...");
  delay(2000);

  setupMPUwithOffset();

  Serial.println("System ready!");
  Serial.println("Brake_Right\tBrake_Left\tRoll\tAcceleration");
  delay(2000);
}

void loop() {

  int mappedLoad1 = 0, mappedLoad2 = 0;
  int mappedDistance = readToFDistanceMapped();
  int mappedRoll = readMappedRoll();

  readLoadCellsMapped(mappedLoad1, mappedLoad2);

  Serial.print(mappedLoad1);        // Brake_Right
  Serial.print("\t");
  Serial.print(mappedLoad2);        // Brake_Left
  Serial.print("\t");
  Serial.print(mappedRoll);         // Roll
  Serial.print("\t");
  Serial.println(mappedDistance);   // Acceleration (ToF)

}