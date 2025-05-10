#include <AccelStepper.h>
#include <HX711.h>
#include <MPU6050.h>
#include <VL53L1X.h>
#include <Wire.h>

// Load Cells
HX711 load1;
HX711 load2;
#define LOAD1_DOUT  34
#define LOAD1_SCK   35
#define LOAD2_DOUT  32
#define LOAD2_SCK   33

// ToF Sensors
VL53L1X tof1;
VL53L1X tof2;
#define TOF1_XSHUT 18
#define TOF2_XSHUT 19

// IMU
//MPU6050 imu;

// Stepper
#define STEP_PIN 25
#define DIR_PIN 26
#define ENA_PIN 27
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


// Parameters
// Stepper
const float stepsPerRev = 400.0;

float maxRevPerSec = 5.0;
float smoothRevPerSec = maxRevPerSec;
float targetRevPerSec = 0.0;

float impulseAcc = 600.0;   // rev/s²
float smoothAcc = 50.0;

float targetRevs = 20.0;

// Input/Output

String inputString = "";
bool inputComplete = false;

bool runStepper = false;
bool switchedToSmooth = false;

float userWeight = 0;
float intensity = 0;

// Loadcell
long minValue1 = 0;       // Ohne Belastung
long minValue2 = 0;
long maxValue1 = 100000;   // Ziel-Belastung
long maxValue2 = 100000;   // Ziel-Belastung


void handleSerialInput() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputString += inChar;
    }
  }
}


void processSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "START") {
    stepper.setCurrentPosition(0);
    stepper.setAcceleration(impulseAcc * stepsPerRev);
    stepper.setMaxSpeed(maxRevPerSec * stepsPerRev);
    stepper.moveTo(targetRevs * stepsPerRev);
    runStepper = true;
    switchedToSmooth = false;
    Serial.println("Motor impulse started.");
  } else if (cmd == "STOP") {
    runStepper = false;
    Serial.println("Motor stopped.");
  } else if (cmd.startsWith("TARGET ")) {
    targetRevs = cmd.substring(7).toFloat();
    Serial.print("New TARGET: ");
    Serial.println(targetRevs);
  } else if (cmd.startsWith("IMPULSE_ACC ")) {
    impulseAcc = cmd.substring(12).toFloat();
    Serial.print("New IMPULSE_ACC: ");
    Serial.println(impulseAcc);
  } else if (cmd.startsWith("SMOOTH_ACC ")) {
    smoothAcc = cmd.substring(11).toFloat();
    Serial.print("New SMOOTH_ACC: ");
    Serial.println(smoothAcc);
  } else if (cmd.startsWith("MAX_SPEED ")) {
    maxRevPerSec = cmd.substring(10).toFloat();
    Serial.print("New MAX_SPEED: ");
    Serial.println(maxRevPerSec);
  } else if (cmd.startsWith("WEIGHT ")) {
    userWeight = cmd.substring(7).toFloat();
    Serial.print("New WEIGHT: ");
    Serial.print(userWeight);
    Serial.println(" kg");
  } else if (cmd.startsWith("INTENSITY ")) {
    intensity = cmd.substring(10).toFloat();
    intensity = constrain(intensity, 0.0, 1.0);
    Serial.print("New INTENSITY: ");
    Serial.println(intensity);

    targetRevPerSec = intensity * (userWeight/100) * maxRevPerSec;
    
    stepper.setCurrentPosition(0);
    stepper.setAcceleration(impulseAcc * stepsPerRev);
    stepper.setMaxSpeed(targetRevPerSec * stepsPerRev);
    stepper.moveTo(targetRevs * stepsPerRev);
    runStepper = true;
    switchedToSmooth = false;
  } else {
    Serial.println("Unknown command.");
  }
}


void setupToFSensors() {
  pinMode(TOF1_XSHUT, OUTPUT);
  pinMode(TOF2_XSHUT, OUTPUT);

  // Turn off both
  digitalWrite(TOF1_XSHUT, LOW);
  digitalWrite(TOF2_XSHUT, LOW);
  delay(10);

  // Initialize TOF1
  digitalWrite(TOF1_XSHUT, HIGH);
  delay(10);
  tof1.init(true);
  tof1.setAddress(0x30);
  tof1.setTimeout(500);
  if (!tof1.init()) {
    Serial.println("VL53L1X 1 nicht gefunden!");
    while (1);
  }
  tof1.setDistanceMode(VL53L1X::Short);
  //tof1.setMeasurementTimingBudget(50000);
  //tof1.startContinuous(50);

  // Initialize TOF2
  digitalWrite(TOF2_XSHUT, HIGH);
  delay(10);
  tof2.init(true);
  tof2.setAddress(0x31);
  tof2.setTimeout(500);
  if (!tof2.init()) {
    Serial.println("VL53L1X 2 nicht gefunden!");
    while (1);
  }
  tof2.setDistanceMode(VL53L1X::Short);
  //tof2.setMeasurementTimingBudget(50000);
  //tof2.startContinuous(50);
}


void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22); // I2C Pins: ESP32 SDA = 21, SCL = 22

  // --- Stepper setup ---
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  Serial.println("Stepper impulse start system ready.");
  Serial.println("Commands:");
  Serial.println("  START");
  Serial.println("  STOP");
  Serial.println("  TARGET <rev>");
  Serial.println("  IMPULSE_ACC <rev/s²>");
  Serial.println("  SMOOTH_ACC <rev/s²>");
  Serial.println("  MAX_SPEED <rev/s>");

  // --- Loadcells setup ---
  load1.begin(LOAD1_DOUT, LOAD1_SCK);
  load2.begin(LOAD2_DOUT, LOAD2_SCK);
  delay(500);

  // Initial reference
  minValue1 = load1.get_units(10);
  minValue2 = load1.get_units(10);

  // --- VL53L1X setup ---
  setupToFSensors();

  // --- MPU6050 setup ---
  //imu.initialize();
  //if (!imu.testConnection()) {
    //Serial.println("MPU6050 Verbindung fehlgeschlagen!");
    //while (1);
  //}
  Serial.println("Sensoren bereit!");

  // Userweight configuration
  Serial.println("Bitte Gewicht mit Befehl eingeben: WEIGHT <kg> (z.B. WEIGHT 72.5)");
  while (userWeight <= 0) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      processSerialCommand(input); 
    }
  }

  Serial.print("Gewicht bestätigt: ");
  Serial.print(userWeight);
  Serial.println(" kg");

}


void loop() {
  // --- Serial input handler ---
  handleSerialInput();

  // --- Command processing ---
  if (inputComplete) {
    processSerialCommand(inputString);
    inputString = "";
    inputComplete = false;
  }

  // --- Stepper control ---
  handleStepperMotor();

  // --- Sensor readings ---
  int mappedDistance = readToFDistanceMapped();
  //int mappedRoll = readMappedRoll();
  int mappedLoad1 = 0, mappedLoad2 = 0;
  readLoadCellsMapped(mappedLoad1, mappedLoad2);

  // --- Serial output ---
  Serial.print("Load1: "); Serial.print(mappedLoad1);
  Serial.print(" | Load2: "); Serial.print(mappedLoad2);
  if (mappedDistance >= 0) {
    Serial.print(" | Distanz (gemappt): "); Serial.print(mappedDistance);
  }
  //Serial.print(" | Roll (gemappt): "); Serial.println(mappedRoll);
}


void handleStepperMotor() {
  if (runStepper) {
    stepper.run();

    if (!switchedToSmooth && abs(stepper.speed()) >= maxRevPerSec * stepsPerRev) {
      stepper.setAcceleration(smoothAcc * stepsPerRev);
      stepper.setMaxSpeed(smoothRevPerSec * stepsPerRev);

      // Calculate how far we need to travel to decelerate to 0
      float v = stepper.speed(); // current speed (steps/s)
      float a = smoothAcc * stepsPerRev; // acceleration in steps/s²
      long decelSteps = (v * v) / (2 * a); // classic physics formula

      int dir = (stepper.targetPosition() - stepper.currentPosition()) >= 0 ? 1 : -1;
      stepper.move(dir * decelSteps); // move extra steps to decelerate
      switchedToSmooth = true;
    }

    // Stop only when speed is zero and no more distance
    if (switchedToSmooth && stepper.distanceToGo() == 0 && abs(stepper.speed()) < 1) {
      runStepper = false;
      Serial.println("Stepper impulse and deceleration complete.");
    }
  }
}


// --- VL53L1X: Averaging and mapping ---
int readToFDistanceMapped() {
  uint16_t distance1 = tof1.read();
  uint16_t distance2 = tof2.read();

  if (tof1.timeoutOccurred() || tof2.timeoutOccurred()) {
    if (tof1.timeoutOccurred()) Serial.println("VL53L1X Sensor 1 Timeout!");
    if (tof2.timeoutOccurred()) Serial.println("VL53L1X Sensor 2 Timeout!");
    return -1; // return invalid
  }

  uint16_t average = (distance1 + distance2) / 2;
  int mapped = map(average, 220, 90, 0, 255);
  return constrain(mapped, 0, 255);
}


// --- MPU6050: Roll-measurement and mapping ---
//int readMappedRoll() {
  //int16_t ax, ay, az;
  //imu.getAcceleration(&ax, &ay, &az);

//  float axg = ax / 16384.0;
 // float ayg = ay / 16384.0;
  //float azg = az / 16384.0;

  //float roll = atan2(ayg, sqrt(axg * axg + azg * azg)) * 180.0 / PI;
  //int mappedRoll = map((int)roll, -90, 90, 0, 255);
  //return constrain(mappedRoll, 0, 255);
//}


// --- Load Cells: Mapping and adaptive MaxValue ---
void readLoadCellsMapped(int &mapped1, int &mapped2) {
  long raw1 = load1.read();
  long raw2 = load2.read();

  if (raw1 > maxValue1) maxValue1 = raw1;
  if (raw2 > maxValue2) maxValue2 = raw2;

  mapped1 = map(raw1, minValue1, maxValue1, 0, 255);
  mapped1 = constrain(mapped1, 0, 255);

  mapped2 = map(raw2, minValue2, maxValue2, 0, 255);
  mapped2 = constrain(mapped2, 0, 255);
}
