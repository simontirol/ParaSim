#include <AccelStepper.h>

const int enaPin = 7;
const int dirPin = 8;
const int stepPin = 9;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Parameters
const float stepsPerRev = 400.0; // Driver Setting - Half Stepping

float maxRevPerSec = 10.0; // rev/s

float impulseAcc = 3.0;   // rev/s²

float targetRevs = 0.3; // revs

int intensity = 0; // intensity 0-255 = 0-100%

bool runStepper = false;

int movementPhase = 0;  // 0: nicht gestartet, 1: vorwärts, 2: rückwärts

String inputString = "";
bool inputComplete = false;

void serialEvent() {
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
    digitalWrite(enaPin, LOW); // Enable motor coils
    stepper.setCurrentPosition(0);
    stepper.setAcceleration(impulseAcc * stepsPerRev);
    stepper.setMaxSpeed(maxRevPerSec * stepsPerRev);
    stepper.moveTo(targetRevs * stepsPerRev);
    runStepper = true;
    movementPhase = 1;
    Serial.println("Motor started (Phase 1).");
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
  } else if (cmd.startsWith("MAX_SPEED ")) {
    maxRevPerSec = cmd.substring(10).toFloat();
    Serial.print("New MAX_SPEED: ");
    Serial.println(maxRevPerSec);
  } else if (cmd.startsWith("INTENSITY ")) {
    int value = cmd.substring(10).toInt();
    value = constrain(value, -255, 255);
    intensity = value;
    Serial.print("New INTENSITY: ");
    Serial.println(intensity);

    // Calculate factor from absolute intensity
    float factor = abs(intensity) / 255.0;
    int direction = (intensity >= 0) ? 1 : -1;

    // Configure and move motor
    stepper.setCurrentPosition(0);
    digitalWrite(enaPin, LOW); // Enable motor coils
    stepper.setAcceleration(impulseAcc * stepsPerRev * factor);
    stepper.setMaxSpeed(maxRevPerSec * stepsPerRev * factor);
    stepper.moveTo(direction * targetRevs * stepsPerRev);
    runStepper = true;
    movementPhase = 1;

    Serial.print("Motor started in direction: ");
    Serial.println((direction == 1) ? "right" : "left");
  } else if (cmd == "ENABLE") {
  digitalWrite(enaPin, LOW);  // Disable coils
  Serial.println("Motor coil on.");
  } else if (cmd == "DISABLE") {
  digitalWrite(enaPin, HIGH);  // Disable coils
  Serial.println("Motor coil off.");
  } else {
  Serial.println("Unknown command.");
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH); // Disable motor by default

  Serial.println("Stepper impulse start system ready.");
  Serial.println("Commands:");
  Serial.println("  START");
  Serial.println("  STOP");
  Serial.println("  TARGET <rev>");
  Serial.println("  IMPULSE_ACC <rev/s²>");
  Serial.println("  MAX_SPEED <rev/s>");
  Serial.println("  INTENSITY <-255...+255> (auto-starts motor)");
  Serial.println("  ENABLE activate motor coils");
  Serial.println("  DISABLE deactivate motor coils");
}

void loop() {
  if (inputComplete) {
    processSerialCommand(inputString);
    inputString = "";
    inputComplete = false;
  }

  if (runStepper) {
    stepper.run();

    if (stepper.distanceToGo() == 0 && abs(stepper.speed()) < 1) {
      // Movement finished
      runStepper = false;
      movementPhase = 0;

      digitalWrite(enaPin, HIGH); // Disable motor coils
      Serial.println("Stepper move complete. Motor disabled.");
    }
  }
}
