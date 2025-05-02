#include <Arduino.h>
#include <math.h> // Include for round()

// Motor Pins
const int PUL_PIN = 4;
const int DIR_PIN = 3;
const int ENA_PIN = 5;

// Stepper settings
const int stepsPerRevolution = 200;
const int pulseWidthMicros = 10;
const int pulseDelayMicros = 500;

// --- NEW: Define the prefix for results from Pi ---
const String RESULT_PREFIX = "AgResult,";

// HC-12 Baud Rate
const long HC12_BAUD_RATE = 9600;

// Motor movement mapping
const int STEPS_PER_SCORE_UNIT = 25;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Motor Control Arduino - HC12 Result Receiver");
  Serial.println("-------------------------------------------");

  Serial1.begin(HC12_BAUD_RATE);
  Serial.print("Serial1 (HC-12) port opened at ");
  Serial.print(HC12_BAUD_RATE);
  Serial.println(" baud.");

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(PUL_PIN, LOW);
  // Enable driver (Assuming Active LOW enable - CHECK YOUR DRIVER)
  digitalWrite(ENA_PIN, LOW);
  Serial.println("Stepper driver enabled.");
  delay(10);
  Serial.println("Setup complete. Waiting for AgResult commands...");
}

// Function to rotate the motor (same as before)
void rotateSteps(int steps, bool clockwise) {
  if (steps <= 0) return;
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW); // Adjust if needed
  delayMicroseconds(100);
  // Serial.print(clockwise ? "  CW" : "  CCW"); Serial.print(" Stepping: "); Serial.println(steps);
  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH); delayMicroseconds(pulseWidthMicros);
    digitalWrite(PUL_PIN, LOW); delayMicroseconds(pulseDelayMicros);
  }
  // Serial.println("  Stepping complete.");
}

// Function to determine motor movement based ONLY on score (for now)
void moveMotorBasedOnScore(float score) {
  int roundedScore = round(score);
  Serial.print("Received score: "); Serial.print(score, 2);
  Serial.print(", Rounded: "); Serial.println(roundedScore);

  int stepsToMove = 0;
  bool moveClockwise = true;

  if (roundedScore > 0) {
    stepsToMove = constrain(abs(roundedScore), 1, 8) * STEPS_PER_SCORE_UNIT;
    moveClockwise = true;
  } else if (roundedScore < 0) {
    stepsToMove = constrain(abs(roundedScore), 1, 6) * STEPS_PER_SCORE_UNIT;
    moveClockwise = false;
  } // else stepsToMove remains 0

  if (stepsToMove > 0) {
    Serial.println("Executing motor movement based on score...");
    digitalWrite(ENA_PIN, LOW); // Ensure driver enabled
    delay(5);
    rotateSteps(stepsToMove, moveClockwise);
    // Optional: Disable driver
    // digitalWrite(ENA_PIN, HIGH);
  } else {
    Serial.println("Score is 0, no movement needed.");
  }
}

void loop() {
  if (Serial1.available() > 0) {
    String receivedLine = Serial1.readStringUntil('\n');
    receivedLine.trim();

    // Serial.print("HC12 RX: \""); Serial.print(receivedLine); Serial.println("\""); // Debug

    // --- MODIFIED: Check for the new result prefix ---
    if (receivedLine.startsWith(RESULT_PREFIX)) {
      Serial.println("  -> Matched AgResult prefix.");

      // Extract the part after "AgResult,"
      String dataString = receivedLine.substring(RESULT_PREFIX.length());

      // Parse the comma-separated values: SCORE, MAG_R, MAG_L
      // Find comma positions
      int firstComma = dataString.indexOf(',');
      int secondComma = dataString.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) { // Check if both commas exist
        // Extract substrings
        String scoreString = dataString.substring(0, firstComma);
        String magRString = dataString.substring(firstComma + 1, secondComma);
        String magLString = dataString.substring(secondComma + 1);

        // Convert to floats
        float receivedScore = scoreString.toFloat();
        float receivedMagR = magRString.toFloat();
        float receivedMagL = magLString.toFloat();
        float combined = receivedMagR + receivedMagL;
        Serial.print("  -> Parsed Score: "); Serial.println(receivedScore, 2);
        Serial.print("  -> Parsed Mag R: "); Serial.println(receivedMagR, 4);
        Serial.print("  -> Parsed Mag L: "); Serial.println(receivedMagL, 4);

        // --- Use ONLY the score for motor control for now ---
        moveMotorBasedOnScore(combined/20);
        // ----------------------------------------------------

        // You could add logic here later to use receivedMagR and receivedMagL

      } else {
        Serial.println("  -> Error parsing AgResult data (commas not found).");
      }
    } else {
       if (receivedLine.length() > 0) {
           // Serial.println("  -> Ignoring message (doesn't match prefix)."); // Reduce noise
       }
    }
  }
}