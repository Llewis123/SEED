#include <Arduino_BMI270_BMM150.h>
#include <math.h> // Include for sqrt()

// --- IDENTIFY THIS ARDUINO ---
// UNCOMMENT ONE of the following lines:
//#define IS_RIGHT_LEG
#define IS_LEFT_LEG
// -----------------------------

// Constants
#define G 9.80665

// --- TDM Configuration ---
const unsigned long CYCLE_LENGTH_MS = 2000;
const unsigned long SLOT_DURATION_MS = CYCLE_LENGTH_MS / 2;
const unsigned long TRANSMIT_INTERVAL_MS = 1000; // Send at most once per second within slot

// Variables to hold sensor data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ; // Add magnetometer variables
float currentGyroMag = 0.0; // Store instantaneous gyro magnitude

// Timing variables
unsigned long lastTransmitMillis = 0;

// HC-12 uses Hardware Serial1 (Pins D0, D1)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  #ifdef IS_RIGHT_LEG
    Serial.println("RIGHT LEG Arduino Setup (ML+Mag)...");
  #elif defined(IS_LEFT_LEG)
    Serial.println("LEFT LEG Arduino Setup (ML+Mag)...");
  #else
    Serial.println("WARNING: Leg side not defined!"); while(1);
  #endif

  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!"); while (1);
  }
  // Optional: Set magnetometer rate if needed (check library examples)
  // IMU.setMagneticFieldOutputDataRate(100); // Example: 100 Hz
  Serial.println("IMU Initialized.");

  Serial1.begin(9600);
  Serial.println("HC-12 Serial1 Initialized at 9600 baud.");
  Serial.println("Setup complete.");
  lastTransmitMillis = millis() - TRANSMIT_INTERVAL_MS;
}

void loop() {
  unsigned long currentMillis = millis();
  bool dataRead = false;

  // --- Read Sensor Data (Including Magnetometer) ---
  float libAccX, libAccY, libAccZ;
  float libGyroX, libGyroY, libGyroZ;
  float libMagX, libMagY, libMagZ;

  // Prioritize reading Accel/Gyro as they update faster usually
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(libAccX, libAccY, libAccZ);
    accX = libAccX * G; accY = libAccY * G; accZ = libAccZ * G;
    dataRead = true; // Mark data as potentially read
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(libGyroX, libGyroY, libGyroZ);
    gyroX = libGyroX; gyroY = libGyroY; gyroZ = libGyroZ;
    // Calculate instantaneous magnitude when new gyro data arrives
    currentGyroMag = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
    dataRead = true;
  }
   // Read Magnetometer data - might update less frequently
   if (IMU.magneticFieldAvailable()) {
     IMU.readMagneticField(libMagX, libMagY, libMagZ);
     magX = libMagX; magY = libMagY; magZ = libMagZ;
     // Don't strictly need to set dataRead=true here unless it's the only new data
     // dataRead = true; // Uncomment if magnetometer is primary trigger
   }
  // --- End Sensor Reading ---


  // --- TDM Transmission Logic ---
  if (dataRead && (currentMillis - lastTransmitMillis >= TRANSMIT_INTERVAL_MS)) {
    unsigned long time_in_cycle = currentMillis % CYCLE_LENGTH_MS;
    bool is_my_timeslot = false;
    #ifdef IS_RIGHT_LEG
      if (time_in_cycle < SLOT_DURATION_MS) { is_my_timeslot = true; }
    #elif defined(IS_LEFT_LEG)
      if (time_in_cycle >= SLOT_DURATION_MS) { is_my_timeslot = true; }
    #endif

    if (is_my_timeslot) {
      transmitSensorData(); // Send data including magnitude
      lastTransmitMillis = currentMillis;
      // Serial.println("--> Transmitted Data in Time Slot"); // Debug
    }
  }
  // --- End TDM Logic ---

  delay(5);
}

// Function to format and transmit sensor data via HC-12 using Serial1
// NOW INCLUDES MAGNETOMETER AND GYRO MAGNITUDE (11 parts total)
void transmitSensorData() {
  String dataPacket = "";

  #ifdef IS_RIGHT_LEG
    dataPacket += "R";
  #elif defined(IS_LEFT_LEG)
    dataPacket += "L";
  #else
    dataPacket += "?";
  #endif

  // Add 9 sensor features + 1 magnitude value
  dataPacket += ","; dataPacket += String(gyroX, 3); // 1
  dataPacket += ","; dataPacket += String(gyroY, 3); // 2
  dataPacket += ","; dataPacket += String(gyroZ, 3); // 3
  dataPacket += ","; dataPacket += String(accX, 3);  // 4
  dataPacket += ","; dataPacket += String(accY, 3);  // 5
  dataPacket += ","; dataPacket += String(accZ, 3);  // 6
  dataPacket += ","; dataPacket += String(magX, 2);  // 7 - Mag precision lower usually
  dataPacket += ","; dataPacket += String(magY, 2);  // 8
  dataPacket += ","; dataPacket += String(magZ, 2);  // 9
  dataPacket += ","; dataPacket += String(currentGyroMag, 3); // 10 - Inst. Gyro Magnitude

  // Send data packet via Serial1 with delimiters
  Serial1.print("$"); // Start marker
  Serial1.print(dataPacket);
  Serial1.println("~"); // End marker (includes newline)
  // Serial1.flush(); // Optional

  // Print transmitted data to main Serial Monitor for debugging
  Serial.print("Serial1 TX -> $"); Serial.print(dataPacket); Serial.println("~"); // Reduce debug noise
}