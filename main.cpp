/*************************************************************
 * Autonomous Sumo Robot with ESP32-S3, TB6612FNG, TCRT5000, 
 * and 3x VL53L0X sensors.
 *************************************************************/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ------------------------- Pin Definitions -------------------------
#define STBY_PIN     7

// Motor A pins
#define AIN1_PIN     6
#define AIN2_PIN     5
#define PWMA_PIN     4   

// Motor B pins
#define BIN1_PIN     15 
#define BIN2_PIN     16
#define PWMB_PIN     17  

// TCRT5000 line sensors
#define LINE_SENSOR_FL 1  // Front-left corner
#define LINE_SENSOR_FR 41  // Front-right corner
#define OPP_SENSOR_BL 38 // Back-left corner
#define OPP_SENSOR_BR 45  // Back-right corner

// I2C pins
#define I2C_SCL_PIN 9
#define I2C_SDA_PIN 8

// XSHUT pins for each VL53L0X sensor to give unique addresses
#define XSHUT_LEFT   46
#define XSHUT_CENTER 3
#define XSHUT_RIGHT  18

// The new addresses we want to assign each sensor
#define LEFT_ADDRESS   0x30
#define CENTER_ADDRESS 0x31
#define RIGHT_ADDRESS  0x32

// ---------------------- Global Objects -----------------------------
Adafruit_VL53L0X loxLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X loxCenter= Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// For storing distance measurements
uint16_t distLeft   = 8190; // Default max
uint16_t distCenter = 8190;
uint16_t distRight  = 8190;

// ------------------------ Setup Sensors -----------------------------
void setupSensors() {
  // 1) Set all XSHUT pins LOW to shut down all sensors
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // 2) Initialize Left sensor
  pinMode(XSHUT_LEFT, INPUT); 
  delay(10);
  if (!loxLeft.begin(0x29)) { 
    Serial.println("Failed to find VL53L0X (Left) at default address!");
    while (1);
  }
  loxLeft.setAddress(LEFT_ADDRESS);

  // 3) Initialize Center sensor
  digitalWrite(XSHUT_CENTER, HIGH); 
  delay(10);
  if (!loxCenter.begin(0x29)) {
    Serial.println("Failed to find VL53L0X (Center) at default address!");
    while (1);
  }
  loxCenter.setAddress(CENTER_ADDRESS);

  // 4) Initialize Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH); 
  delay(10);
  if (!loxRight.begin(0x29)) {
    Serial.println("Failed to find VL53L0X (Right) at default address!");
    while (1);
  }
  loxRight.setAddress(RIGHT_ADDRESS);

  Serial.println("VL53L0X sensors initialized with unique I2C addresses.");
}

// ----------------------- Motor Control ------------------------------
void setupMotorDriver() {
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  // Disable standby
  digitalWrite(STBY_PIN, HIGH);
}

void driveMotorA(int speed) {
  // speed: -255 to 255
  if (speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, speed);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, -speed);
  }
}

void driveMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, speed);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, -speed);
  }
}

void stopMotors() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
}

void detection (bool opponentLeft, bool opponentCenter, bool opponentRight, bool opponentSideLeft, bool opponentSideRight){
    if (opponentCenter) {
      driveMotorA(250);
      driveMotorB(-250);
      delay(50);
      stopMotors();
    } else if (opponentLeft < opponentRight) {
      driveMotorA(-50);
      driveMotorB(-250);
      delay(50);
      driveMotorA(250);
      driveMotorB(-250);
      delay(25);
      stopMotors();
    } else if (opponentRight < opponentLeft) {
      driveMotorA(250);
      driveMotorB(50);
      delay(50);
      driveMotorA(250);
      driveMotorB(-250);
      delay(25);
      stopMotors();
    } else if (opponentLeft == opponentRight) {
      opponentCenter = true;
    } else if (opponentSideRight && !opponentSideLeft) {
      driveMotorA(250);
      driveMotorB(50);
      delay(30);
      driveMotorA(250);
      driveMotorB(-250);
      delay(25);
      stopMotors();
    } else if (opponentSideLeft && !opponentSideRight) {
      driveMotorA(-50);
      driveMotorB(-250);
      delay(30);
      driveMotorA(250);
      driveMotorB(-250);
      delay(25);
      stopMotors();
    }
}

void checkLineSensors() {
  bool lineFL = digitalRead(LINE_SENSOR_FL); 
  bool lineFR = digitalRead(LINE_SENSOR_FR);

  if (lineFL && lineFR) {
    driveMotorA(-250);
    driveMotorB(250);
    delay(100);
    stopMotors();
  } else if (lineFL && !lineFR) {
    driveMotorA(-250);
    driveMotorB(250);
    delay(100);
    driveMotorA(200);
    driveMotorB(100);
  } else if (!lineFL && lineFR) {
    driveMotorA(-250);
    driveMotorB(250);
    delay(100);
    driveMotorA(-200);
    driveMotorB(-100);
  }
}

// ----------------------- Setup & Loop -------------------------------
void setup() {
  Serial.begin(115250);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Line sensors
  pinMode(LINE_SENSOR_FL, INPUT_PULLUP);
  pinMode(LINE_SENSOR_FR, INPUT_PULLUP);
  pinMode(OPP_SENSOR_BL, INPUT_PULLUP);
  pinMode(OPP_SENSOR_BR, INPUT_PULLUP);

  // Initialize sensors
  setupSensors();

  // Initialize motor driver
  setupMotorDriver();

  Serial.println("Setup complete.");
}

uint16_t readDistance(Adafruit_VL53L0X &sensor, uint8_t address) {
  VL53L0X_RangingMeasurementData_t measure;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus == 4) {
    return 8190; // out of range
  } else {
    return measure.RangeMilliMeter;
  }
}

uint16_t getAverageDistance(Adafruit_VL53L0X &sensor, uint8_t address, int samples = 5) {
  uint32_t total = 0;
  for (int i = 0; i < samples; i++) {
    total += readDistance(sensor, address);
    delay(10); // Small delay between readings
  }
  return total / samples;
}

void loop() {
  // Read line sensors
  bool lineFL = digitalRead(LINE_SENSOR_FL);  // HIGH or LOW
  bool lineFR = digitalRead(LINE_SENSOR_FR);

  // Check line sensors at the beginning of the loop
  checkLineSensors();

  // Read VL53L0X sensors with averaging
  distLeft = getAverageDistance(loxLeft, LEFT_ADDRESS);
  distCenter = getAverageDistance(loxCenter, CENTER_ADDRESS);
  distRight = getAverageDistance(loxRight, RIGHT_ADDRESS);

  // Determine where the opponent is (if anywhere close)
  const uint16_t detectionThreshold = 120; // millimeters

  bool opponentLeft   = (distLeft   < detectionThreshold);
  bool opponentCenter = (distCenter < detectionThreshold);
  bool opponentRight  = (distRight  < detectionThreshold);
  bool opponentSideLeft = !digitalRead(OPP_SENSOR_BL); 
  bool opponentSideRight = !digitalRead(OPP_SENSOR_BR); 

  // If both front line sensors detect white (no line), execute the specified movement
  if (!lineFL && !lineFR) {
    driveMotorA(250);
    driveMotorB(-100);
    delay(50);
    stopMotors();
    delay(50);

    // First detection
    detection(opponentLeft, opponentCenter, opponentRight, opponentSideLeft, opponentSideRight);
    delay(50);

    driveMotorA(100);
    driveMotorB(-250);
    delay(50);
    stopMotors();
    delay(50);

    // Second detection
    detection(opponentLeft, opponentCenter, opponentRight, opponentSideLeft, opponentSideRight);
    delay(50);

    return;
  }

  // Check line sensors again before continuing
  // checkLineSensors();

  delay(50);
}
