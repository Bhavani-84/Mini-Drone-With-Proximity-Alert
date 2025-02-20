#include <Wire.h>

// LiDAR sensor configuration
#define LIDAR_I2C_ADDR 0x62 // Default I2C address for LiDAR
#define LIDAR_DISTANCE_REG 0x8f

// Pin definitions
const int buzzerPin = 9; // Buzzer connected to pin 9
const int redLedPin = 10; // Red LED connected to pin 10

// Threshold distance in cm for triggering proximity alert
const int proximityThreshold = 50;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize I2C communication for LiDAR
  Wire.begin();
  
  // Set pin modes
  pinMode(buzzerPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  
  // Default state: LED and buzzer ON
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(redLedPin, HIGH);
}

void loop() {
  // Read distance from LiDAR sensor
  int distance = readLidarDistance();
  
  // Check if an obstacle is within the threshold
  if (distance > 0 && distance < proximityThreshold) {
    // Rapid blinking and buzzing
    for (int i = 0; i < 5; i++) { // Adjust iterations for desired blink duration
      digitalWrite(redLedPin, LOW); // Turn off LED momentarily
      digitalWrite(buzzerPin, LOW); // Turn off buzzer momentarily
      delay(100); // Short delay
      digitalWrite(redLedPin, HIGH); // Turn on LED
      digitalWrite(buzzerPin, HIGH); // Turn on buzzer
      delay(100); // Short delay
    }
    Serial.println("Obstacle detected! Distance: " + String(distance) + " cm");
  } else {
    // Default state: LED and buzzer stay on
    digitalWrite(redLedPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
  }
  
  // Small delay for stability
  delay(100);
}

// Function to read distance from LiDAR sensor
int readLidarDistance() {
  int distance = -1; // Default value for error
  
  Wire.beginTransmission(LIDAR_I2C_ADDR);
  Wire.write(LIDAR_DISTANCE_REG);
  Wire.endTransmission(false);
  
  Wire.requestFrom(LIDAR_I2C_ADDR, 2);
  if (Wire.available() >= 2) {
    distance = (Wire.read() << 8) | Wire.read();
  }
  
  return distance;
}