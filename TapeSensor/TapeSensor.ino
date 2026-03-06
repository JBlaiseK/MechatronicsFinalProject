#include <Stepper.h>

// 200 steps = 360 degrees
const int stepsPerRevolution = 200; 
const int steps90Degrees = 50; 

// Pins for Enable (Must be HIGH for motor to move)
const int PIN_ENA = 13; 

// Initialize the stepper on pins 2, 4, 3, 5
Stepper myStepper(stepsPerRevolution, 5, 4, 3, 2);

void setup() {
  // Set Enable pins as outputs and turn them HIGH
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, HIGH);

  // 10 RPM is slow enough for torque but fast enough to see
  myStepper.setSpeed(15); 
  
  Serial.begin(9600);
  // Serial.println("Starting 90-degree back-and-forth test...");
}

void loop() {
  // 1. Rotate 90 degrees Forward (Clockwise)
  // Serial.println("Moving Forward 90...");
  myStepper.step(steps90Degrees);
  delay(1000); // Pause for 1 second
  
  // 2. Rotate 90 degrees Backward (Counter-Clockwise)
  //Serial.println("Moving Backward 90...");
  //myStepper.step(-steps90Degrees);
  //delay(1000); // Pause for 1 second
}