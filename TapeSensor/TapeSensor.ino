// Define the pin where the phototransistor (white wire) is connected
const int sensorPin = A2; 
const int sen
int sensorValue = 0; // Variable to store the raw reading

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  Serial.println("Tape Sensor Calibration Started!");
  Serial.println("--------------------------------");
}

void loop() {
  // Read the input from the sensor
  sensorValue = analogRead(sensorPin);

  
  // Print the value to the Serial Monitor
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
  
  // Wait 100 milliseconds before the next reading so it's easy to read
  delay(100); 
}