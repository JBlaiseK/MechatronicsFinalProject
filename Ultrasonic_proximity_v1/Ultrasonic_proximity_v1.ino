// Define the sensor pins
const int trigPin = 11;
const int echoPin = 12; 

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600); 
  
  // Set the pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // 1. Ensure the trigger pin is clear/LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 2. Send a 10-microsecond HIGH pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. Read the echo pin (returns the sound wave travel time in microseconds)
  long duration = pulseIn(echoPin, HIGH);

  // 4. Calculate the distance in centimeters
  // Speed of sound is approx. 0.0343 cm/microsecond
  // Divide by 2 to account for the sound wave traveling out and back
  float distance_cm = duration * 0.0343 / 2.0;

  // 5. Print the results to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance_cm, 1); 
  Serial.println(" cm");

  // Wait 60 milliseconds before taking the next reading to prevent echo overlaps
  delay(60); 
}