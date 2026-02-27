#include <Servo.h>

const int IR_PIN = A0;
const int SERVO_PIN = 9;

Servo scannerServo;

// Arrays for raw and processed data
int rawAmplitudes[181]; 
int smoothedAmplitudes[181];

// Troubleshooting toggle
// Set to 'true' to output data formatted for the Arduino Serial Plotter
const bool DEBUG_PLOTTER = false; 

void setup() {
  Serial.begin(115200);
  scannerServo.attach(SERVO_PIN);
  
  // Speed up ADC for faster continuous sampling
  bitClear(ADCSRA, ADPS0);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
  
  scannerServo.write(0);
  delay(1000);
}

void loop() {
  if (!DEBUG_PLOTTER) Serial.println("\n--- Starting Sweep ---");

  // 1. SWEEP AND RECORD (Using Window Sampling)
  for (int angle = 0; angle <= 180; angle++) {
    scannerServo.write(angle);
    delay(15); // Let servo settle
    
    rawAmplitudes[angle] = getIrAmplitude();
  }

  // 2. SMOOTH THE DATA (3-Point Moving Average)
  // This removes 1-degree noise spikes to find the true center of the beacon
  for (int i = 0; i <= 180; i++) {
    if (i == 0 || i == 180) {
      smoothedAmplitudes[i] = rawAmplitudes[i]; // Edges stay same
    } else {
      smoothedAmplitudes[i] = (rawAmplitudes[i-1] + rawAmplitudes[i] + rawAmplitudes[i+1]) / 3;
    }
  }

  // 3. FIND PEAK 1
  int maxAmp1 = 0;
  int angle1 = -1;
  for (int i = 0; i <= 180; i++) {
    if (smoothedAmplitudes[i] > maxAmp1) {
      maxAmp1 = smoothedAmplitudes[i];
      angle1 = i;
    }
  }

  // 4. FIND PEAK 2 (Must be >= 5 degrees away)
  int maxAmp2 = 0;
  int angle2 = -1;
  for (int i = 0; i <= 180; i++) {
    if (abs(i - angle1) >= 5) {
      if (smoothedAmplitudes[i] > maxAmp2) {
        maxAmp2 = smoothedAmplitudes[i];
        angle2 = i;
      }
    }
  }

  // 5. OUTPUT RESULTS
  if (DEBUG_PLOTTER) {
    // Print the entire array so the Serial Plotter can graph it
    for (int i = 0; i <= 180; i++) {
      Serial.print(rawAmplitudes[i]); // Blue line: Raw data
      Serial.print(",");
      Serial.println(smoothedAmplitudes[i]); // Red line: Smoothed data
    }
    // Add a visual gap in the plotter between sweeps
    for(int i=0; i<5; i++) { Serial.println("0,0"); } 
  } else {
    // Standard text output
    Serial.print("Target 1 -> Angle: "); Serial.print(angle1);
    Serial.print(" (Amplitude: "); Serial.print(maxAmp1); Serial.println(")");
    
    Serial.print("Target 2 -> Angle: "); Serial.print(angle2);
    Serial.print(" (Amplitude: "); Serial.print(maxAmp2); Serial.println(")");
  }

  delay(3000); 
  scannerServo.write(0);
  delay(1000);
}

// --- NEW Window-Based Amplitude Function ---
int getIrAmplitude() {
  int min_val = 1023;
  int max_val = 0;
  
  unsigned long start_time = millis();
  
  // Sample continuously for 5 milliseconds.
  // 909 Hz = ~1.1ms period. 5ms ensures we see ~4.5 full wave cycles 
  // regardless of when the servo interrupt fires.
  while (millis() - start_time < 5) {
    int val = analogRead(IR_PIN);
    if (val < min_val) min_val = val;
    if (val > max_val) max_val = val;
  }
  
  return (max_val - min_val);
}