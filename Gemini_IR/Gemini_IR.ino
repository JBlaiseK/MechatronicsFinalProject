const int IR_PIN = A0;

// Signal Parameters
const int FREQ = 909;
const unsigned long PERIOD_US = 1000000 / FREQ;         // ~1100 microseconds
const int SAMPLES_PER_PERIOD = 8;                       // 4 reads high, 4 reads low
const unsigned long INTERVAL_US = PERIOD_US / SAMPLES_PER_PERIOD; // ~137 microsecond cadence

// Window Parameters
const int NUM_PERIODS = 5;                              // Sample over 5 full waves
const int TOTAL_SAMPLES = SAMPLES_PER_PERIOD * NUM_PERIODS; 

void setup() {
  Serial.begin(115200);

  // SPEED UP THE ADC
  // Change the ADC prescaler from 128 to 32.
  // This reduces analogRead() time from ~104us to ~26us without losing precision.
  bitClear(ADCSRA, ADPS0);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
  
  Serial.println("909 Hz IR Beacon Scanner Initialized");
}

void loop() {
  int min_val = 1023;
  int max_val = 0;
  long sum = 0;

  unsigned long last_sample_time = micros();

  // Rapid sampling loop
  for (int i = 0; i < TOTAL_SAMPLES; ) {
    // Non-blocking precise timing
    if (micros() - last_sample_time >= INTERVAL_US) {
      last_sample_time += INTERVAL_US; // Maintain strict cadence
      
      int val = analogRead(IR_PIN);
      
      // Track min and max to find the square wave amplitude
      if (val < min_val) min_val = val;
      if (val > max_val) max_val = val;
      
      sum += val;
      i++;
    }
  }

  // Calculate results
  int amplitude = max_val - min_val;
  int mean = sum / TOTAL_SAMPLES;

  // Print results
  // If the beacon is detected, 'amplitude' will be high. 
  // If no beacon, 'amplitude' will be near 0 (just noise) and mean should be ~512.
  Serial.print("Mean: ");
  Serial.print(mean);
  Serial.print("\t Amplitude (Peak-to-Peak): ");
  Serial.println(amplitude);

  delay(50); // Brief pause before the next scan window
}