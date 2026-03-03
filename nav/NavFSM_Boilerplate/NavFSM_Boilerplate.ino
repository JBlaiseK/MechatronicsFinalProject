/*
  NavFSM_Skeleton.ino — NAV Arduino (master)
  - Drives wheel motors
  - Reads IR + tape sensors
  - Stops at hog line, commands shooter to fire N pucks
  - Waits for shooter DONE
  - Returns to box for reload

*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ProtoLink.h"

#include <Servo.h>

// ---------------- Pin Map (MADE UP) ----------------
// Tape sensors (analog reflectance / IR line sensors)


/* ASSUMPTION -- I DO NOT KNOW HOW MANY TAPE SENSORS WE HAVE*/
static const uint8_t PIN_TAPE_L = A0;
static const uint8_t PIN_TAPE_R = A2;

// IR sensor analog input for beacon intensity
static const uint8_t PIN_IR_AMP = A3;

// User start / enable button (active-low)
static const uint8_t PIN_ENABLE_BTN = 2;

// Optional (Are we still doing the LED status? )
static const uint8_t PIN_STATUS_LED = 13;

// NAV TO SHOOTER link (SoftwareSerial -- library that sends messages to different arduinos
static const uint8_t PIN_LINK_RX = 10; // NAV receives on D10  (wired from SHOOTER TX)
static const uint8_t PIN_LINK_TX = 11; // NAV transmits on D11 (wired to SHOOTER RX)

static const uint8_t PIN_IR_SERVO = 9; 
static Servo irServo;

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

// ---- 909 Hz sampling params
static const int IR_FREQ = 909;
static const unsigned long IR_PERIOD_US = 1000000UL / IR_FREQ;   // ~1100 us
static const int IR_SAMPLES_PER_PERIOD = 8;
static const unsigned long IR_INTERVAL_US = IR_PERIOD_US / IR_SAMPLES_PER_PERIOD; // ~137 us


static const int IR_NUM_PERIODS = 5;
static const int IR_TOTAL_SAMPLES = IR_SAMPLES_PER_PERIOD * IR_NUM_PERIODS; // 40 samples



// ---- Servo scan params ----
static const int SERVO_MIN_DEG = 0;
static const int SERVO_MAX_DEG = 180;
static const int SERVO_FWD_DEG = 90;

// coarse scan step. 6–10 degrees -- subject to change!
static const int SERVO_STEP_DEG = 6;                // TODO calibrate
static const uint32_t SERVO_SETTLE_MS = 20;         // TODO calibrate

// store scan results at coarse step positions
static const int SCAN_POINTS = (SERVO_MAX_DEG - SERVO_MIN_DEG) / SERVO_STEP_DEG + 1;
static int irAmp[SCAN_POINTS];          // amplitude per scan index
static int irAngle[SCAN_POINTS];        // angle per scan index
static int scanIdx = 0;

// to prevent picking two peaks that are basically the same beacon lobe
static const int MIN_PEAK_SEPARATION_DEG = 25;      // TODO calibrate




// ---------------- CONSTANTS -- NEED REFINING I DO NOT KNOW WHAT THEY ARE  ----------------
static const int   TAPE_THRESHOLD = 600;   // TODO: calibrate

// static const int   IR_THRESHOLD   = 200;   // TODO: calibrate

static const int16_t SPEED_FWD    = 80;    // TODO: calibrate
static const int16_t SPEED_TURN   = 70;    // TODO: calibrate


// Turning calibration: how many milliseconds produces 1 degree of rotation
// TODO: need these values
static const float MS_PER_DEG_TURN = 8.0f;


// Tape-follow speed -- Basically modify the motors by this amount if they are too left or right of the tape
static const int16_t SPEED_DELTA = 35;

// Hog line detection behavior
static const uint32_t HOGLINE_DEBOUNCE_MS = 120;  //TODO: calibrate

// Volley config
static const uint8_t DEFAULT_SHOTS_PER_VOLLEY = 3;    // TODO: set from UI / DIP switch / Serial
static const uint16_t DEFAULT_DISTANCE_METRIC = 0;    // TODO: compute from IR/odometry/etc

// Reload waiting
static const uint32_t RELOAD_WAIT_MS = 12000;         // 10–15 seconds, TODO: calibrate

// ---------------- “Hardware Abstraction” ----------------
// Replace these with actual Raptor calls (or your own motor driver).
static void motorsStop() {
  // TODO:
}

static void motorsTank(int16_t left, int16_t right) {
  // TODO: 
}

// ---------------- Simple Timer ----------------
struct SimpleTimer {
  uint32_t t0 = 0;
  uint32_t dur = 0;
  bool running = false;

  void start(uint32_t durationMs) { t0 = millis(); dur = durationMs; running = true; }
  void stop() { running = false; }
  bool expired() const { return running && (uint32_t)(millis() - t0) >= dur; }
};

// ---------------- Tape helpers to access really easily -------- //
enum TapeBits : uint8_t {
  TAPE_NONE   = 0,
  TAPE_LEFT   = 1 << 0,
  TAPE_CENTER = 1 << 1,
  TAPE_RIGHT  = 1 << 2
};

static uint8_t readTapeBits() {
  // ASSUMPTION higher ADC means “darker tape” (common for reflectance sensors),

  int l = analogRead(PIN_TAPE_L);
  int r = analogRead(PIN_TAPE_R);

  uint8_t bits = TAPE_NONE;
  if (l > TAPE_THRESHOLD) bits |= TAPE_LEFT;
  if (r > TAPE_THRESHOLD) bits |= TAPE_RIGHT;
  return bits;
}

static bool isTapeCrossing(uint8_t bits) {
  // Assuming that "crossing" means that multiple sensors see tape at once.
  // TODO: refine 
  uint8_t count = ((bits & TAPE_LEFT) ? 1 : 0) + ((bits & TAPE_RIGHT) ? 1 : 0);
  return count == 2;
}

// ---------------- NAV FSM ----------------
typedef enum {
  NAV_IDLE,
  INIT_ORIENT,
  EXIT_START_ZONE,
  SEEK_CENTER_TAPE,
  ALIGN_TO_TAPE,
  FOLLOW_TO_HOGLINE,
  AT_HOGLINE_WAIT_SHOOT,
  TURN_AROUND,
  RETURN_TO_START_ZONE,
  IN_START_ZONE_RELOAD_WAIT,
  REORIENT_FOR_NEXT_RUN,
  NAV_DONE
} NavState;

static NavState state = NAV_IDLE;
static NavState next_state = NAV_IDLE;
static NavState prev_state = (NavState)255; // null value

// “Global” events / bookkeeping
static bool enableLatched = false;
static bool shooterBusy = false;
static uint16_t cmdSeq = 1;
static uint8_t shotsLeftTotal = 8;
static uint8_t lastVolleyFired = 0;

static SimpleTimer orientTimer;
static SimpleTimer exitTimer;
static SimpleTimer hogDebounceTimer;
static SimpleTimer turnTimer;
static SimpleTimer reloadTimer;

static char rxLine[Proto::LINE_MAX];



static int  readIrAmplitude909Hz();
static void handleInitOrient();
static void computeTwoPeaksAndBisector(int &p1Idx, int &p2Idx, int &bisDeg);



// Actual Scan of to populate the two arrays we need...
static int readIrAmplitude909Hz() {
  int min_val = 1023;
  int max_val = 0;

  unsigned long last_sample_time = micros();

  // Strict cadence sampling
  for (int i = 0; i < IR_TOTAL_SAMPLES; ) {
    if (micros() - last_sample_time >= IR_INTERVAL_US) {
      last_sample_time += IR_INTERVAL_US;

      int val = analogRead(PIN_IR_AMP);

      if (val < min_val) min_val = val;
      if (val > max_val) max_val = val;

      i++;
    }
  }
  return (max_val - min_val); // peak-to-peak amplitude
}


// Helper for the IR calculation

static void computeTwoPeaksAndBisector(int &p1Idx, int &p2Idx, int &bisDeg) {
  // Find best peak index (max amplitude)
  p1Idx = 0;
  for (int i = 1; i < SCAN_POINTS; i++) {
    if (irAmp[i] > irAmp[p1Idx]) p1Idx = i;
  }

  // Find second-best peak far enough away
  p2Idx = -1;
  for (int i = 0; i < SCAN_POINTS; i++) {
    int sep = abs(irAngle[i] - irAngle[p1Idx]);
    if (sep < MIN_PEAK_SEPARATION_DEG) continue;

    if (p2Idx < 0 || irAmp[i] > irAmp[p2Idx]) {
      p2Idx = i;
    }
  }

  // If no good second peak, fall back to just facing strongest peak
  if (p2Idx < 0) {
    bisDeg = irAngle[p1Idx];
    return;
  }

  // Bisector between the two peak angles
  bisDeg = (irAngle[p1Idx] + irAngle[p2Idx]) / 2;
}













// IR SENSOR FSM --- ripped from Jackson's IR SENSOR

static void handleInitOrient() {
  switch (orientPhase) {

    case ORIENT_SCAN_SETTLE:
      // wait for servo to settle before sampling
      if (servoSettleTimer.expired()) {
        orientPhase = ORIENT_SCAN_SAMPLE;
      }
      break;

    case ORIENT_SCAN_SAMPLE: {
      // sample IR amplitude at the current scan angle
      int amp = readIrAmplitude909Hz();
      irAmp[scanIdx] = amp;

      // advance scan index
      scanIdx++;

      if (scanIdx >= SCAN_POINTS) {
        // finished scanning all angles
        orientPhase = ORIENT_COMPUTE;
      } else {
        // move to next angle and settle
        int nextAng = irAngle[scanIdx];
        irServo.write(nextAng);
        servoSettleTimer.start(SERVO_SETTLE_MS);
        orientPhase = ORIENT_SCAN_SETTLE;
      }
    } break;

    case ORIENT_COMPUTE: {
      int p1Idx, p2Idx, bisDeg;
      computeTwoPeaksAndBisector(p1Idx, p2Idx, bisDeg);

      peak1Angle = irAngle[p1Idx];
      peak2Angle = (p2Idx < 0) ? peak1Angle : irAngle[p2Idx];
      bisectorAngle = bisDeg;

      // Heading error relative to robot forward direction.
      // Servo 90° means straight ahead.
      orientErrorDeg = bisectorAngle - SERVO_FWD_DEG;

      // Convert degrees to time for turning
      uint32_t turnMs = (uint32_t)(abs(orientErrorDeg) * MS_PER_DEG_TURN);

      // If already aligned, we're done
      if (turnMs < 30) {
        motorsStop();
        orientPhase = ORIENT_IDLE;
        return;
      }

      // Choose turn direction:
      // If bisector is to the right (>90), turn right to bring it to center.
      // IMPORTANT: You may need to swap these depending on your motor sign convention.
      if (orientErrorDeg > 0) {
        // turn right
        motorsTank(SPEED_TURN, -SPEED_TURN);
      } else {
        // turn left
        motorsTank(-SPEED_TURN, SPEED_TURN);
      }

      orientTurnTimer.start(turnMs);
      orientPhase = ORIENT_TURN;

      // Debug prints (optional)
      Serial.print("[ORIENT] p1="); Serial.print(peak1Angle);
      Serial.print(" p2="); Serial.print(peak2Angle);
      Serial.print(" bis="); Serial.print(bisectorAngle);
      Serial.print(" err="); Serial.print(orientErrorDeg);
      Serial.print(" turnMs="); Serial.println(turnMs);
    } break;

    case ORIENT_TURN:
      if (orientTurnTimer.expired()) {
        motorsStop();
        orientPhase = ORIENT_IDLE;
      }
      break;

    case ORIENT_IDLE:
    default:
      // do nothing
      break;
  }
}
















// ---------------- Event Tests ----------------
static bool TestEnablePressed() {
  // Active-low button
  return digitalRead(PIN_ENABLE_BTN) == LOW;
}

static bool TestOrientDone() {
  return (orientPhase == ORIENT_IDLE);
}

static bool TestExitDone() { return exitTimer.expired(); }

static bool TestHogLineReached() {
  // Basic approach: detect a “tape crossing” and then debounce for stability.
  uint8_t bits = readTapeBits();
  if (isTapeCrossing(bits)) {
    if (!hogDebounceTimer.running) hogDebounceTimer.start(HOGLINE_DEBOUNCE_MS);
    return hogDebounceTimer.expired();
  } else {
    hogDebounceTimer.stop();
    return false;
  }
}

static bool TestTurnDone() { return turnTimer.expired(); }
static bool TestReloadDone() { return reloadTimer.expired(); }


// Shooter link events
static void pollShooterLink() {
  if (!Proto::readLine(link, rxLine, sizeof(rxLine))) return;

  Proto::AckMsg ack;
  Proto::DoneMsg done;

  if (Proto::parseAck(rxLine, ack)) {
    // TODO: optionally verify seq matches expected
    Serial.print("[NAV] shooter ACK seq="); Serial.println(ack.seq);
    return;
  }

  if (Proto::parseDone(rxLine, done)) {
    Serial.print("[NAV] shooter DONE seq="); Serial.print(done.seq);
    Serial.print(" fired="); Serial.println(done.fired);

    shooterBusy = false;
    lastVolleyFired = done.fired;

    // TODO: update shotsLeftTotal based on fired count
    if (shotsLeftTotal >= done.fired) shotsLeftTotal -= done.fired;
    else shotsLeftTotal = 0;

    return;
  }

  // Unknown line:
  Serial.print("[NAV] ??? "); Serial.println(rxLine);
}


// ---------------- Responses (ONLY set next_state / flags) ----------------
static void RespToEnable() { enableLatched = true; next_state = INIT_ORIENT; }
static void RespOrientDone() { next_state = EXIT_START_ZONE; }
static void RespExitDone() { next_state = SEEK_CENTER_TAPE; }
static void RespFoundTape() { next_state = ALIGN_TO_TAPE; }
static void RespAligned() { next_state = FOLLOW_TO_HOGLINE; }
static void RespReachedHogLine() { next_state = AT_HOGLINE_WAIT_SHOOT; }
static void RespShooterDone() { next_state = TURN_AROUND; }
static void RespTurnDone() { next_state = RETURN_TO_START_ZONE; }
static void RespReturnedToBox() { next_state = IN_START_ZONE_RELOAD_WAIT; }
static void RespReloadDone() { next_state = REORIENT_FOR_NEXT_RUN; }
static void RespReorientDone() { next_state = (shotsLeftTotal > 0) ? EXIT_START_ZONE : NAV_DONE; }

// ---------------- State Handlers (DO actions on entry) ----------------
static void onEnter(NavState s) {
  switch (s) {
    case NAV_IDLE:
      digitalWrite(PIN_STATUS_LED, HIGH);
      motorsStop();
      break;

    case INIT_ORIENT:
      digitalWrite(PIN_STATUS_LED, HIGH);
      motorsStop();
      // init scan arrays
      scanIdx = 0;
      for (int i = 0; i < SCAN_POINTS; i++) {
        irAmp[i] = 0;
        irAngle[i] = SERVO_MIN_DEG + i * SERVO_STEP_DEG;
      }


      // begin scan at SERVO_MIN
      irServo.write(SERVO_MIN_DEG);
      servoSettleTimer.start(SERVO_SETTLE_MS);

      orientPhase = ORIENT_SCAN_SETTLE;
      break;

    case EXIT_START_ZONE:
      motorsTank(SPEED_FWD, SPEED_FWD);

      exitTimer.start(1200);            // TODO calibrate
      break;


    case SEEK_CENTER_TAPE:
      motorsTank(SPEED_TURN, -SPEED_TURN);
      break;


    case ALIGN_TO_TAPE:
      // handled in loop (non-blocking)
      break;

    case FOLLOW_TO_HOGLINE:
      // handled in loop
      break;

    case AT_HOGLINE_WAIT_SHOOT:
      motorsStop();
      if (!shooterBusy) {
        uint8_t volleyShots = min(DEFAULT_SHOTS_PER_VOLLEY, shotsLeftTotal);
        Proto::sendFire(link, cmdSeq, volleyShots, DEFAULT_DISTANCE_METRIC);
        Serial.print("[NAV] sent FIRE seq="); Serial.print(cmdSeq);
        Serial.print(" shots="); Serial.println(volleyShots);
        shooterBusy = true;
        cmdSeq++;
      }
      break;

    case TURN_AROUND: {
      uint32_t ms = (uint32_t)(180.0f * MS_PER_DEG_TURN);
      motorsTank(SPEED_TURN, -SPEED_TURN);
      turnTimer.start(ms);
    } break;

    case RETURN_TO_START_ZONE:
      // handled in loop
      break;

    case IN_START_ZONE_RELOAD_WAIT:
      motorsStop();
      reloadTimer.start(RELOAD_WAIT_MS);
      break;

    case REORIENT_FOR_NEXT_RUN: {
      uint32_t ms = (uint32_t)(180.0f * MS_PER_DEG_TURN);
      motorsTank(SPEED_TURN, -SPEED_TURN);
      turnTimer.start(ms);
    } break;

    case NAV_DONE:
      motorsStop();
      digitalWrite(PIN_STATUS_LED, LOW);
      break;
  }
}

static void handleSeekCenterTape() {
  uint8_t bits = readTapeBits();
  if (bits != TAPE_NONE) RespFoundTape();
}

static void handleAlignToTape() {
  uint8_t bits = readTapeBits();
  if (bits & TAPE_CENTER) {
    motorsStop();
    RespAligned();
    return;
  }
  if (bits & TAPE_LEFT) motorsTank(-SPEED_TURN, SPEED_TURN);
  else if (bits & TAPE_RIGHT) motorsTank(SPEED_TURN, -SPEED_TURN);
  else {
    motorsTank(SPEED_TURN, -SPEED_TURN);
    next_state = SEEK_CENTER_TAPE;
  }
}

static void handleFollowToHogline(bool headingOut) {
  uint8_t bits = readTapeBits();

  int16_t left = SPEED_FWD;
  int16_t right = SPEED_FWD;

  if (bits & TAPE_CENTER) {
    // straight
  } else if (bits & TAPE_LEFT) {
    left -= SPEED_DELTA; right += SPEED_DELTA;
  } else if (bits & TAPE_RIGHT) {
    left += SPEED_DELTA; right -= SPEED_DELTA;
  } else {

    // TODO: LINE RECOVERY -- NOT REALLY COVERED
    left = 0; right = 0;
  }

  motorsTank(left, right);

  if (TestHogLineReached()) {
    // TODO: distinguish hog line vs start-zone boundary
    if (headingOut) RespReachedHogLine();
    else RespReturnedToBox();
  }
}





// ---------------- Global Event Checking ----------------
static void checkGlobalEvents() {
  next_state = state;
  pollShooterLink();

  if (!enableLatched && state == NAV_IDLE && TestEnablePressed()) RespToEnable();

  if (state == INIT_ORIENT) {
    handleInitOrient(); 
    if (TestOrientDone()) RespOrientDone();

  } else if (state == EXIT_START_ZONE && TestExitDone()) {
    RespExitDone();

  } else if (state == SEEK_CENTER_TAPE) {
    handleSeekCenterTape();

  } else if (state == ALIGN_TO_TAPE) {
    handleAlignToTape();

  } else if (state == FOLLOW_TO_HOGLINE) {
    handleFollowToHogline(true);

  } else if (state == AT_HOGLINE_WAIT_SHOOT) {
    if (!shooterBusy) RespShooterDone();

  } else if (state == TURN_AROUND && TestTurnDone()) {
    motorsStop();
    RespTurnDone();

  } else if (state == RETURN_TO_START_ZONE) {
    handleFollowToHogline(false);

  } else if (state == IN_START_ZONE_RELOAD_WAIT && TestReloadDone()) {
    RespReloadDone();

  } else if (state == REORIENT_FOR_NEXT_RUN && TestTurnDone()) {
    motorsStop();
    RespReorientDone();
  }

  if (next_state != state) {
    prev_state = state;
    state = next_state;
    onEnter(state);
  }
}






// ---------------- Arduino setup/loop ----------------
void setup() {
  pinMode(PIN_ENABLE_BTN, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);

  Serial.begin(115200);
  link.begin(19200);

  Serial.println("[NAV] boot");

  bitClear(ADCSRA, ADPS0);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);

  irServo.attach(PIN_IR_SERVO);
  irServo.write(SERVO_FWD_DEG);

  
  // MOTORS OFF
  motorsStop();

  state = NAV_IDLE;
  next_state = NAV_IDLE;
  prev_state = (NavState)255;
  onEnter(state);
}



void loop() {
  checkGlobalEvents();
  // TODO: periodic debug prints (tape bits, state) at low rate
}
