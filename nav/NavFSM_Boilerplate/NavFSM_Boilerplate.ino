#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ProtoLink.h"
#include <Servo.h>


// ========================= Timers ============================
struct SimpleTimer {
  uint32_t t0 = 0;
  uint32_t dur = 0;
  bool running = false;

  void start(uint32_t durationMs) { t0 = millis(); dur = durationMs; running = true; }
  void stop() { running = false; }
  bool expired() const { return running && (uint32_t)(millis() - t0) >= dur; }
};



// ===================== STAGE SELECT HERE ====================

// CHANGE THIS ONE LINE:
static const uint8_t STAGE_MODE = 11;     // 0 = FULL_FSM, 1..15 = tests (see list below)

// Safety gate: robot will not move unless true.
static const bool TEST_ENABLE_MOTORS = true;

/*
  STAGE MODES:
    0  = FULL_FSM
    1  = TAPE_MONITOR_RAW
    2  = ULTRASONIC_MONITOR
    3  = IR_AMPLITUDE_MONITOR_909HZ      (no servo sweep)
    4  = SERVO_SWEEP_ONLY               (servo moves, no IR)
    5  = IR_SCAN_NO_TURN                (scan + print peaks, no wheel turn)
    6  = IR_ORIENT_ONLY                 (scan + wheel turn)
    7  = EXIT_BOX_UNTIL_CROSS
    8  = LINE_FOLLOW_UNTIL_HOG


    9  = TURN_90_TIMED
    10 = TURN_180_TIMED
    11 = MOTORS_CONSTANT_FORWARD

    12 = MOTORS_TOGGLE_DIRECTION_DEMO
    13 = SHOOTER_LINK_MONITOR
    14 = SHOOTER_FIRE_ONCE
    15 = DRIVE_INTO_BOX_UNTIL_US_NEAR   (straight drive + ultrasonic stop)
*/


// ========================= Pin Map ===========================
// IR sensor analog input (amplitude after op-amp)
static const uint8_t PIN_IR_AMP = A0;

// Tape sensors (LEFT + RIGHT only)
static const uint8_t PIN_TAPE_L = A1;
static const uint8_t PIN_TAPE_R = A2;


// Servo that sweeps IR sensor
static const uint8_t PIN_IR_SERVO = 9;
static Servo irServo;

// Enable button (active-low)
static const uint8_t PIN_ENABLE_BTN = 2;



// MOTORS
static const uint8_t PIN_L_ENA = 9;  // PWM
static const uint8_t PIN_L_IN1 = 8;
static const uint8_t PIN_L_IN2 = 7;

 
static const uint8_t PIN_R_ENA = 10;  // PWM
static const uint8_t PIN_R_IN1 = 11; 
static const uint8_t PIN_R_IN2 = 12;


// // Status LED
static const uint8_t PIN_STATUS_LED = 13;

// NAV <-> SHOOTER link (SoftwareSerial)
static const uint8_t PIN_LINK_RX = 10; // NAV receives on D10
static const uint8_t PIN_LINK_TX = 11; // NAV transmits on D11

SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);


// Ultrasonic (HC-SR04 style)
static const uint8_t PIN_US_TRIG = 10;
static const uint8_t PIN_US_ECHO = 11;

// ======================== Constants ==========================




// -------- Tape + motion --------
static const int   TAPE_THRESHOLD = 600;    // TODO calibrate using STAGE 1
static const int16_t SPEED_FWD    = 80;     // TODO calibrate
static const int16_t SPEED_TURN   = 70;     // TODO calibrate
static const int16_t SPEED_SEARCH = 55;     // recovery turn when tape lost
static const int16_t SPEED_DELTA  = 35;     // tape-follow steering delta

// Robot turn calibration ONLY (wheel motors)
static const float MS_PER_DEG_ROBOT = 8.0f; // TODO calibrate using STAGE 9/10

// Tape crossing debounce
static const uint32_t EXIT_CROSS_DEBOUNCE_MS = 120;
static const uint32_t HOGLINE_DEBOUNCE_MS    = 120;

// Exit box fallback timeout
static const uint32_t EXIT_TIMEOUT_MS = 2500;

// -------- Shooter config --------
static const uint8_t  DEFAULT_SHOTS_PER_VOLLEY = 3;
static const uint16_t DEFAULT_DISTANCE_METRIC  = 0;

// -------- Reload wait (FULL FSM only) --------
static const uint32_t RELOAD_WAIT_MS = 12000;

// -------- Ultrasonic --------
static const uint32_t US_PING_INTERVAL_MS = 60;
static const uint32_t US_ECHO_TIMEOUT_US  = 25000;
static const float    US_CM_PER_US        = 0.0343f / 2.0f;

// Hysteresis / debounce
static const float    US_NEAR_WALL_CM  = 12.0f;
static const float    US_CLEAR_WALL_CM = 25.0f;
static const uint8_t  US_NEAR_N        = 3;
static const uint8_t  US_CLEAR_N       = 3;

// -------- IR 909Hz sampling (your method) --------
static const int IR_FREQ = 909;
static const unsigned long IR_PERIOD_US = 1000000UL / IR_FREQ; // ~1100 us
static const int IR_SAMPLES_PER_PERIOD = 8;
static const unsigned long IR_INTERVAL_US = IR_PERIOD_US / IR_SAMPLES_PER_PERIOD; // ~137 us
static const int IR_NUM_PERIODS = 5;
static const int IR_TOTAL_SAMPLES = IR_SAMPLES_PER_PERIOD * IR_NUM_PERIODS; // 40 samples

// -------- Servo scan params --------
static const int SERVO_MIN_DEG = 0;
static const int SERVO_MAX_DEG = 180;
static const int SERVO_FWD_DEG = 90;

// Coarse scan resolution and settle time
static const int SERVO_STEP_DEG = 6;          // TODO tune
static const uint32_t SERVO_SETTLE_MS = 20;   // TODO tune

static const int SCAN_POINTS = (SERVO_MAX_DEG - SERVO_MIN_DEG) / SERVO_STEP_DEG + 1;
static int irAmp[SCAN_POINTS];
static int irAngle[SCAN_POINTS];
static int scanIdx = 0;


// Keep peaks separated
static const int MIN_PEAK_SEPARATION_DEG = 25; // TODO tune


// =============== Hardware abstraction (TODO) =================



// ===================== Motor Driver (ENA/IN1/IN2) =====================

static inline uint8_t clampPwm(int16_t x) {
  if (x < 0) x = -x;
  if (x > 255) x = 255;
  return (uint8_t)x;
}

static void setMotorDirection_L(bool forward) {
  if (forward) { digitalWrite(PIN_L_IN1, HIGH); digitalWrite(PIN_L_IN2, LOW); }
  else         { digitalWrite(PIN_L_IN1, LOW);  digitalWrite(PIN_L_IN2, HIGH); }
}

static void setMotorDirection_R(bool forward) {
  if (forward) { digitalWrite(PIN_R_IN1, HIGH); digitalWrite(PIN_R_IN2, LOW); }
  else         { digitalWrite(PIN_R_IN1, LOW);  digitalWrite(PIN_R_IN2, HIGH); }
}

static void motorWrite_L(int16_t cmd) {
  if (cmd == 0) { analogWrite(PIN_L_ENA, 0); return; }
  setMotorDirection_L(cmd > 0);
  analogWrite(PIN_L_ENA, clampPwm(cmd));
}

static void motorWrite_R(int16_t cmd) {
  if (cmd == 0) { analogWrite(PIN_R_ENA, 0); return; }
  setMotorDirection_R(cmd > 0);
  analogWrite(PIN_R_ENA, clampPwm(cmd));
}

static void motorsStop() {
  analogWrite(PIN_L_ENA, 0);
  analogWrite(PIN_R_ENA, 0);
}

static void motorsTank(int16_t leftCmd, int16_t rightCmd) {
  motorWrite_L(leftCmd);
  motorWrite_R(rightCmd);
}



static void safeMotorsTank(int16_t left, int16_t right) {
  if (TEST_ENABLE_MOTORS) motorsTank(left, right);
  else motorsStop();
}

static void safeMotorsStop() {
  motorsStop();
}




// ========================== Tape =============================

enum TapeBits : uint8_t {
  TAPE_NONE  = 0,
  TAPE_LEFT  = 1 << 0,
  TAPE_RIGHT = 1 << 1
};

static uint8_t readTapeBits() {
  int l = analogRead(PIN_TAPE_L);
  int r = analogRead(PIN_TAPE_R);

  uint8_t bits = TAPE_NONE;
  if (l > TAPE_THRESHOLD) bits |= TAPE_LEFT;
  if (r > TAPE_THRESHOLD) bits |= TAPE_RIGHT;
  return bits;
}

static bool isTapeCrossing(uint8_t bits) {
  return (bits & TAPE_LEFT) && (bits & TAPE_RIGHT);
}

static bool crossingDebounced(SimpleTimer &deb, uint32_t debounceMs) {
  uint8_t bits = readTapeBits();
  if (isTapeCrossing(bits)) {
    if (!deb.running) deb.start(debounceMs);
    return deb.expired();
  } else {
    deb.stop();
    return false;
  }
}



// Last seen tape side for recovery when both sensors go off.
// -1 = last saw LEFT, +1 = last saw RIGHT, 0 = both/unknown
static int8_t lastTapeSide = 0;

static void updateTapeMemory(uint8_t bits) {
  bool L = bits & TAPE_LEFT;
  bool R = bits & TAPE_RIGHT;

  if (L && !R) lastTapeSide = -1;
  else if (!L && R) lastTapeSide = +1;
  else if (L && R) lastTapeSide = 0;
}



// Line follow: if none, recovery turn toward lastTapeSide.
static void lineFollowStep() {
  uint8_t bits = readTapeBits();
  updateTapeMemory(bits);

  bool L = bits & TAPE_LEFT;
  bool R = bits & TAPE_RIGHT;

  if (L && R) {
    safeMotorsTank(SPEED_FWD, SPEED_FWD);
  } else if (L && !R) {
    safeMotorsTank(SPEED_FWD - SPEED_DELTA, SPEED_FWD + SPEED_DELTA);
  } else if (!L && R) {
    safeMotorsTank(SPEED_FWD + SPEED_DELTA, SPEED_FWD - SPEED_DELTA);
  } else {
    // lost tape
    if (lastTapeSide < 0) safeMotorsTank(-SPEED_SEARCH, SPEED_SEARCH);
    else                  safeMotorsTank(SPEED_SEARCH, -SPEED_SEARCH);
  }
}


// ======================== Ultrasonic =========================

static uint32_t usLastPingMs = 0;
static float    usDistanceCm = -1.0f;
static bool     usValid = false;
static uint8_t  usNearCount = 0;
static uint8_t  usClearCount = 0;
static bool     usNearWall = false;

static float ultrasonicReadOnceCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, US_ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  return (float)duration * US_CM_PER_US;
}

static void ultrasonicUpdate() {
  uint32_t now = millis();
  if ((uint32_t)(now - usLastPingMs) < US_PING_INTERVAL_MS) return;
  usLastPingMs = now;

  float cm = ultrasonicReadOnceCm();
  if (cm <= 0.0f) {
    usValid = false;
    return;
  }

  usValid = true;

  // smooth a bit
  if (usDistanceCm <= 0.0f) usDistanceCm = cm;
  else usDistanceCm = 0.6f * usDistanceCm + 0.4f * cm;

  if (usDistanceCm <= US_NEAR_WALL_CM) {
    usNearCount++;
    usClearCount = 0;
  } else if (usDistanceCm >= US_CLEAR_WALL_CM) {
    usClearCount++;
    usNearCount = 0;
  } else {
    usNearCount = 0;
    usClearCount = 0;
  }

  if (usNearCount >= US_NEAR_N) usNearWall = true;
  if (usClearCount >= US_CLEAR_N) usNearWall = false;
}

static bool TestUsNearWall() {
  if (!usValid) return false;
  return usNearWall;
}

static void ultrasonicReset() {
  usLastPingMs = 0;
  usDistanceCm = -1.0f;
  usValid = false;
  usNearCount = 0;
  usClearCount = 0;
  usNearWall = false;
}

// ============================================================
// =================== Shooter Link Helpers ====================
static bool shooterBusy = false;
static uint16_t cmdSeq = 1;
static uint8_t shotsLeftTotal = 8; // TODO set initial count
static uint8_t lastVolleyFired = 0;

static char rxLine[Proto::LINE_MAX];

static void pollShooterLink() {
  // Drain all available lines each loop
  while (Proto::readLine(link, rxLine, sizeof(rxLine))) {
    Proto::AckMsg ack;
    Proto::DoneMsg done;

    if (Proto::parseAck(rxLine, ack)) {
      Serial.print("[NAV] shooter ACK seq="); Serial.println(ack.seq);
      continue;
    }

    if (Proto::parseDone(rxLine, done)) {
      Serial.print("[NAV] shooter DONE seq="); Serial.print(done.seq);
      Serial.print(" fired="); Serial.println(done.fired);

      shooterBusy = false;
      lastVolleyFired = done.fired;

      if (shotsLeftTotal >= done.fired) shotsLeftTotal -= done.fired;
      else shotsLeftTotal = 0;

      continue;
    }

    Serial.print("[NAV] ??? "); Serial.println(rxLine);
  }
}

static void sendFireOnce(uint8_t shots, uint16_t dist) {
  Proto::sendFire(link, cmdSeq, shots, dist);
  Serial.print("[NAV] sent FIRE seq="); Serial.print(cmdSeq);
  Serial.print(" shots="); Serial.print(shots);
  Serial.print(" dist="); Serial.println(dist);

  shooterBusy = true;
  cmdSeq++;
}


// ======================= IR Orientation ======================
typedef enum {
  ORIENT_IDLE,
  ORIENT_SCAN_SETTLE,
  ORIENT_SCAN_SAMPLE,
  ORIENT_COMPUTE,
  ORIENT_TURN
} OrientPhase;

static OrientPhase orientPhase = ORIENT_IDLE;

static SimpleTimer servoSettleTimer;
static SimpleTimer orientTurnTimer;

static int peak1Angle = SERVO_FWD_DEG;
static int peak2Angle = SERVO_FWD_DEG;
static int bisectorAngle = SERVO_FWD_DEG;
static int orientErrorDeg = 0;

// 909 Hz amplitude window
static int readIrAmplitude909Hz() {
  int min_val = 1023;
  int max_val = 0;

  unsigned long last_sample_time = micros();

  for (int i = 0; i < IR_TOTAL_SAMPLES; ) {
    if (micros() - last_sample_time >= IR_INTERVAL_US) {
      last_sample_time += IR_INTERVAL_US;

      int val = analogRead(PIN_IR_AMP);
      if (val < min_val) min_val = val;
      if (val > max_val) max_val = val;
      i++;
    }
  }
  return (max_val - min_val);
}

static void computeTwoPeaksAndBisector(int &p1Idx, int &p2Idx, int &bisDeg) {
  p1Idx = 0;
  for (int i = 1; i < SCAN_POINTS; i++) {
    if (irAmp[i] > irAmp[p1Idx]) p1Idx = i;
  }

  p2Idx = -1;
  for (int i = 0; i < SCAN_POINTS; i++) {
    int sep = abs(irAngle[i] - irAngle[p1Idx]);
    if (sep < MIN_PEAK_SEPARATION_DEG) continue;

    if (p2Idx < 0 || irAmp[i] > irAmp[p2Idx]) p2Idx = i;
  }

  if (p2Idx < 0) {
    bisDeg = irAngle[p1Idx];
    return;
  }

  bisDeg = (irAngle[p1Idx] + irAngle[p2Idx]) / 2;
}

static void initOrientStart() {
  scanIdx = 0;
  for (int i = 0; i < SCAN_POINTS; i++) {
    irAmp[i] = 0;
    irAngle[i] = SERVO_MIN_DEG + i * SERVO_STEP_DEG;
  }

  irServo.write(SERVO_MIN_DEG);
  servoSettleTimer.start(SERVO_SETTLE_MS);
  orientPhase = ORIENT_SCAN_SETTLE;
}

static void handleInitOrient() {
  switch (orientPhase) {

    case ORIENT_SCAN_SETTLE:
      if (servoSettleTimer.expired()) orientPhase = ORIENT_SCAN_SAMPLE;
      break;

    case ORIENT_SCAN_SAMPLE: {
      int amp = readIrAmplitude909Hz();
      irAmp[scanIdx] = amp;

      scanIdx++;
      if (scanIdx >= SCAN_POINTS) {
        orientPhase = ORIENT_COMPUTE;
      } else {
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

      orientErrorDeg = bisectorAngle - SERVO_FWD_DEG;

      uint32_t turnMs = (uint32_t)(abs(orientErrorDeg) * MS_PER_DEG_ROBOT);

      if (turnMs < 30) {
        safeMotorsStop();
        orientPhase = ORIENT_IDLE;
        return;
      }

      // NOTE: verify this is right/left for your wiring
      if (orientErrorDeg > 0) safeMotorsTank(SPEED_TURN, -SPEED_TURN);
      else                    safeMotorsTank(-SPEED_TURN, SPEED_TURN);

      orientTurnTimer.start(turnMs);
      orientPhase = ORIENT_TURN;

      Serial.print("[ORIENT] p1="); Serial.print(peak1Angle);
      Serial.print(" p2="); Serial.print(peak2Angle);
      Serial.print(" bis="); Serial.print(bisectorAngle);
      Serial.print(" err="); Serial.print(orientErrorDeg);
      Serial.print(" ms="); Serial.println(turnMs);
    } break;

    case ORIENT_TURN:
      if (orientTurnTimer.expired()) {
        safeMotorsStop();
        orientPhase = ORIENT_IDLE;
      }
      break;

    case ORIENT_IDLE:
    default:
      break;
  }
}









// ========================== FULL FSM =========================
typedef enum {
  F_IDLE_WAIT_ENABLE,
  F_INIT_ORIENT,
  F_EXIT_BOX_UNTIL_CROSS,
  F_LINE_FOLLOW_TO_HOG,
  F_AT_HOG_SEND_FIRE,
  F_WAIT_SHOOTER_DONE,

  // return sequence (your updated requirement)
  F_TURN_180_AFTER_FIRE,
  F_TURN_RIGHT_INTO_BOX,
  F_DRIVE_INTO_BOX_US,

  F_RELOAD_WAIT,
  F_REORIENT_180,
  F_DONE
} FullState;

static FullState fullState = F_IDLE_WAIT_ENABLE;

static SimpleTimer fExitDeb;
static SimpleTimer fExitTimeout;
static SimpleTimer fHogDeb;
static SimpleTimer fTurnTimer;
static SimpleTimer fReloadTimer;

static bool prevBtn = false;

static bool readEnableButtonPressedEdge() {
  bool cur = (digitalRead(PIN_ENABLE_BTN) == LOW);
  bool edge = cur && !prevBtn;
  prevBtn = cur;
  return edge;
}

static void fullFsmResetRun() {
  fullState = F_IDLE_WAIT_ENABLE;

  fExitDeb.stop();
  fExitTimeout.stop();
  fHogDeb.stop();
  fTurnTimer.stop();
  fReloadTimer.stop();

  ultrasonicReset();
  lastTapeSide = 0;

  shooterBusy = false;
  orientPhase = ORIENT_IDLE;

  safeMotorsStop();
}

static void startTimedTurnRight(float deg) {
  uint32_t ms = (uint32_t)(deg * MS_PER_DEG_ROBOT);
  // NOTE: verify that this actually turns RIGHT for your wiring.
  safeMotorsTank(SPEED_TURN, -SPEED_TURN);
  fTurnTimer.start(ms);
}

static void fullFsmLoop() {
  pollShooterLink();

  switch (fullState) {

    case F_IDLE_WAIT_ENABLE:
      safeMotorsStop();
      if (readEnableButtonPressedEdge()) {
        Serial.println("[FULL] enable -> INIT_ORIENT");
        initOrientStart();
        fullState = F_INIT_ORIENT;
      }
      break;

    case F_INIT_ORIENT:
      handleInitOrient();
      if (orientPhase == ORIENT_IDLE) {
        Serial.println("[FULL] orient done -> EXIT_BOX_UNTIL_CROSS");
        fExitDeb.stop();
        fExitTimeout.start(EXIT_TIMEOUT_MS);
        fullState = F_EXIT_BOX_UNTIL_CROSS;
      }
      break;

    case F_EXIT_BOX_UNTIL_CROSS:
      safeMotorsTank(SPEED_FWD, SPEED_FWD);

      if (crossingDebounced(fExitDeb, EXIT_CROSS_DEBOUNCE_MS)) {
        Serial.println("[FULL] exit crossing -> LINE_FOLLOW_TO_HOG");
        fHogDeb.stop();
        lastTapeSide = 0;
        fullState = F_LINE_FOLLOW_TO_HOG;
        break;
      }

      if (fExitTimeout.expired()) {
        Serial.println("[FULL] exit timeout -> LINE_FOLLOW_TO_HOG anyway");
        fHogDeb.stop();
        lastTapeSide = 0;
        fullState = F_LINE_FOLLOW_TO_HOG;
        break;
      }
      break;

    case F_LINE_FOLLOW_TO_HOG:
      lineFollowStep();
      if (crossingDebounced(fHogDeb, HOGLINE_DEBOUNCE_MS)) {
        Serial.println("[FULL] hogline crossing -> AT_HOG_SEND_FIRE");
        safeMotorsStop();
        fullState = F_AT_HOG_SEND_FIRE;
      }
      break;

    case F_AT_HOG_SEND_FIRE: {
      safeMotorsStop();

      if (shotsLeftTotal == 0) {
        Serial.println("[FULL] no shots left -> DONE");
        fullState = F_DONE;
        break;
      }

      uint8_t volley = min((uint8_t)DEFAULT_SHOTS_PER_VOLLEY, shotsLeftTotal);
      sendFireOnce(volley, DEFAULT_DISTANCE_METRIC);
      fullState = F_WAIT_SHOOTER_DONE;
    } break;

    case F_WAIT_SHOOTER_DONE:
      safeMotorsStop();
      if (!shooterBusy) {
        Serial.println("[FULL] shooter done -> TURN_180_AFTER_FIRE");
        startTimedTurnRight(180.0f);          // timed 180 using "right turn" helper (direction depends on your wiring)
        ultrasonicReset();                   // ultrasonic only used on box-entry drive
        fullState = F_TURN_180_AFTER_FIRE;
      }
      break;

    case F_TURN_180_AFTER_FIRE:
      if (fTurnTimer.expired()) {
        safeMotorsStop();
        Serial.println("[FULL] 180 done -> TURN_RIGHT_INTO_BOX");
        startTimedTurnRight(90.0f);           // turn right into the box
        fullState = F_TURN_RIGHT_INTO_BOX;
      }
      break;

    case F_TURN_RIGHT_INTO_BOX:
      if (fTurnTimer.expired()) {
        safeMotorsStop();
        Serial.println("[FULL] right turn done -> DRIVE_INTO_BOX_US");
        ultrasonicReset();
        fullState = F_DRIVE_INTO_BOX_US;
      }
      break;

    case F_DRIVE_INTO_BOX_US:
      ultrasonicUpdate();

      // drive straight into the box
      safeMotorsTank(SPEED_FWD, SPEED_FWD);

      if (TestUsNearWall()) {
        Serial.println("[FULL] near wall -> RELOAD_WAIT");
        safeMotorsStop();
        fReloadTimer.start(RELOAD_WAIT_MS);
        fullState = F_RELOAD_WAIT;
      }
      break;

    case F_RELOAD_WAIT:
      safeMotorsStop();
      if (fReloadTimer.expired()) {
        Serial.println("[FULL] reload done -> REORIENT_180");
        startTimedTurnRight(180.0f);          // face out again
        fullState = F_REORIENT_180;
      }
      break;

    case F_REORIENT_180:
      if (fTurnTimer.expired()) {
        safeMotorsStop();

        if (shotsLeftTotal == 0) {
          Serial.println("[FULL] shots exhausted -> DONE");
          fullState = F_DONE;
        } else {
          Serial.println("[FULL] next loop -> INIT_ORIENT");
          initOrientStart();
          fullState = F_INIT_ORIENT;
        }
      }
      break;

    case F_DONE:
    default:
      safeMotorsStop();
      // slow blink to indicate DONE
      static uint32_t last = 0;
      static bool on = false;
      if (millis() - last > 600) {
        last = millis();
        on = !on;
        digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
      }
      break;
  }
}












// ============================================================
// ===================== Stage/Test Runner =====================

/*
  STAGE MODES:
    0  = FULL_FSM
    1  = TAPE_MONITOR_RAW
    2  = ULTRASONIC_MONITOR
    3  = IR_AMPLITUDE_MONITOR_909HZ      (no servo sweep)
    4  = SERVO_SWEEP_ONLY               (servo moves, no IR)
    5  = IR_SCAN_NO_TURN                (scan + print peaks, no wheel turn)
    6  = IR_ORIENT_ONLY                 (scan + wheel turn)
    7  = EXIT_BOX_UNTIL_CROSS
    8  = LINE_FOLLOW_UNTIL_HOG
    9  = TURN_90_TIMED
    10 = TURN_180_TIMED
    11 = MOTORS_CONSTANT_FORWARD
    12 = MOTORS_TOGGLE_DIRECTION_DEMO
    13 = SHOOTER_LINK_MONITOR
    14 = SHOOTER_FIRE_ONCE
    15 = DRIVE_INTO_BOX_UNTIL_US_NEAR   (straight drive + ultrasonic stop)
*/

static bool stageDone = false;

static void markStageDone(const char *msg) {
  if (stageDone) return;
  stageDone = true;
  safeMotorsStop();
  Serial.print("[STAGE DONE] ");
  Serial.println(msg);
}

static void stageIdleBlink() {
  static uint32_t last = 0;
  static bool on = false;
  if (millis() - last > 350) {
    last = millis();
    on = !on;
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
  }
}

static const char* stageName(uint8_t s) {
  switch (s) {
    case 0:  return "FULL_FSM";
    case 1:  return "TAPE_MONITOR_RAW";
    case 2:  return "ULTRASONIC_MONITOR";
    case 3:  return "IR_AMPLITUDE_MONITOR_909HZ";
    case 4:  return "SERVO_SWEEP_ONLY";
    case 5:  return "IR_SCAN_NO_TURN";
    case 6:  return "IR_ORIENT_ONLY";
    case 7:  return "EXIT_BOX_UNTIL_CROSS";
    case 8:  return "LINE_FOLLOW_UNTIL_HOG";
    case 9:  return "TURN_90_TIMED";
    case 10: return "TURN_180_TIMED";
    case 11: return "MOTORS_CONSTANT_FORWARD";
    case 12: return "MOTORS_TOGGLE_DIRECTION_DEMO";
    case 13: return "SHOOTER_LINK_MONITOR";
    case 14: return "SHOOTER_FIRE_ONCE";
    case 15: return "DRIVE_INTO_BOX_UNTIL_US_NEAR";
    default: return "UNKNOWN";
  }
}

// ---- Stage 1: tape monitor ----
static void stageTapeMonitorRaw() {
  safeMotorsStop();

  static uint32_t last = 0;
  if (millis() - last < 100) return;
  last = millis();

  int l = analogRead(PIN_TAPE_L);
  int r = analogRead(PIN_TAPE_R);
  uint8_t bits = readTapeBits();

  Serial.print("[TAPE] L="); Serial.print(l);
  Serial.print(" R="); Serial.print(r);
  Serial.print(" bits=");
  Serial.print((bits & TAPE_LEFT) ? "L" : ".");
  Serial.println((bits & TAPE_RIGHT) ? "R" : ".");
}

// ---- Stage 2: ultrasonic monitor ----
static void stageUltrasonicMonitor() {
  safeMotorsStop();
  ultrasonicUpdate();

  static uint32_t last = 0;
  if (millis() - last < 120) return;
  last = millis();

  Serial.print("[US] valid="); Serial.print(usValid ? "Y" : "N");
  Serial.print(" cm="); Serial.print(usDistanceCm);
  Serial.print(" near="); Serial.println(usNearWall ? "Y" : "N");
}

// ---- Stage 3: IR amplitude monitor (no servo sweep) ----
static void stageIrAmplitudeMonitor() {
  safeMotorsStop();

  static uint32_t last = 0;
  if (millis() - last < 80) return;
  last = millis();

  int amp = readIrAmplitude909Hz();
  Serial.print("[IR] amp="); Serial.println(amp);
}

// ---- Stage 4: servo sweep only ----
static void stageServoSweepOnly() {
  safeMotorsStop();

  static int pos = SERVO_MIN_DEG;
  static int dir = +1;
  static uint32_t last = 0;

  if (millis() - last < 20) return;
  last = millis();

  irServo.write(pos);
  pos += dir;

  if (pos >= SERVO_MAX_DEG) { pos = SERVO_MAX_DEG; dir = -1; }
  if (pos <= SERVO_MIN_DEG) { pos = SERVO_MIN_DEG; dir = +1; }
}

// ---- Stage 5: IR scan, compute peaks, print, no wheel turn ----
static void stageIrScanNoTurn() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE] IR scan (no wheel turn)");
    initOrientStart();
  }

  // If it enters ORIENT_TURN for any reason, stop it
  if (orientPhase == ORIENT_TURN) {
    safeMotorsStop();
    orientPhase = ORIENT_IDLE;
  } else {
    handleInitOrient();
  }

  if (orientPhase == ORIENT_IDLE) {
    Serial.print("[IR] peak1="); Serial.print(peak1Angle);
    Serial.print(" peak2="); Serial.print(peak2Angle);
    Serial.print(" bis="); Serial.print(bisectorAngle);
    Serial.print(" err="); Serial.println(orientErrorDeg);
    markStageDone("IR scan complete (no turn).");
  }
}

// ---- Stage 6: IR orient only ----
static void stageIrOrientOnly() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE] IR orient only (scan + wheel turn)");
    initOrientStart();
  }

  handleInitOrient();

  if (orientPhase == ORIENT_IDLE) {
    markStageDone("IR orient complete.");
  }
}

// ---- Stage 7: exit box until crossing ----
static SimpleTimer sExitDeb;
static SimpleTimer sExitTimeout;

static void stageExitUntilCross() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE] Exit box until crossing.");
    sExitDeb.stop();
    sExitTimeout.start(EXIT_TIMEOUT_MS);
  }

  safeMotorsTank(SPEED_FWD, SPEED_FWD);

  if (crossingDebounced(sExitDeb, EXIT_CROSS_DEBOUNCE_MS)) {
    markStageDone("Exit crossing detected.");
    return;
  }

  if (sExitTimeout.expired()) {
    markStageDone("Exit timeout (no crossing).");
    return;
  }
}

// ---- Stage 8: line follow until hogline ----
static SimpleTimer sHogDeb;

static void stageFollowUntilHog() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE] Line follow until hogline crossing.");
    sHogDeb.stop();
    lastTapeSide = 0;
  }

  lineFollowStep();

  if (crossingDebounced(sHogDeb, HOGLINE_DEBOUNCE_MS)) {
    markStageDone("Hogline crossing detected.");
  }
}

// ---- Stage 9/10: timed turns ----
static SimpleTimer sTurn;
static bool sTurnStarted = false;

static void stageTurnTimed(float deg) {
  if (stageDone) { stageIdleBlink(); return; }
  Serial.println("Inside Stage Turn Timed subroutine -- trying to turn \n");
  Serial.println(sTurnStarted);


  if (!sTurnStarted) {
    sTurnStarted = true;

    uint32_t ms = (uint32_t)(deg * MS_PER_DEG_ROBOT);

    Serial.print("[STAGE] Timed turn deg=");
    Serial.print(deg);
    Serial.print(" ms=");
    Serial.println(ms);

    // Default "right turn" assumption; verify for your wiring
    safeMotorsTank(SPEED_TURN, -SPEED_TURN);
    sTurn.start(ms);
  }

  if (sTurn.expired()) {
    safeMotorsStop();
    markStageDone("Timed turn complete.");
  }
}

// ---- Stage 11: motors constant forward ----
static void stageMotorsConstantForward() {
  if (!TEST_ENABLE_MOTORS) {
    safeMotorsStop();
    static uint32_t last = 0;
    if (millis() - last > 500) {
      last = millis();
      Serial.println("[STAGE] TEST_ENABLE_MOTORS=false (motors disabled).");
    }
    return;
  }

  safeMotorsTank(SPEED_FWD, SPEED_FWD);

  static uint32_t last = 0;
  if (millis() - last > 250) {
    last = millis();
    Serial.print("[STAGE] motors L="); Serial.print(SPEED_FWD);
    Serial.print(" R="); Serial.println(SPEED_FWD);
  }
}

// ---- Stage 12: motor direction toggle demo ----
enum DirState { RUNNING, WAITING };
static DirState dirState = WAITING;
static bool isForward = true;
static uint32_t lastToggleMs = 0;
static const uint32_t TIME_DELAY_MS = 1000;

static bool prevBtn2 = false;
static bool buttonEdgeForToggle() {
  bool cur = (digitalRead(PIN_ENABLE_BTN) == LOW);
  bool edge = cur && !prevBtn2;
  prevBtn2 = cur;
  return edge;
}

static void stageMotorToggleDemo() {
  if (!TEST_ENABLE_MOTORS) {
    safeMotorsStop();
    static uint32_t last = 0;
    if (millis() - last > 500) {
      last = millis();
      Serial.println("[STAGE12] motors disabled (TEST_ENABLE_MOTORS=false).");
    }
    return;
  }

  if (buttonEdgeForToggle()) {
    if (dirState == WAITING) {
      dirState = RUNNING;
      lastToggleMs = millis();
      Serial.println("[STAGE12] RUNNING");
    } else {
      dirState = WAITING;
      Serial.println("[STAGE12] WAITING");
    }
  }

  if (dirState == WAITING) {
    safeMotorsStop();
    return;
  }

  if (millis() - lastToggleMs >= TIME_DELAY_MS) {
    lastToggleMs += TIME_DELAY_MS;
    isForward = !isForward;
    Serial.print("[STAGE12] Toggled -> ");
    Serial.println(isForward ? "forward" : "backward");
  }

  if (isForward) safeMotorsTank(SPEED_FWD, SPEED_FWD);
  else           safeMotorsTank(-SPEED_FWD, -SPEED_FWD);
}

// ---- Stage 13: shooter link monitor ----
static void stageShooterLinkMonitor() {
  safeMotorsStop();
  pollShooterLink();

  static uint32_t last = 0;
  if (millis() - last > 500) {
    last = millis();
    Serial.println("[STAGE13] Listening for ACK/DONE...");
  }
}

// ---- Stage 14: shooter fire once ----
static void stageShooterFireOnce() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  static bool sent = false;

  if (!started) {
    started = true;
    sent = false;
    Serial.println("[STAGE14] Send FIRE once, wait DONE.");
    safeMotorsStop();
  }

  pollShooterLink();

  if (!sent) {
    if (shotsLeftTotal == 0) {
      markStageDone("No shots left to fire.");
      return;
    }
    uint8_t volley = min((uint8_t)DEFAULT_SHOTS_PER_VOLLEY, shotsLeftTotal);
    sendFireOnce(volley, DEFAULT_DISTANCE_METRIC);
    sent = true;
  }

  if (sent && !shooterBusy) {
    markStageDone("Shooter DONE received.");
  }
}

// ---- Stage 15: drive into box until ultrasonic near wall ----
static void stageDriveIntoBoxUntilUsNear() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE15] Drive forward until ultrasonic near wall.");
    ultrasonicReset();
  }

  ultrasonicUpdate();
  safeMotorsTank(SPEED_FWD, SPEED_FWD);

  if (TestUsNearWall()) {
    markStageDone("Near wall detected.");
  }
}


// ===================== Arduino setup/loop ====================
void setup() {
  pinMode(PIN_ENABLE_BTN, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Ultrasonic pins
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  Serial.begin(115200);
  link.begin(19200);

  Serial.println("[NAV] boot");
  Serial.print("[NAV] STAGE_MODE="); Serial.print(STAGE_MODE);
  Serial.print(" ("); Serial.print(stageName(STAGE_MODE)); Serial.println(")");
  Serial.print("[NAV] TEST_ENABLE_MOTORS="); Serial.println(TEST_ENABLE_MOTORS ? "true" : "false");

  // Speed up ADC (your prescaler trick)
  bitClear(ADCSRA, ADPS0);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);

  // Servo init
  irServo.attach(PIN_IR_SERVO);
  irServo.write(SERVO_FWD_DEG);

  pinMode(PIN_L_ENA, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);

  pinMode(PIN_R_ENA, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);

  safeMotorsStop();

  stageDone = false;
  ultrasonicReset();
  lastTapeSide = 0;

  // reset stage-specific state
  sExitDeb.stop();
  sExitTimeout.stop();
  sHogDeb.stop();
  sTurn.stop();
  sTurnStarted = false;

  // reset shooter bookkeeping
  shooterBusy = false;
  cmdSeq = 1;

  // FULL FSM reset
  fullFsmResetRun();

  Serial.println("[NAV] ready");
}

void loop() {
  switch (STAGE_MODE) {
    case 0:  fullFsmLoop();                 break;
    case 1:  stageTapeMonitorRaw();         break;
    case 2:  stageUltrasonicMonitor();      break;
    case 3:  stageIrAmplitudeMonitor();     break;
    case 4:  stageServoSweepOnly();         break;
    case 5:  stageIrScanNoTurn();           break;
    case 6:  stageIrOrientOnly();           break;
    case 7:  stageExitUntilCross();         break;
    case 8:  stageFollowUntilHog();         break;
    case 9:  stageTurnTimed(90.0f);         break;
    case 10: stageTurnTimed(180.0f);        break;
    case 11: stageMotorsConstantForward();  break;
    case 12: stageMotorToggleDemo();        break;
    case 13: stageShooterLinkMonitor();     break;
    case 14: stageShooterFireOnce();        break;
    case 15: stageDriveIntoBoxUntilUsNear();break;

    default:
      safeMotorsStop();
      digitalWrite(PIN_STATUS_LED, HIGH);
      break;
  }
}