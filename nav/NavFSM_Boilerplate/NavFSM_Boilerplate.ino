#include <Arduino.h>
#include <Stepper.h>

// ========================= Timers ============================
struct SimpleTimer {
  uint32_t t0 = 0;
  uint32_t dur = 0;
  bool running = false;

  void start(uint32_t durationMs) { t0 = millis(); dur = durationMs; running = true; }
  void stop() { running = false; }
  bool expired() const { return running && (uint32_t)(millis() - t0) >= dur; }
};

// ===================== STAGE SELECT HERE =====================
// 0 = FULL_FSM
// 1 = TAPE_MONITOR_RAW
// 2 = ULTRASONIC_MONITOR
// 3 = EXIT_BOX_UNTIL_CROSS
// 4 = LINE_FOLLOW_UNTIL_HOG
// 5 = TURN_90_TIMED
// 6 = TURN_180_TIMED
// 7 = MOTORS_CONSTANT_FORWARD
// 8 = MOTORS_TOGGLE_DIRECTION_DEMO
// 9 = SWIVEL_STEPPER_TEST (step +90 every second)
// 10 = SWIVEL_FIRE_ONCE (step +90 once)
// 11 = DRIVE_UNTIL_US_NEAR
// 12 = ESCAPE_BOX_US_THEN_CROSS (UPDATED: turn-until-clear + backup-if-too-close, then drive to exit tape)

static const uint8_t STAGE_MODE = 0;

// Safety gate: robot will not move unless true.
static const bool TEST_ENABLE_MOTORS = true;

// ========================= Pin Map ===========================

// Tape sensors: outer-left, inner-left, inner-right, outer-right
static const uint8_t PIN_TAPE_LO = A4;
static const uint8_t PIN_TAPE_LI = A5;
static const uint8_t PIN_TAPE_RI = A1;
static const uint8_t PIN_TAPE_RO = A2;

// Enable button (active-low)
static const uint8_t PIN_ENABLE_BTN = 4;

// MOTORS
static const uint8_t PIN_L_ENA = 9;   // PWM
static const uint8_t PIN_L_IN1 = 7;
static const uint8_t PIN_L_IN2 = 8;

static const uint8_t PIN_R_ENA = 10;  // PWM
static const uint8_t PIN_R_IN1 = 12;
static const uint8_t PIN_R_IN2 = 11;

// Status LED (disabled because you’re using D13 as stepper enable)
static const uint8_t PIN_STATUS_LED = 255;

// Ultrasonic (HC-SR04 style)
static const uint8_t PIN_US_TRIG = 5;
static const uint8_t PIN_US_ECHO = 6;

// ===================== Swivel Stepper (test-sketch style) ========================

// 200 steps = 360 degrees
static const int stepsPerRevolution = 200;
static const int steps90Degrees     = 50;

// Enable pin (must be HIGH for motor to move) — per your test
static const uint8_t PIN_SWIVEL_EN = 13;

// Coil pins (must NOT conflict with your bot’s other pins)
static const uint8_t PIN_SWIVEL_1 = 2;
static const uint8_t PIN_SWIVEL_2 = 3;
static const uint8_t PIN_SWIVEL_3 = A0;  // digital OK
static const uint8_t PIN_SWIVEL_4 = A3;  // digital OK

// Initialize the stepper (same ordering pattern as your test)
static Stepper myStepper(stepsPerRevolution, PIN_SWIVEL_4, PIN_SWIVEL_3, PIN_SWIVEL_2, PIN_SWIVEL_1);

// ======================== Constants ==========================

// -------- Tape + motion --------
static const int TAPE_THRESH_LO = 200;
static const int TAPE_THRESH_LI = 200;
static const int TAPE_THRESH_RI = 200;
static const int TAPE_THRESH_RO = 200;

static const int16_t SPEED_FWD    = 175;
static const int16_t SPEED_TURN   = 255;  // used for timed turns
static const int16_t SPEED_SEARCH = 150;
static const int16_t SPEED_DELTA  = 35;

// Robot turn calibration ONLY (timed turns)
static const float MS_PER_DEG_ROBOT = 3.3f;

// Tape crossing debounce
static const uint32_t EXIT_CROSS_DEBOUNCE_MS = 1000;
static const uint32_t HOGLINE_DEBOUNCE_MS    = 120;

// Exit box fallback timeout
static const uint32_t EXIT_TIMEOUT_MS = 3000;

// Ultrasonic
static const uint32_t US_PING_INTERVAL_MS = 60;
static const uint32_t US_ECHO_TIMEOUT_US = 25000;
static const float    US_CM_PER_US = 0.0343f / 2.0f;

// Hysteresis / debounce
static const float    US_NEAR_WALL_CM  = 12.0f;
static const float    US_CLEAR_WALL_CM = 25.0f;
static const uint8_t  US_NEAR_N        = 3;
static const uint8_t  US_CLEAR_N = 3;

// ===== New “turn-until-clear + backup” tuning knobs =====
static const float    US_TOO_CLOSE_CM  = 6.0f;   // "stuck / corner" threshold
static const uint32_t BACKUP_TIME_MS   = 350;    // backup duration when too close
static const int16_t  SPEED_TURN_SCAN  = 140;    // turning speed while scanning for clear path

static bool stageDone = false;

// =============== Hardware abstraction ========================
static int readAnalogSettled(uint8_t pin);

// ===================== Status LED helpers ====================
static inline void statusLedInit() {
  if (PIN_STATUS_LED != 255) pinMode(PIN_STATUS_LED, OUTPUT);
}
static inline void statusLedWrite(bool on) {
  if (PIN_STATUS_LED != 255) digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

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

static void swivelStop() {
  digitalWrite(PIN_SWIVEL_EN, LOW);
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
  TAPE_LO    = 1 << 0,
  TAPE_LI    = 1 << 1,
  TAPE_RI    = 1 << 2,
  TAPE_RO    = 1 << 3
};

static uint8_t readTapeBits() {
  int lo = readAnalogSettled(PIN_TAPE_LO);
  int li = readAnalogSettled(PIN_TAPE_LI);
  int ri = readAnalogSettled(PIN_TAPE_RI);
  int ro = readAnalogSettled(PIN_TAPE_RO);

  uint8_t bits = TAPE_NONE;
  if (lo < TAPE_THRESH_LO) bits |= TAPE_LO;
  if (li < TAPE_THRESH_LI) bits |= TAPE_LI;
  if (ri < TAPE_THRESH_RI) bits |= TAPE_RI;
  if (ro < TAPE_THRESH_RO) bits |= TAPE_RO;

  return bits;
}

static bool isTapeCrossing(uint8_t bits) {
  uint8_t count = 0;
  if (bits & TAPE_LO) count++;
  if (bits & TAPE_LI) count++;
  if (bits & TAPE_RI) count++;
  if (bits & TAPE_RO) count++;
  return (count >= 3);
}

static bool crossingDebounced(SimpleTimer &deb, uint32_t debounceMs) {
  uint8_t bits = readTapeBits();

  if (isTapeCrossing(bits)) {
    if (!deb.running) deb.start(debounceMs);

    if (deb.expired()) {
      Serial.println("[TAPE] debounced crossing TRUE");
      return true;
    }
    return false;
  } else {
    deb.stop();
    return false;
  }
}

// -1 = last saw LEFT, +1 = last saw RIGHT, 0 = centered/unknown
static int8_t lastTapeSide = 0;

static void updateTapeMemory(uint8_t bits) {
  int leftScore = 0;
  int rightScore = 0;

  if (bits & TAPE_LO) leftScore += 2;
  if (bits & TAPE_LI) leftScore += 1;

  if (bits & TAPE_RO) rightScore += 2;
  if (bits & TAPE_RI) rightScore += 1;

  if (leftScore > rightScore) lastTapeSide = -1;
  else if (rightScore > leftScore) lastTapeSide = +1;
  else lastTapeSide = 0;
}

static void lineFollowStep() {
  uint8_t bits = readTapeBits();
  updateTapeMemory(bits);

  bool lo = bits & TAPE_LO;
  bool li = bits & TAPE_LI;
  bool ri = bits & TAPE_RI;
  bool ro = bits & TAPE_RO;

  if (li && ri && !lo && !ro) { safeMotorsTank(SPEED_FWD, SPEED_FWD); return; }
  if (isTapeCrossing(bits))   { safeMotorsTank(SPEED_FWD, SPEED_FWD); return; }

  if (li && !ri && !lo) { safeMotorsTank(SPEED_FWD - SPEED_DELTA, SPEED_FWD + SPEED_DELTA); return; }
  if (ri && !li && !ro) { safeMotorsTank(SPEED_FWD + SPEED_DELTA, SPEED_FWD - SPEED_DELTA); return; }

  if (lo) { safeMotorsTank(SPEED_FWD - 2 * SPEED_DELTA, SPEED_FWD + 2 * SPEED_DELTA); return; }
  if (ro) { safeMotorsTank(SPEED_FWD + 2 * SPEED_DELTA, SPEED_FWD - 2 * SPEED_DELTA); return; }

  if (lastTapeSide < 0) safeMotorsTank(-SPEED_SEARCH, SPEED_SEARCH);
  else if (lastTapeSide > 0) safeMotorsTank(SPEED_SEARCH, -SPEED_SEARCH);
  else safeMotorsTank(SPEED_SEARCH, SPEED_SEARCH);
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
  
  // If it times out, the wall is out of range. 
  // Treat this as "infinity" (e.g., 999 cm) so the robot knows the path is clear.
  if (duration == 0) return 999.0f; 

  return (float)duration * US_CM_PER_US;
}

static void ultrasonicUpdate() {
  uint32_t now = millis();
  if ((uint32_t)(now - usLastPingMs) < US_PING_INTERVAL_MS) return;
  usLastPingMs = now;

  float cm = ultrasonicReadOnceCm();
  
  // Reject physically impossible ghost readings (e.g., sensor minimum is usually ~2cm)
  if (cm < 2.0f) {
    return; // Ignore this reading entirely, keep previous state
  }

  usValid = true;

  // EMA Filter
  if (usDistanceCm <= 0.0f) usDistanceCm = cm;
  else usDistanceCm = 0.6f * usDistanceCm + 0.4f * cm;

  // Hysteresis logic
  if (usDistanceCm <= US_NEAR_WALL_CM) { usNearCount++; usClearCount = 0; }
  else if (usDistanceCm >= US_CLEAR_WALL_CM) { usClearCount++; usNearCount = 0; }
  else { usNearCount = 0; usClearCount = 0; }

  if (usNearCount >= US_NEAR_N) usNearWall = true;
  if (usClearCount >= US_CLEAR_N) usNearWall = false;
}

static bool testUsNearWall() {
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

enum RetState {
  RET_INIT = 0,
  RET_TURN_180 = 1,
  RET_DRIVE_1 = 2,
  RET_TURN_90_CW = 3,
  RET_DRIVE_2 = 4,
  RET_TURN_90_CW2 = 5,
  RET_REVERSE_FINAL = 6,
  RET_DONE = 7
};

static RetState sRetState = RET_INIT;
static SimpleTimer sRetTimer;
static bool retStarted = false;


// ========================== FULL FSM =========================
typedef enum {
  F_IDLE_WAIT_ENABLE,
  CASE_12,
  CASE_4,
  CASE_RETURN_TO_START, // Replaced F_TURN_180_AFTER_FIRE and F_DRIVE_TO_WALL
  F_DONE
} FullState;

static FullState fullState = CASE_12;

static SimpleTimer fExitDeb;
static SimpleTimer fExitTimeout;
static SimpleTimer fHogDeb;
static SimpleTimer fTurnTimer;
static SimpleTimer fBackupTimer;

static bool prevBtn = false;

static void fullFsmResetRun() {
  fullState = CASE_12;
  stageDone = false; 
  retStarted = false; // Reset the new stage's initialization flag
  ultrasonicReset();
  lastTapeSide = 0;
  safeMotorsStop();
}

static void fullFsmLoop() {
  ultrasonicUpdate();

  switch (fullState) {

    case CASE_12:
      stageEscapeBoxUsThenCross();
      if (stageDone) {              
        stageDone = false;          
        fullState = CASE_4;         
      }
      break;

    case CASE_4:
      stageFollowUntilHog();
      if (stageDone) {
        stageDone = false;
        fullState = CASE_RETURN_TO_START; // Trigger the new return sequence
      }
      break;

    case CASE_RETURN_TO_START:
      stageReturnToStart();
      if (stageDone) {
        stageDone = false;
        fullState = F_DONE;
      }
      break;

    case F_DONE:
      Serial.println("FSM Complete");
      safeMotorsStop();
      break;

    default:
      safeMotorsStop();
      break;
  }
} 
// ============================================================
// ===================== Stage/Test Runner =====================


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
    statusLedWrite(on);
  }
}

static const char* stageName(uint8_t s) {
  switch (s) {
    case 0:  return "FULL_FSM";
    case 1:  return "TAPE_MONITOR_RAW";
    case 2:  return "ULTRASONIC_MONITOR";
    case 3:  return "EXIT_BOX_UNTIL_CROSS";
    case 4:  return "LINE_FOLLOW_UNTIL_HOG";
    case 5:  return "TURN_90_TIMED";
    case 6:  return "TURN_180_TIMED";
    case 7:  return "MOTORS_CONSTANT_FORWARD";
    case 8:  return "MOTORS_TOGGLE_DIRECTION_DEMO";
    case 9:  return "SWIVEL_STEPPER_TEST";
    case 10: return "SWIVEL_FIRE_ONCE";
    case 11: return "DRIVE_UNTIL_US_NEAR";
    case 12: return "ESCAPE_BOX_US_THEN_CROSS";
    case 13: return "COMBO_EXIT_AND_FOLLOW";
    case 14: return "RETURN_TO_START";
    default: return "UNKNOWN";
  }
}

// ---- Stage 1: tape monitor ----
static void stageTapeMonitorRaw() {
  safeMotorsStop();

  static uint32_t last = 0;
  if (millis() - last < 120) return;
  last = millis();

  int lo = readAnalogSettled(PIN_TAPE_LO);
  int li = readAnalogSettled(PIN_TAPE_LI);
  int ri = readAnalogSettled(PIN_TAPE_RI);
  int ro = readAnalogSettled(PIN_TAPE_RO);

  uint8_t bits = readTapeBits();

  Serial.print("[TAPE] ");
  Serial.print("LO="); Serial.print(lo);
  Serial.print(" LI="); Serial.print(li);
  Serial.print(" RI="); Serial.print(ri);
  Serial.print(" RO="); Serial.print(ro);
  Serial.print(" bits=");
  Serial.print((bits & TAPE_LO) ? "LO " : ".. ");
  Serial.print((bits & TAPE_LI) ? "LI " : ".. ");
  Serial.print((bits & TAPE_RI) ? "RI " : ".. ");
  Serial.println((bits & TAPE_RO) ? "RO" : "..");
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

// ---- Stage 3: exit box until crossing ----
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

// ---- Stage 4: line follow until hogline ----
enum LfState {
  LF_INIT = 0,
  LF_TURN_CW_45 = 1,
  LF_DRIVE_TO_LINE = 2,
  LF_TURN_CCW_45 = 3,
  LF_FOLLOWING = 4,
  LF_REVERSE_AFTER_HOG = 5  // Moved reverse to the final phase
};

static LfState sLfState = LF_INIT;
static SimpleTimer sLfTimer;
static bool lfStarted = false;

// --- Local Stage 4 Constants ---
static const int16_t SPEED_FIND_LINE = 150; 
static const int16_t SPEED_BACKUP_LF = -120; // Slightly faster backup to ensure it clears the line

static void stageFollowUntilHog() {
  if (stageDone) { stageIdleBlink(); return; }

  if (!lfStarted) {
    lfStarted = true;
    sLfState = LF_INIT;
  }

  uint8_t tapeBits = readTapeBits();
  bool leftOuter  = (tapeBits & TAPE_LO);
  bool leftInner  = (tapeBits & TAPE_LI);
  bool rightInner = (tapeBits & TAPE_RI);
  bool rightOuter = (tapeBits & TAPE_RO);

  switch (sLfState) {
    case LF_INIT:
      Serial.println("[STAGE 4] Init: Skipping reverse, going straight to CW turn.");
      sLfTimer.start((uint32_t)(45.0f * MS_PER_DEG_ROBOT));
      sLfState = LF_TURN_CW_45;
      break;

    case LF_TURN_CW_45:
      safeMotorsTank(SPEED_TURN, -SPEED_TURN);
      if (sLfTimer.expired()) {
        safeMotorsStop();
        Serial.println("[STAGE 4] Driving to intercept line...");
        
        // NEW: Start a 500ms "blind" timer so the robot clears the starting area
        sLfTimer.start(500); 
        
        sLfState = LF_DRIVE_TO_LINE;
      }
      break;

    case LF_DRIVE_TO_LINE:
      safeMotorsTank(SPEED_FIND_LINE, SPEED_FIND_LINE);
      
      // NEW: Wait for the 500ms blind timer to expire BEFORE we start checking sensors.
      // Then, stop if ANY of the four sensors hit the tape.
      if (sLfTimer.expired() && (leftInner)) {
        safeMotorsStop();
        Serial.println("[STAGE 4] Intercepted! Squaring up CCW...");
        sLfTimer.start((uint32_t)(45.0f * MS_PER_DEG_ROBOT));
        sLfState = LF_TURN_CCW_45;
      }
      break;

    case LF_TURN_CCW_45:
      safeMotorsTank(-SPEED_TURN, SPEED_TURN);
      if (sLfTimer.expired()) {
        safeMotorsStop();
        Serial.println("[STAGE 4] Align complete. Running Follower.");
        lastTapeSide = 0;
        sLfState = LF_FOLLOWING;
      }
      break;

    case LF_FOLLOWING:
      // REQUIREMENT: Both outers AND at least one inner must see tape for Hog Line
      if (leftOuter && rightOuter && (leftInner || rightInner)) {
        safeMotorsStop();
        Serial.println("[STAGE 4] Hogline hit! Starting 2s reverse reset...");
        sLfTimer.start(900); // 2-second reverse
        sLfState = LF_REVERSE_AFTER_HOG;
      } else {
        lineFollowStep(); 
      }
      break;

    case LF_REVERSE_AFTER_HOG:
      safeMotorsTank(SPEED_BACKUP_LF, SPEED_BACKUP_LF);
      if (sLfTimer.expired()) {
        safeMotorsStop();
        myStepper.step(steps90Degrees);
        delay(100);
        myStepper.step(steps90Degrees);
        delay(100);
        myStepper.step(steps90Degrees);
        delay(100);
        markStageDone("Hogline hit and 2s reverse complete.");
      }
      break;
  }
}

// ---- Stage 5/6: timed turns ----
static SimpleTimer sTurn;
static bool sTurnStarted = false;

static void stageTurnTimed(float deg) {
  if (stageDone) { stageIdleBlink(); return; }

  if (!sTurnStarted) {
    sTurnStarted = true;

    uint32_t ms = (uint32_t)(deg * MS_PER_DEG_ROBOT);
    Serial.print("[STAGE] Timed turn deg="); Serial.print(deg);
    Serial.print(" ms="); Serial.println(ms);

    safeMotorsTank(SPEED_TURN, -SPEED_TURN);
    sTurn.start(ms);
  }

  if (sTurn.expired()) {
    safeMotorsStop();
    markStageDone("Timed turn complete.");
  }
}

// ---- Stage 7: motors constant forward ----
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
}

// ---- Stage 8: motor direction toggle demo ----
enum DirState { RUNNING, WAITING };
static DirState dirState = WAITING;
static bool isForward = true;
static uint32_t lastToggleMs = 0;
static const uint32_t TIME_DELAY_MS = 1000;

static bool prevBtn2 = false;
static bool buttonEdgeForToggle() {
  bool cur = (digitalRead(PIN_ENABLE_BTN) == HIGH);
  bool edge = cur && !prevBtn2;
  prevBtn2 = cur;
  return edge;
}

static void     stageMotorToggleDemo() {
  if (!TEST_ENABLE_MOTORS) { safeMotorsStop(); return; }

  if (buttonEdgeForToggle()) {
    if (dirState == WAITING) { dirState = RUNNING; lastToggleMs = millis(); }
    else { dirState = WAITING; }
  }

  if (dirState == WAITING) { safeMotorsStop(); return; }

  if (millis() - lastToggleMs >= TIME_DELAY_MS) {
    lastToggleMs += TIME_DELAY_MS;
    isForward = !isForward;
  }

  if (isForward) safeMotorsTank(SPEED_FWD, SPEED_FWD);
  else safeMotorsTank(-SPEED_FWD, -SPEED_FWD);
}

// ---- Stage 9: swivel stepper test (like your sketch) ----
static void stageSwivelMotorTest() {
  if (!TEST_ENABLE_MOTORS) { swivelStop(); return; }
  safeMotorsStop();
  myStepper.step(steps90Degrees);
  delay(100);
  myStepper.step(steps90Degrees);
  delay(100);
  myStepper.step(steps90Degrees);
  delay(100);
}

// ---- Stage 10: swivel fire once (like your sketch) ----
static void stageSwivelFireOnce() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    safeMotorsStop();
    myStepper.step(steps90Degrees);
    delay(1000);
    swivelStop();
    markStageDone("Swivel fire complete (90 deg).");
  }
}

// ---- Stage 11: drive until ultrasonic near wall ----
static void stageDriveUntilUsNear() {
  if (stageDone) { stageIdleBlink(); return; }

  static bool started = false;
  if (!started) {
    started = true;
    Serial.println("[STAGE11] Drive forward until ultrasonic near wall.");
    ultrasonicReset();
  }

  ultrasonicUpdate();
  safeMotorsTank(SPEED_FWD, SPEED_FWD);

  if (testUsNearWall()) {
    markStageDone("Near wall detected.");
  }
}

// ---- Stage 12: blahblah ----

static const int16_t SPEED_FWD_ESCAPE   = 100; 
static const int16_t SPEED_BACKUP       = -100;
static const int16_t SPEED_TURN_ESCAPE  = 175;

// The Nuclear Option for breaking stiction
static const int16_t SPEED_ALIGN        = 175; 
static const int16_t SPEED_CREEP        = 150;

static const uint32_t BACKUP_MS  = 400;
static const uint32_t TURN_AWAY_MS = 350; 

// Micro-stepping timings
static const uint32_t ALIGN_PULSE_MS = 30; 
static const uint32_t ALIGN_BRAKE_MS = 40; 

// Exit distance threshold
static const float US_EXIT_THRESHOLD_CM = 140.0f; 

enum EscState {
  ESC_SCAN_SPIN = 0,         
  ESC_DRIVE_FWD = 1,
  ESC_ALIGN_TAPE = 2,
  ESC_VERIFY_PAUSE = 3,
  ESC_VERIFY_US = 4,
  ESC_REJECT_LINE_BACKUP = 5,
  ESC_REJECT_LINE_TURN = 6,
  ESC_EXIT_CROSSING = 7,
  ESC_STUCK_RECOVERY = 8     // NEW: Watchdog recovery state
};

static EscState sEscState = ESC_SCAN_SPIN; // Start by scanning
static SimpleTimer sEscStateTimer;
static SimpleTimer sEscDebugTimer; 
static SimpleTimer sEscBufferTimer; 
static SimpleTimer sEscScanTimer; 

// NEW: Stuck detection watchdog timers and trackers
static SimpleTimer sEscStuckTimer;
static float stuckRefCm = -1.0f;

// Memory variable to track the last known turn direction
static int8_t lastAlignDir = 0; 

static void stageEscapeBoxUsThenCross() {
  if (stageDone) { stageIdleBlink(); return; }

  ultrasonicUpdate();
  if (!usValid) { safeMotorsStop(); return; }

  // ==========================================
  // STUCK DETECTION WATCHDOG
  // ==========================================
  // Don't run the watchdog if we are already recovering or crossing the exit
  if (sEscState != ESC_STUCK_RECOVERY && sEscState != ESC_EXIT_CROSSING) {
    if (usDistanceCm < US_EXIT_THRESHOLD_CM) {
      // Calculate absolute difference without requiring <math.h>
      float diff = usDistanceCm - stuckRefCm;
      if (diff < 0) diff = -diff;

      if (diff <= 0.2f) {
        if (!sEscStuckTimer.running) sEscStuckTimer.start(2000); // 2 second timer
        
        if (sEscStuckTimer.expired()) {
          Serial.println("[STAGE12] WATCHDOG: Corner Jam Detected! Executing 1s reverse.");
          sEscStateTimer.start(1000); // Reverse for 1 second
          sEscState = ESC_STUCK_RECOVERY;
          sEscStuckTimer.stop();
        }
      } else {
        // Distance changed, reset the watchdog
        stuckRefCm = usDistanceCm;
        sEscStuckTimer.stop();
      }
    } else {
      // We see open space, reset the watchdog
      stuckRefCm = -1.0f;
      sEscStuckTimer.stop();
    }
  }

  uint8_t tapeBits = readTapeBits();

  bool leftOuter  = (tapeBits & TAPE_LO);
  bool leftInner  = (tapeBits & TAPE_LI);
  bool rightOuter = (tapeBits & TAPE_RO);

  uint8_t hitCount = 0;
  if (leftOuter)  hitCount++;
  if (leftInner)  hitCount++;
  if (rightOuter) hitCount++;

  bool lockedOn = (hitCount >= 2);
  bool anyHit   = (hitCount > 0);

  // --- VERBOSE SENSOR DEBUGGING ---
  if (!sEscDebugTimer.running) sEscDebugTimer.start(250);
  if (sEscDebugTimer.expired()) {
    Serial.print("[DEBUG] State: "); Serial.print((int)sEscState);
    Serial.print(" | US: "); Serial.print(usDistanceCm);
    Serial.print(" | Tape: "); 
    Serial.print(leftOuter ? "1" : "0"); Serial.print("/");
    Serial.print(leftInner ? "1" : "0"); Serial.print("/");
    Serial.print(rightOuter ? "1" : "0");
    Serial.print(" | Hits: "); Serial.print(hitCount);
    Serial.print(" | MemDir: "); Serial.print(lastAlignDir);
    Serial.print(" | Lock: "); Serial.println(lockedOn ? "YES" : "NO");
    sEscDebugTimer.start(250); 
  }
  // --------------------------------

  switch(sEscState) {
    case ESC_SCAN_SPIN:
      safeMotorsTank(SPEED_TURN_ESCAPE, -SPEED_TURN_ESCAPE); 

      if (usDistanceCm < US_EXIT_THRESHOLD_CM) {
        sEscScanTimer.start(300); 
      } 
      else if (!sEscScanTimer.running) {
        sEscScanTimer.start(300);
      }
      else if (sEscScanTimer.expired()) {
        Serial.println("[STAGE12] Consistent open path found! Driving forward.");
        sEscState = ESC_DRIVE_FWD;
      }
      break;

    case ESC_DRIVE_FWD:
      safeMotorsTank(SPEED_FWD_ESCAPE, SPEED_FWD_ESCAPE);

      if (anyHit) {
        lastAlignDir = 0; 
        sEscState = ESC_ALIGN_TAPE;
      } 
      else if (usDistanceCm > 0.0f && usDistanceCm < 15.0f) {
        Serial.println("[STAGE12] Wall bump! Backing up.");
        sEscStateTimer.start(BACKUP_MS);
        sEscState = ESC_REJECT_LINE_BACKUP;
      }
      break;

    case ESC_ALIGN_TAPE:
      if (lockedOn) {
        safeMotorsStop();
        sEscStateTimer.start(250);
        sEscState = ESC_VERIFY_PAUSE;
      }
      else if (!sEscBufferTimer.running || sEscBufferTimer.expired()) {
        sEscBufferTimer.start(ALIGN_PULSE_MS + ALIGN_BRAKE_MS);
        
        if (leftOuter || leftInner) {
          safeMotorsTank(-SPEED_ALIGN, SPEED_ALIGN); 
          lastAlignDir = -1; 
        }
        else if (rightOuter) {
          safeMotorsTank(SPEED_ALIGN, -SPEED_ALIGN); 
          lastAlignDir = 1; 
        }
        else {
          if (lastAlignDir == -1) {
            safeMotorsTank(SPEED_CREEP, SPEED_ALIGN); 
          } 
          else if (lastAlignDir == 1) {
            safeMotorsTank(SPEED_ALIGN, SPEED_CREEP); 
          } 
          else {
            safeMotorsTank(SPEED_CREEP, SPEED_CREEP); 
          }
        }
      }
      else if ((millis() - sEscBufferTimer.t0) > ALIGN_PULSE_MS) {
        safeMotorsStop();
      }
      break;

    case ESC_VERIFY_PAUSE:
      safeMotorsStop();
      if (!lockedOn) {
        sEscState = ESC_ALIGN_TAPE;
      }
      else if (sEscStateTimer.expired()) {
        sEscState = ESC_VERIFY_US;
      }
      break;

    case ESC_VERIFY_US:
      safeMotorsStop();

      if (!lockedOn) {
        sEscState = ESC_ALIGN_TAPE;
        break;
      }

      if (usDistanceCm >= US_EXIT_THRESHOLD_CM) {
        Serial.println("[STAGE12] >>> EXIT CONFIRMED <<< Driving forward 3s.");
        sEscStateTimer.start(3000); 
        sEscState = ESC_EXIT_CROSSING;
      } else {
        Serial.println("[STAGE12] Rejecting line (Wall ahead).");
        sEscStateTimer.start(BACKUP_MS);
        sEscState = ESC_REJECT_LINE_BACKUP;
      }
      break;

    case ESC_EXIT_CROSSING:
      safeMotorsTank(SPEED_FWD_ESCAPE, SPEED_FWD_ESCAPE);
      if (sEscStateTimer.expired()) {
        safeMotorsStop();
        markStageDone("Escape Verified and cleared line.");
      }
      break;

    case ESC_REJECT_LINE_BACKUP:
      safeMotorsTank(SPEED_BACKUP, SPEED_BACKUP);
      if (sEscStateTimer.expired()) {
        sEscStateTimer.start(TURN_AWAY_MS);
        sEscState = ESC_REJECT_LINE_TURN;
      }
      break;

    case ESC_REJECT_LINE_TURN:
      safeMotorsTank(SPEED_TURN_ESCAPE, -SPEED_TURN_ESCAPE);
      if (sEscStateTimer.expired()) {
        sEscState = ESC_SCAN_SPIN;
      }
      break;

    // ==========================================
    // NEW WATCHDOG RECOVERY STATE
    // ==========================================
    case ESC_STUCK_RECOVERY:
      safeMotorsTank(SPEED_BACKUP, SPEED_BACKUP);
      if (sEscStateTimer.expired()) {
        // Reset variables and resume normal scanning
        stuckRefCm = -1.0f;
        sEscState = ESC_SCAN_SPIN;
      }
      break;
  }
}

// ---- Stage 14: Return to Start ----

static void stageReturnToStart() {
  if (stageDone) { stageIdleBlink(); return; }

  // Ensure ultrasonic data is fresh if running as an isolated stage
  ultrasonicUpdate();

  if (!retStarted) {
    retStarted = true;
    sRetState = RET_INIT;
  }

  switch (sRetState) {
    case RET_INIT:
      Serial.println("[STAGE 14] Init: Turning 180.");
      sRetTimer.start((uint32_t)(180.0f * MS_PER_DEG_ROBOT));
      sRetState = RET_TURN_180;
      break;

    case RET_TURN_180:
      safeMotorsTank(SPEED_TURN, -SPEED_TURN); // CW Turn
      if (sRetTimer.expired()) {
        safeMotorsStop();
        Serial.println("[STAGE 14] 180 done. Line following to 20cm from wall.");
        
        // NEW: Reset tape memory before starting the line follow
        lastTapeSide = 0; 
        
        // Brief pause to prevent motor current spikes on immediate reversal
        sRetTimer.start(200); 
        sRetState = RET_DRIVE_1;
      }
      break;

    case RET_DRIVE_1:
      if (sRetTimer.running && !sRetTimer.expired()) {
        safeMotorsStop(); // Waiting out the pause
      } else {
        
        // NEW: Use the line follower instead of blind forward drive
        lineFollowStep(); 
        
        if (usValid && usDistanceCm <= 20.0f) {
          safeMotorsStop();
          Serial.println("[STAGE 14] Wall 1 reached. Turning 90 CW.");
          sRetTimer.start((uint32_t)(90.0f * MS_PER_DEG_ROBOT));
          sRetState = RET_TURN_90_CW;
        }
      }
      break;

    case RET_TURN_90_CW:
      safeMotorsTank(SPEED_TURN, -SPEED_TURN); // CW Turn
      if (sRetTimer.expired()) {
        safeMotorsStop();
        Serial.println("[STAGE 14] 90 done. Driving to 23cm from wall.");
        sRetTimer.start(200); // Brief pause
        sRetState = RET_DRIVE_2;
      }
      break;

    case RET_DRIVE_2:
      if (sRetTimer.running && !sRetTimer.expired()) {
        safeMotorsStop(); // Waiting out the pause
      } else {
        // Kept intact: blind forward drive
        safeMotorsTank(SPEED_FWD, SPEED_FWD); 
        if (usValid && usDistanceCm <= 23.0f) {
          safeMotorsStop();
          Serial.println("[STAGE 14] Wall 2 reached. Turning 90 CW.");
          
          sRetTimer.start((uint32_t)(90.0f * MS_PER_DEG_ROBOT)); 
          sRetState = RET_TURN_90_CW2;
        }
      }
      break;

    case RET_TURN_90_CW2:
      safeMotorsTank(SPEED_TURN, -SPEED_TURN); // CW Turn
      if (sRetTimer.expired()) {
        safeMotorsStop();
        Serial.println("[STAGE 14] Final 90 done. Reversing for 2 seconds.");
        
        // Start the 2-second reverse timer
        sRetTimer.start(2000); 
        sRetState = RET_REVERSE_FINAL;
      }
      break;

    case RET_REVERSE_FINAL:
      // Drive backward using the standard forward speed, inverted
      safeMotorsTank(-SPEED_FWD, -SPEED_FWD); 
      
      if (sRetTimer.expired()) {
        safeMotorsStop();
        sRetState = RET_DONE;
        markStageDone("Returned to start and backed into the box.");
      }
      break;

    case RET_DONE:
      safeMotorsStop();
      break;
  }
}

// ===================== Arduino setup/loop ====================
void setup() {
  // pinMode(PIN_ENABLE_BTN, INPUT_PULLDOWN);
  statusLedInit();

  pinMode(PIN_TAPE_LO, INPUT);
  pinMode(PIN_TAPE_LI, INPUT);
  pinMode(PIN_TAPE_RI, INPUT);
  pinMode(PIN_TAPE_RO, INPUT);

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  Serial.begin(115200);
  Serial.println("[NAV] boot");

  analogReference(DEFAULT);

  pinMode(PIN_L_ENA, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);

  pinMode(PIN_R_ENA, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);

  // Stepper enable + speed (same style as your test)
  pinMode(PIN_SWIVEL_EN, OUTPUT);
  digitalWrite(PIN_SWIVEL_EN, HIGH);
  myStepper.setSpeed(15);

  safeMotorsStop();
  stageDone = false;
  ultrasonicReset();
  lastTapeSide = 0;

  sExitDeb.stop();
  sExitTimeout.stop();
  sTurn.stop();
  sTurnStarted = false;


    // reset Stage 12 state
  sEscState = ESC_DRIVE_FWD;
  

  // reset escape test state
  // escPhase = ESC_INIT;

  fullFsmResetRun();

  Serial.print("[NAV] STAGE_MODE="); Serial.print(STAGE_MODE);
  Serial.print(" ("); Serial.print(stageName(STAGE_MODE)); Serial.println(")");
  Serial.println("[NAV] ready");
}

static int readAnalogSettled(uint8_t pin) {
  (void)analogRead(pin);
  delayMicroseconds(200);
  (void)analogRead(pin);
  delayMicroseconds(200);
  return analogRead(pin);
}

void loop() {
  switch (STAGE_MODE) {
    case 0:  fullFsmLoop();                break;
    case 1:  stageTapeMonitorRaw();        break;
    case 2:  stageUltrasonicMonitor();     break;
    case 3:  stageExitUntilCross();        break;
    case 4:  stageFollowUntilHog();        break;
    case 5:  stageTurnTimed(90.0f);        break;
    case 6:  stageTurnTimed(180.0f);       break;
    case 7:  stageMotorsConstantForward(); break;
    case 8:  stageMotorToggleDemo();       break;
    case 9:  stageSwivelMotorTest();       break;
    case 10: stageSwivelFireOnce();        break;
    case 11: stageDriveUntilUsNear();      break;
    case 12: stageEscapeBoxUsThenCross();  break;
    case 14: stageReturnToStart();         break;

    default:
      safeMotorsStop();
      statusLedWrite(true);
      break;
  }
}