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
static const uint8_t STAGE_MODE = 2;

// Safety gate: robot will not move unless true.
static const bool TEST_ENABLE_MOTORS = true;

// ========================= Pin Map ===========================

// Tape sensors: outer-left, inner-left, inner-right, outer-right
static const uint8_t PIN_TAPE_LO = A2;
static const uint8_t PIN_TAPE_LI = A5;
static const uint8_t PIN_TAPE_RI = A4;
static const uint8_t PIN_TAPE_RO = A1;

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

static const int16_t SPEED_FWD    = 150;
static const int16_t SPEED_TURN   = 250;  // used for timed turns
static const int16_t SPEED_SEARCH = 55;
static const int16_t SPEED_DELTA  = 35;

// Robot turn calibration ONLY (timed turns)
static const float MS_PER_DEG_ROBOT = 5.0f;

// Tape crossing debounce
static const uint32_t EXIT_CROSS_DEBOUNCE_MS = 500;
static const uint32_t HOGLINE_DEBOUNCE_MS    = 120;

// Exit box fallback timeout
static const uint32_t EXIT_TIMEOUT_MS = 3000;

// Ultrasonic
static const uint32_t US_PING_INTERVAL_MS = 60;
static const uint32_t US_ECHO_TIMEOUT_US  = 25000;
static const float    US_CM_PER_US        = 0.0343f / 2.0f;

// Hysteresis / debounce
static const float    US_NEAR_WALL_CM  = 12.0f;
static const float    US_CLEAR_WALL_CM = 25.0f;
static const uint8_t  US_NEAR_N        = 3;
static const uint8_t  US_CLEAR_N       = 3;

// ===== New “turn-until-clear + backup” tuning knobs =====
static const float    US_TOO_CLOSE_CM  = 6.0f;   // "stuck / corner" threshold
static const uint32_t BACKUP_TIME_MS   = 350;    // backup duration when too close
static const int16_t  SPEED_TURN_SCAN  = 140;    // turning speed while scanning for clear path

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
  if (lo > TAPE_THRESH_LO) bits |= TAPE_LO;
  if (li > TAPE_THRESH_LI) bits |= TAPE_LI;
  if (ri > TAPE_THRESH_RI) bits |= TAPE_RI;
  if (ro > TAPE_THRESH_RO) bits |= TAPE_RO;

  return bits;
}

static bool isTapeCrossing(uint8_t bits) {
  bool li = bits & TAPE_LI;
  bool ri = bits & TAPE_RI;
  bool lo = bits & TAPE_LO;
  bool ro = bits & TAPE_RO;

  uint8_t count = 0;
  if (lo) count++;
  if (li) count++;
  if (ri) count++;
  if (ro) count++;

  // Serial.print(lo, li, ri, ro);

  Serial.print(li && ri || (count >= 3));

  return (li && ri) || (count >= 3);
}

static bool crossingDebounced(SimpleTimer &deb, uint32_t debounceMs) {
  uint8_t bits = readTapeBits();
  if (isTapeCrossing(bits)) {
    Serial.println("We have detected crossing the tape!");
    if (!deb.running) deb.start(debounceMs);
    return deb.expired();
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
  if (duration == 0) return -1.0f;

  return (float)duration * US_CM_PER_US;
}

static void ultrasonicUpdate() {
  uint32_t now = millis();
  if ((uint32_t)(now - usLastPingMs) < US_PING_INTERVAL_MS) return;
  usLastPingMs = now;

  float cm = ultrasonicReadOnceCm();
  if (cm <= 0.0f) { usValid = false; return; }

  usValid = true;

  if (usDistanceCm <= 0.0f) usDistanceCm = cm;
  else usDistanceCm = 0.6f * usDistanceCm + 0.4f * cm;

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

// ========================== FULL FSM =========================
typedef enum {
  F_IDLE_WAIT_ENABLE,

  // UPDATED escape logic:
  F_TURN_UNTIL_CLEAR,
  F_BACKUP_FROM_WALL,

  F_EXIT_BOX_UNTIL_CROSS,
  F_LINE_FOLLOW_TO_HOG,
  F_FIRE_SWIVEL,
  F_TURN_180_AFTER_FIRE,
  F_DRIVE_TO_WALL,
  F_DONE
} FullState;

static FullState fullState = F_IDLE_WAIT_ENABLE;

static SimpleTimer fExitDeb;
static SimpleTimer fExitTimeout;
static SimpleTimer fHogDeb;
static SimpleTimer fTurnTimer;
static SimpleTimer fBackupTimer;

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
  fBackupTimer.stop();

  ultrasonicReset();
  lastTapeSide = 0;

  safeMotorsStop();
}

static void startTimedTurnRight(float deg) {
  uint32_t ms = (uint32_t)(deg * MS_PER_DEG_ROBOT);
  safeMotorsTank(SPEED_TURN, -SPEED_TURN);
  fTurnTimer.start(ms);
}

static void fullFsmLoop() {
  ultrasonicUpdate();

  switch (fullState) {
    case F_IDLE_WAIT_ENABLE:
      safeMotorsStop();
      if (readEnableButtonPressedEdge()) {
        Serial.println("[FULL] enable -> TURN_UNTIL_CLEAR");
        ultrasonicReset();
        fullState = F_TURN_UNTIL_CLEAR;
      }
      break;

    case F_TURN_UNTIL_CLEAR: {
      // Need at least one good reading before we trust it
      if (!usValid) {
        safeMotorsStop();
        break;
      }

      // Too close: back up first, then resume turning
      if (usDistanceCm > 0.0f && usDistanceCm <= US_TOO_CLOSE_CM) {
        Serial.print("[FULL] too close (cm="); Serial.print(usDistanceCm);
        Serial.println(") -> BACKUP_FROM_WALL");
        fBackupTimer.start(BACKUP_TIME_MS);
        fullState = F_BACKUP_FROM_WALL;
        break;
      }

      // If wall is near, keep turning until clear
      if (testUsNearWall()) {
        safeMotorsTank(SPEED_TURN_SCAN, -SPEED_TURN_SCAN); // right turn scan (verify)
      } else {
        // Clear ahead -> drive out until exit tape crossing
        safeMotorsStop();
        Serial.println("[FULL] clear -> EXIT_BOX_UNTIL_CROSS");
        fExitDeb.stop();
        fExitTimeout.start(EXIT_TIMEOUT_MS);
        fullState = F_EXIT_BOX_UNTIL_CROSS;
      }
    } break;

    case F_BACKUP_FROM_WALL:
      // Reverse with a slight arc to help unstick from corners
      safeMotorsTank(-SPEED_FWD, -(SPEED_FWD / 2));

      if (fBackupTimer.expired()) {
        safeMotorsStop();
        ultrasonicReset(); // refresh after moving
        Serial.println("[FULL] backup done -> TURN_UNTIL_CLEAR");
        fullState = F_TURN_UNTIL_CLEAR;
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
        safeMotorsStop();
        Serial.println("[FULL] hogline crossing -> FIRE_SWIVEL");
        fullState = F_FIRE_SWIVEL;
      }
      break;

    case F_FIRE_SWIVEL:
      safeMotorsStop();
      Serial.println("[FULL] firing swivel (stepper +90) ...");
      myStepper.step(steps90Degrees);
      delay(1000);
      Serial.println("[FULL] fire done -> TURN_180_AFTER_FIRE");
      startTimedTurnRight(180.0f);
      fullState = F_TURN_180_AFTER_FIRE;
      break;

    case F_TURN_180_AFTER_FIRE:
      if (fTurnTimer.expired()) {
        safeMotorsStop();
        ultrasonicReset();
        Serial.println("[FULL] 180 done -> DRIVE_TO_WALL");
        fullState = F_DRIVE_TO_WALL;
      }
      break;

    case F_DRIVE_TO_WALL:
      safeMotorsTank(SPEED_FWD, SPEED_FWD);
      if (testUsNearWall()) {
        safeMotorsStop();
        Serial.println("[FULL] wall reached -> DONE");
        fullState = F_DONE;
      }
      break;

    case F_DONE:
    default:
      safeMotorsStop();
      break;
  }
}

// ============================================================
// ===================== Stage/Test Runner =====================
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
  bool cur = (digitalRead(PIN_ENABLE_BTN) == LOW);
  bool edge = cur && !prevBtn2;
  prevBtn2 = cur;
  return edge;
}

static void stageMotorToggleDemo() {
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
  delay(1000);
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

// ---- Stage 12: escape box using ultrasonic, then stop on exit crossing ----
// UPDATED per your request: turn until clear (continuous), and if too close back up then return to turning.
static SimpleTimer sEscExitDeb;
static SimpleTimer sEscExitTimeout;
static SimpleTimer sEscBackupTimer;

typedef enum {
  ESC_INIT,
  ESC_TURN_UNTIL_CLEAR,
  ESC_BACKUP,
  ESC_DRIVE_OUT,
  ESC_DONE
} EscapePhase;

static EscapePhase escPhase = ESC_INIT;

static void stageEscapeBoxUsThenCross() {
  if (stageDone) { stageIdleBlink(); return; }

  ultrasonicUpdate();

  switch (escPhase) {
    case ESC_INIT:
      Serial.println("[STAGE12] Escape box (smart): turn until clear, backup if too close, then drive to exit tape.");
      ultrasonicReset();
      sEscExitDeb.stop();
      sEscExitTimeout.start(EXIT_TIMEOUT_MS);
      sEscBackupTimer.stop();
      safeMotorsStop();
      escPhase = ESC_TURN_UNTIL_CLEAR;
      break;

    case ESC_TURN_UNTIL_CLEAR:
      // wait for a valid reading
      if (!usValid) {
        safeMotorsStop();
        break;
      }

      // Too close => backup
      if (usDistanceCm > 0.0f && usDistanceCm <= US_TOO_CLOSE_CM) {
        Serial.print("[STAGE12] too close (cm="); Serial.print(usDistanceCm);
        Serial.println(") -> BACKUP");
        sEscBackupTimer.start(BACKUP_TIME_MS);
        escPhase = ESC_BACKUP;
        break;
      }

      // Near wall => keep turning
      if (testUsNearWall()) {
        safeMotorsTank(SPEED_TURN_SCAN, -SPEED_TURN_SCAN);
      } else {
        Serial.println("[STAGE12] clear -> DRIVE_OUT");
        safeMotorsStop();
        sEscExitDeb.stop();
        escPhase = ESC_DRIVE_OUT;
      }
      break;

    case ESC_BACKUP:
      // reverse arc
      safeMotorsTank(-SPEED_FWD, -(SPEED_FWD / 2));

      if (sEscBackupTimer.expired()) {
        safeMotorsStop();
        ultrasonicReset();
        Serial.println("[STAGE12] backup done -> TURN_UNTIL_CLEAR");
        escPhase = ESC_TURN_UNTIL_CLEAR;
      }
      break;

    case ESC_DRIVE_OUT:
      safeMotorsTank(SPEED_FWD, SPEED_FWD);

      if (crossingDebounced(sEscExitDeb, EXIT_CROSS_DEBOUNCE_MS)) {
        safeMotorsStop();
        markStageDone("Escaped box: exit crossing detected.");
        escPhase = ESC_DONE;
        return;
      }

      if (sEscExitTimeout.expired()) {
        safeMotorsStop();
        markStageDone("Escape timeout: no exit crossing.");
        escPhase = ESC_DONE;
        return;
      }
      break;

    case ESC_DONE:
    default:
      safeMotorsStop();
      break;
  }
}

// ===================== Arduino setup/loop ====================
void setup() {
  pinMode(PIN_ENABLE_BTN, INPUT_PULLUP);
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
  myStepper.setSpeed(20);

  safeMotorsStop();
  stageDone = false;
  ultrasonicReset();
  lastTapeSide = 0;

  sExitDeb.stop();
  sExitTimeout.stop();
  sHogDeb.stop();
  sTurn.stop();
  sTurnStarted = false;

  // reset escape test state
  escPhase = ESC_INIT;

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

    default:
      safeMotorsStop();
      statusLedWrite(true);
      break;
  }
}