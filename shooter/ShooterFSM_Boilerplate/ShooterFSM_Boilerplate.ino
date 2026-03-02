/*
  ShooterFSM_Skeleton.ino — SHOOTER Arduino (slave)
  - Receives FIRE command
  - Loads chamber, pulls back, fires solenoid pulse, repeats N times
  - Sends DONE when finished

  IMPORTANT: Pins are MADE UP as requested. Change to match your wiring.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ProtoLink.h"

// ---------------- Pin Map -- SUBJECT TO CHANGE ----------------
static const uint8_t PIN_LINK_RX = 10; // SHOOTER receives on D10 (wired from NAV TX)
static const uint8_t PIN_LINK_TX = 11; // SHOOTER transmits on D11 (wired to   NAV RX)
SoftwareSerial link(PIN_LINK_RX, PIN_LINK_TX);

// Loader motor (ball+chain) — assume H-bridge or motor driver
static const uint8_t PIN_LOAD_PWM = 5;
static const uint8_t PIN_LOAD_DIR = 4;

// Pullback motor — assume H-bridge or motor driver
static const uint8_t PIN_PULL_PWM = 6;
static const uint8_t PIN_PULL_DIR = 7;

// Solenoid driver
static const uint8_t PIN_SOLENOID = 8;

// Optional limit switch for pullback (active-low)
static const uint8_t PIN_PULL_LIMIT = 3;

// Optional status LED
static const uint8_t PIN_STATUS_LED = 13;


// ---------------- COnstants ---------
static const uint8_t  LOAD_SPEED = 200;       // PWM 0–255, TODO tune
static const uint32_t LOAD_MS    = 900;       // TODO tune

static const uint8_t  PULL_SPEED = 220;       // TODO tune
static const uint32_t PULL_MS_MIN = 450;      // TODO tune
static const uint32_t PULL_MS_MAX = 1100;     // TODO tune

static const uint32_t SOLENOID_PULSE_MS = 90; // TODO tune
static const uint32_t POST_FIRE_SETTLE_MS = 200; // TODO tune


// ---------------- Simple Timer same from nav ----------------
struct SimpleTimer {
  uint32_t t0 = 0;
  uint32_t dur = 0;
  bool running = false;

  void start(uint32_t durationMs) { t0 = millis(); dur = durationMs; running = true; }
  void stop() { running = false; }
  bool expired() const { return running && (uint32_t)(millis() - t0) >= dur; }
};

// --------------------- Helpers --------------------------
static void loaderOff() { analogWrite(PIN_LOAD_PWM, 0); }
static void loaderOnForward() {
  digitalWrite(PIN_LOAD_DIR, HIGH);  // TODO: verify direction
  analogWrite(PIN_LOAD_PWM, LOAD_SPEED);
}


static void pullbackOff() { analogWrite(PIN_PULL_PWM, 0); }
static void pullbackOn() {
  digitalWrite(PIN_PULL_DIR, HIGH);  // TODO: verify direction
  analogWrite(PIN_PULL_PWM, PULL_SPEED);
}


static void solenoidOff() { digitalWrite(PIN_SOLENOID, LOW); }
static void solenoidOn() { digitalWrite(PIN_SOLENOID, HIGH); }


// ---------------- SHOOTER FSM ----------------
typedef enum {
  SHOOTER_IDLE,
  LOADING_CHAMBER,
  PULLING_BACK,
  FIRING_PULSE,
  POST_FIRE_SETTLE,
  SHOOTER_DONE
} ShooterState;

static ShooterState state = SHOOTER_IDLE;
static ShooterState next_state = SHOOTER_IDLE;
static ShooterState prev_state = (ShooterState)255;

static uint16_t activeSeq = 0;
static uint8_t shotsRequested = 0;
static uint8_t shotsLeft = 0;
static uint8_t shotsFired = 0;
static uint16_t distanceMetric = 0;

static SimpleTimer loadTimer;
static SimpleTimer pullTimer;
static SimpleTimer fireTimer;
static SimpleTimer settleTimer;

static char rxLine[Proto::LINE_MAX]; // for syn and ack for the other arduino


// ---------------- Events ----------------
static bool TestGotFireCmd(Proto::FireCmd& cmdOut) {
  if (!Proto::readLine(link, rxLine, sizeof(rxLine))) return false;

  if (!Proto::parseFire(rxLine, cmdOut)) {
    Serial.print("[SHOOTER] ??? "); Serial.println(rxLine);
    return false;
  }

  return true;
}

// we manually have to reload them
static bool TestLoadDone() { return loadTimer.expired(); }


static bool TestPullDone() {
  bool limitHit = (digitalRead(PIN_PULL_LIMIT) == LOW); // active-low
  return pullTimer.expired() || limitHit;
}


static bool TestFirePulseDone() { return fireTimer.expired(); }
static bool TestSettleDone() { return settleTimer.expired(); }

// ---------------- Responses to Events ------------
static void RespToFireCmd(const Proto::FireCmd& cmd) {
  activeSeq = cmd.seq;
  shotsRequested = cmd.shots;
  shotsLeft = cmd.shots;
  shotsFired = 0;
  distanceMetric = cmd.dist;

  Proto::sendAck(link, activeSeq); // ACK immediately -- we have retransmission logic if this does not properly work
  next_state = LOADING_CHAMBER;
}

static void RespLoadDone() { next_state = PULLING_BACK; }
static void RespPullDone() { next_state = FIRING_PULSE; }
static void RespFireDone() { next_state = POST_FIRE_SETTLE; }


static void RespSettleDone() {
  if (shotsLeft > 0) next_state = LOADING_CHAMBER;
  else next_state = SHOOTER_DONE;
}

static void RespDoneSent() { next_state = SHOOTER_IDLE; }

// ---------------- pullback distance metric -- idk if we want to do math or just set a timer but it is here --------
static uint32_t computePullbackMs(uint16_t dist) {

  // TODO: I made this up to map the distance to pullback to the time we have to pull it back -- needs to change...
  uint32_t ms = PULL_MS_MIN + (uint32_t)((uint32_t)dist * (PULL_MS_MAX - PULL_MS_MIN) / 1023UL);

  if (ms < PULL_MS_MIN) ms = PULL_MS_MIN;
  if (ms > PULL_MS_MAX) ms = PULL_MS_MAX;
  return ms;
}


// ------------------ SHOOTER FSM ------------------------

static void onEnter(ShooterState s) {
  switch (s) {
    case SHOOTER_IDLE:
      digitalWrite(PIN_STATUS_LED, LOW);
      loaderOff();
      pullbackOff();
      solenoidOff();
      break;

    case LOADING_CHAMBER:
      digitalWrite(PIN_STATUS_LED, HIGH);
      loaderOnForward();
      loadTimer.start(LOAD_MS);
      break;

    case PULLING_BACK: {
      loaderOff();
      pullbackOn();
      pullTimer.start(computePullbackMs(distanceMetric));
    } break;

    case FIRING_PULSE:
      pullbackOff();
      if (shotsLeft > 0) { shotsLeft--; shotsFired++; }
      solenoidOn();
      fireTimer.start(SOLENOID_PULSE_MS);
      break;

    case POST_FIRE_SETTLE:
      solenoidOff();
      settleTimer.start(POST_FIRE_SETTLE_MS);
      break;

    case SHOOTER_DONE:
      loaderOff();
      pullbackOff();
      solenoidOff();
      Proto::sendDone(link, activeSeq, shotsFired);
      break;
  }
}


// ---------------- Global event checking -----------------
static void checkGlobalEvents() {
  next_state = state;

  if (state == SHOOTER_IDLE) {
    Proto::FireCmd cmd;
    if (TestGotFireCmd(cmd)) RespToFireCmd(cmd);
  }

  if (state == LOADING_CHAMBER && TestLoadDone()) RespLoadDone();
  else if (state == PULLING_BACK && TestPullDone()) RespPullDone();
  else if (state == FIRING_PULSE && TestFirePulseDone()) RespFireDone();
  else if (state == POST_FIRE_SETTLE && TestSettleDone()) RespSettleDone();
  else if (state == SHOOTER_DONE) RespDoneSent();

  if (next_state != state) {
    prev_state = state;
    state = next_state;
    onEnter(state);
  }
}



// ---------------- Setup/loops --------------------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);

  pinMode(PIN_LOAD_PWM, OUTPUT);
  pinMode(PIN_LOAD_DIR, OUTPUT);

  pinMode(PIN_PULL_PWM, OUTPUT);
  pinMode(PIN_PULL_DIR, OUTPUT);

  pinMode(PIN_SOLENOID, OUTPUT);
  solenoidOff();

  pinMode(PIN_PULL_LIMIT, INPUT_PULLUP);

  Serial.begin(115200);
  link.begin(19200);

  Serial.println("[SHOOTER] boot");

  state = SHOOTER_IDLE;
  next_state = SHOOTER_IDLE;
  prev_state = (ShooterState)255;
  onEnter(state);
}

void loop() {
  checkGlobalEvents();
  // TODO: maybe some periodic debug events to figure out what is happening under the hood if this becomes too problematic...
}
