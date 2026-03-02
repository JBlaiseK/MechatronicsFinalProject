#pragma once
/*
  ProtoLink.h — tiny ASCII line protocol for NAV <-> SHOOTER

  Lines are '\n' terminated. Examples:
    NAV -> SHOOTER:  FIRE 12 3 0
    SHOOTER -> NAV: ACK 12
    SHOOTER -> NAV: DONE 12 3

  Fields:
    seq   : monotonically increasing command id (uint16_t)
    shots : number of shots requested (uint8_t)
    dist  : optional distance metric (uint16_t, can be 0 for now)
    fired : number of shots actually fired (uint8_t)
*/

#include <Arduino.h>

namespace Proto {

  static const uint16_t LINE_MAX = 80;

  // ---------- Line reader ----------
  // Non-blocking: call often; returns true once a full line is available.
  inline bool readLine(Stream& s, char* out, size_t outN) {
    static size_t idx = 0;
    while (s.available()) {
      char c = (char)s.read();
      if (c == '\r') continue;
      if (c == '\n') {
        out[idx] = '\0';
        idx = 0;
        return true;
      }
      if (idx + 1 < outN) out[idx++] = c;  // keep space for '\0'
      // else: overflow; silently drop extra chars (TODO: add error flag)
    }
    return false;
  }

  // ---------- Message structs ----------
  struct FireCmd {
    uint16_t seq = 0;
    uint8_t  shots = 0;
    uint16_t dist = 0;
  };

  struct AckMsg {
    uint16_t seq = 0;
  };

  struct DoneMsg {
    uint16_t seq = 0;
    uint8_t fired = 0;
  };

  // ---------- Parsers ----------
  inline bool parseFire(const char* line, FireCmd& out) {
    // Expected: "FIRE <seq> <shots> <dist>"
    unsigned int seq, shots, dist;
    int n = sscanf(line, "FIRE %u %u %u", &seq, &shots, &dist);
    if (n != 3) return false;
    out.seq = (uint16_t)seq;
    out.shots = (uint8_t)shots;
    out.dist = (uint16_t)dist;
    return true;
  }

  inline bool parseAck(const char* line, AckMsg& out) {
    unsigned int seq;
    int n = sscanf(line, "ACK %u", &seq);
    if (n != 1) return false;
    out.seq = (uint16_t)seq;
    return true;
  }

  inline bool parseDone(const char* line, DoneMsg& out) {
    unsigned int seq, fired;
    int n = sscanf(line, "DONE %u %u", &seq, &fired);
    if (n != 2) return false;
    out.seq = (uint16_t)seq;
    out.fired = (uint8_t)fired;
    return true;
  }

  // ---------- Formatters ----------
  inline void sendFire(Stream& s, uint16_t seq, uint8_t shots, uint16_t dist) {
    s.print("FIRE "); s.print(seq); s.print(' ');
    s.print(shots); s.print(' ');
    s.print(dist); s.print('\n');
  }

  inline void sendAck(Stream& s, uint16_t seq) {
    s.print("ACK "); s.print(seq); s.print('\n');
  }

  inline void sendDone(Stream& s, uint16_t seq, uint8_t fired) {
    s.print("DONE "); s.print(seq); s.print(' ');
    s.print(fired); s.print('\n');
  }

} // namespace Proto
