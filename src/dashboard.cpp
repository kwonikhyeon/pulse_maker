#include "dashboard.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

namespace {

constexpr int BOX_WIDTH = 78;
constexpr int RIGHT_COL = BOX_WIDTH;
constexpr int BAR_WIDTH = 24;
constexpr int WAVE_WIDTH = 50;

bool g_firstRender = true;

inline void cursorHome()   { Serial.print("\033[H"); }
inline void hideCursor()   { Serial.print("\033[?25l"); }
inline void clearScreen()  { Serial.print("\033[2J"); }
inline void clearLineEnd() { Serial.print("\033[K"); }
inline void resetStyle()   { Serial.print("\033[0m"); }
inline void rowStart()     { Serial.print("|"); }
inline void rowEnd()       { Serial.printf("\033[%dG|\033[K\r\n", RIGHT_COL); }

void boxRule(char fill) {
  Serial.print("+");
  for (int i = 0; i < BOX_WIDTH - 2; i++) Serial.print(fill);
  Serial.print("+");
  clearLineEnd();
  Serial.print("\r\n");
}

void formatUptime(char *buf, size_t bufLen, uint32_t ms) {
  uint32_t s = ms / 1000;
  uint32_t m = s / 60;
  uint32_t h = m / 60;
  m %= 60;
  s %= 60;
  snprintf(buf, bufLen, "%02lu:%02lu:%02lu",
           (unsigned long)h, (unsigned long)m, (unsigned long)s);
}

void renderBar(int value, int maxValue, int width) {
  int filled = (value * width + maxValue / 2) / maxValue;
  if (filled < 0) filled = 0;
  if (filled > width) filled = width;

  Serial.print("[");
  if (filled > 0) {
    if (filled > (width * 2) / 3)      Serial.print("\033[33m");
    else if (filled > width / 3)       Serial.print("\033[32m");
    else                               Serial.print("\033[36m");
    for (int i = 0; i < filled; i++) Serial.print("#");
    resetStyle();
  }
  for (int i = filled; i < width; i++) Serial.print(".");
  Serial.print("]");
}

void renderEnableTag(bool enabled) {
  if (enabled) Serial.print("\033[1;32m[ ON  ]\033[0m");
  else         Serial.print("\033[1;31m[ OFF ]\033[0m");
}

void renderWatchdogTag(bool tripped) {
  if (tripped) Serial.print("\033[1;33mTRIPPED\033[0m");
  else         Serial.print("\033[32m  ok   \033[0m");
}

void renderHeartbeatTag(bool seen, bool alive) {
  if (!seen)      Serial.print("\033[33mwaiting\033[0m");
  else if (alive) Serial.print("\033[32m alive \033[0m");
  else            Serial.print("\033[31m  lost \033[0m");
}

void renderDutyError(float commanded, float measured, bool measurementValid) {
  if (!measurementValid) {
    Serial.print("\033[90m( ----- )\033[0m");
    return;
  }
  const float err = measured - commanded;
  const float absErr = fabsf(err);
  if (absErr > 2.0f)      Serial.print("\033[31m");
  else if (absErr > 0.5f) Serial.print("\033[33m");
  else                    Serial.print("\033[32m");
  Serial.printf("(%+6.2f%%)", err);
  resetStyle();
}

void renderTitle(const DashboardState &s) {
  char uptime[16];
  formatUptime(uptime, sizeof(uptime), s.uptimeMs);
  rowStart();
  Serial.print("  \033[1;36mTEENSY 4.1 PULSE MAKER\033[0m");
  Serial.printf("\033[%dG", RIGHT_COL - 22);
  Serial.printf("uptime  \033[1m%s\033[0m", uptime);
  rowEnd();
}

void renderCanHeader(const DashboardState &s) {
  rowStart();
  Serial.print("  \033[1mCAN CONTROL\033[0m");
  Serial.printf("\033[%dG", RIGHT_COL - 32);
  Serial.printf("ID 0x%03lX | %lu bps",
                (unsigned long)s.canControlId,
                (unsigned long)s.canBaudRate);
  rowEnd();
}

void renderCanStatusRow(const DashboardState &s) {
  rowStart();
  Serial.print("   PWM ");
  renderEnableTag(s.pwmEnabled);
  Serial.print("   watchdog ");
  renderWatchdogTag(s.canWatchdogTriggered);
  Serial.print("   pc heartbeat ");
  renderHeartbeatTag(s.pcHeartbeatSeen, s.pcHeartbeatAlive);
  rowEnd();
}

void renderCommandRow(const DashboardState &s) {
  rowStart();
  if (s.hasReceivedCommand) {
    Serial.printf("   last cmd %-7s   valid %lu   invalid %lu   age %lu ms",
                  s.lastCanCommandName,
                  (unsigned long)s.canCommandCount,
                  (unsigned long)s.canInvalidCommandCount,
                  (unsigned long)s.lastCanCommandAgeMs);
  } else {
    Serial.printf("   last cmd -         valid 0       invalid %lu       age -",
                  (unsigned long)s.canInvalidCommandCount);
  }
  rowEnd();
}

void renderHeartbeatRow(const DashboardState &s) {
  rowStart();
  Serial.printf("   PC  0x%03lX rx %lu",
                (unsigned long)s.pcHeartbeatId,
                (unsigned long)s.pcHeartbeatCount);
  if (s.pcHeartbeatSeen) {
    Serial.printf(" (%lums ago)", (unsigned long)s.lastPcHeartbeatAgeMs);
  } else {
    Serial.print(" (no rx yet)");
  }
  Serial.printf("   TEENSY 0x%03lX tx %lu @ %lums   wd %lums",
                (unsigned long)s.teensyHeartbeatId,
                (unsigned long)s.teensyHeartbeatCount,
                (unsigned long)s.heartbeatTxIntervalMs,
                (unsigned long)s.canWatchdogTimeoutMs);
  rowEnd();
}

void renderCommandedHeader(const DashboardState &s) {
  rowStart();
  Serial.printf("  \033[1mPWM OUTPUT\033[0m  (commanded, %.0f Hz)", s.pwmFrequencyHz);
  rowEnd();
}

void renderCommandedRow(const DashboardChannel &ch) {
  rowStart();
  Serial.printf("   %-2s -> PWM%-2u   ADC %4d (%4.2f V)   duty %6.2f %%   ",
                ch.label,
                (unsigned)ch.pwmPin,
                ch.adcRaw,
                ch.voltage,
                ch.commandedDutyPercent);
  renderBar(ch.pwmOutput, ch.pwmMax, BAR_WIDTH);
  rowEnd();
}

void renderMeasuredHeader() {
  rowStart();
  Serial.print("  \033[1mPWM MEASURED\033[0m  (jumper feedback, delta vs commanded)");
  rowEnd();
}

void renderMeasuredRow(const DashboardChannel &ch) {
  rowStart();
  if (!ch.measurement.valid) {
    Serial.printf("   PWM%-2u  \033[90mno valid pulses\033[0m  (%s)",
                  (unsigned)ch.measurePin,
                  ch.inputHigh ? "steady HIGH" : "steady LOW ");
    rowEnd();
    return;
  }
  const float frequencyHz = 1000000.0f / (float)ch.measurement.periodUs;
  const float measuredDuty = (float)ch.measurement.highUs * 100.0f /
                             (float)ch.measurement.periodUs;
  Serial.printf("   PWM%-2u  %7.2f Hz  high %4lu us  period %4lu us  duty %5.2f%% ",
                (unsigned)ch.measurePin,
                frequencyHz,
                (unsigned long)ch.measurement.highUs,
                (unsigned long)ch.measurement.periodUs,
                measuredDuty);
  renderDutyError(ch.commandedDutyPercent, measuredDuty, true);
  rowEnd();
}

void renderWaveformHeader() {
  rowStart();
  Serial.print("  \033[1mWAVEFORM\033[0m");
  rowEnd();
}

void renderWaveformRow(const DashboardChannel &ch) {
  rowStart();
  Serial.printf("   PWM%-2u  ", (unsigned)ch.measurePin);

  if (!ch.measurement.valid) {
    Serial.print(ch.inputHigh ? "\033[31m" : "\033[36m");
    for (int i = 0; i < WAVE_WIDTH + 3; i++) {
      Serial.print(ch.inputHigh ? "-" : "_");
    }
    resetStyle();
    rowEnd();
    return;
  }

  int high = (int)((float)ch.measurement.highUs * WAVE_WIDTH /
                   (float)ch.measurement.periodUs + 0.5f);
  if (high < 0) high = 0;
  if (high > WAVE_WIDTH) high = WAVE_WIDTH;

  Serial.print("|\033[32m");
  for (int i = 0; i < high; i++) Serial.print("-");
  resetStyle();
  Serial.print("|");
  for (int i = high; i < WAVE_WIDTH; i++) Serial.print("_");
  Serial.print("|");
  rowEnd();
}

}  // namespace

namespace dashboard {

void render(const DashboardState &state) {
  if (g_firstRender) {
    hideCursor();
    clearScreen();
    g_firstRender = false;
  }
  cursorHome();

  boxRule('=');
  renderTitle(state);
  boxRule('-');

  renderCanHeader(state);
  renderCanStatusRow(state);
  renderCommandRow(state);
  renderHeartbeatRow(state);
  boxRule('-');

  renderCommandedHeader(state);
  renderCommandedRow(state.ch1);
  renderCommandedRow(state.ch2);
  boxRule('-');

  renderMeasuredHeader();
  renderMeasuredRow(state.ch1);
  renderMeasuredRow(state.ch2);
  boxRule('-');

  renderWaveformHeader();
  renderWaveformRow(state.ch1);
  renderWaveformRow(state.ch2);
  boxRule('=');
}

}  // namespace dashboard
