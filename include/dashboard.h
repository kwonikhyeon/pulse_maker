#pragma once

#include <stdint.h>

struct PulseMeasurement {
  uint32_t highUs = 0;
  uint32_t periodUs = 0;
  bool valid = false;
};

struct DashboardChannel {
  const char *label;
  uint8_t pwmPin;
  uint8_t measurePin;
  int adcRaw;
  int adcMax;
  float voltage;
  float commandedFrequencyHz;
  float minFrequencyHz;
  float maxFrequencyHz;
  int pwmOutput;
  int pwmMax;
  float commandedDutyPercent;
  PulseMeasurement measurement;
  bool inputHigh;
};

struct DashboardState {
  uint32_t uptimeMs;
  bool pwmEnabled;

  DashboardChannel ch1;
  DashboardChannel ch2;

  uint32_t canControlId;
  uint32_t canBaudRate;
  uint32_t pcHeartbeatId;
  uint32_t teensyHeartbeatId;
  uint8_t heartbeatMagic;
  uint32_t heartbeatTxIntervalMs;

  const char *lastCanCommandName;
  bool hasReceivedCommand;
  uint32_t canCommandCount;
  uint32_t canInvalidCommandCount;
  uint32_t lastCanCommandAgeMs;

  bool canWatchdogTriggered;
  uint32_t canWatchdogTimeoutMs;

  uint32_t pcHeartbeatCount;
  uint32_t teensyHeartbeatCount;
  bool pcHeartbeatSeen;
  bool pcHeartbeatAlive;
  uint32_t lastPcHeartbeatAgeMs;
};

namespace dashboard {

void render(const DashboardState &state);

}
