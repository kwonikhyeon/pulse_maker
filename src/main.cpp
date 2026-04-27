#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "adc_config.h"
#include "can_config.h"
#include "dashboard.h"
#include "pins.h"
#include "pwm_config.h"
#include "system_config.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
volatile bool pwmEnabled = false;
uint32_t lastCanCommandUs = 0;
uint32_t canCommandCount = 0;
uint32_t canInvalidCommandCount = 0;
uint8_t lastCanCommand = CAN_CMD_OFF;
bool canWatchdogTriggered = false;
uint32_t lastPcHeartbeatUs = 0;
uint32_t lastTeensyHeartbeatTxUs = 0;
uint32_t lastPulseTelemetryTxUs = 0;
uint32_t lastDashboardRenderMs = 0;
uint32_t pcHeartbeatCount = 0;
uint32_t teensyHeartbeatCount = 0;
uint8_t pulseTelemetrySequence = 0;

struct PulseCapture {
  volatile uint32_t lastRiseUs = 0;
  volatile uint32_t lastEdgeUs = 0;
  volatile uint32_t highUs = 0;
  volatile uint32_t periodUs = 0;
  volatile bool hasMeasurement = false;
  volatile bool seenRise = false;
};

PulseCapture pwm1Capture;
PulseCapture pwm2Capture;

const char *canCommandName(uint8_t command) {
  switch (command) {
    case CAN_CMD_OFF:
      return "OFF";
    case CAN_CMD_ON:
      return "ON";
    case CAN_CMD_TOGGLE:
      return "TOGGLE";
    default:
      return "UNKNOWN";
  }
}

void handleCanCommand(const CAN_message_t &message) {
  if (message.flags.extended ||
      message.flags.remote ||
      message.len < 1) {
    return;
  }

  if (message.id == PC_HEARTBEAT_ID) {
    if (message.buf[0] == HEARTBEAT_MAGIC) {
      lastPcHeartbeatUs = micros();
      pcHeartbeatCount++;
    }
    return;
  }

  if (message.id != CAN_CONTROL_ID) {
    return;
  }

  const uint8_t command = message.buf[0];

  switch (command) {
    case CAN_CMD_OFF:
      pwmEnabled = false;
      break;
    case CAN_CMD_ON:
      pwmEnabled = true;
      break;
    case CAN_CMD_TOGGLE:
      pwmEnabled = !pwmEnabled;
      break;
    default:
      canInvalidCommandCount++;
      return;
  }

  lastCanCommand = command;
  lastCanCommandUs = micros();
  lastPcHeartbeatUs = lastCanCommandUs;
  canCommandCount++;
  canWatchdogTriggered = false;
}

void readCanCommands() {
  CAN_message_t message;

  while (Can1.read(message)) {
    handleCanCommand(message);
  }
}

void sendTeensyHeartbeat() {
  const uint32_t nowUs = micros();
  if ((nowUs - lastTeensyHeartbeatTxUs) < HEARTBEAT_TX_INTERVAL_US) {
    return;
  }

  CAN_message_t message;
  message.id = TEENSY_HEARTBEAT_ID;
  message.flags.extended = 0;
  message.flags.remote = 0;
  message.len = 1;
  message.buf[0] = HEARTBEAT_MAGIC;

  Can1.write(message);
  lastTeensyHeartbeatTxUs = nowUs;
  teensyHeartbeatCount++;
}

uint16_t clampU16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : (uint16_t)value;
}

void putU16Le(uint8_t *buffer, uint8_t offset, uint16_t value) {
  buffer[offset] = (uint8_t)(value & 0xFFu);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFFu);
}

void sendPulseTelemetryFrame(uint32_t frameId,
                             uint8_t sequence,
                             int adcRaw,
                             const PulseMeasurement &measurement,
                             bool inputHigh) {
  uint8_t flags = 0;
  if (measurement.valid) flags |= 0x01;
  if (inputHigh) flags |= 0x02;
  if (pwmEnabled) flags |= 0x04;
  if (canWatchdogTriggered) flags |= 0x08;

  CAN_message_t message;
  message.id = frameId;
  message.flags.extended = 0;
  message.flags.remote = 0;
  message.len = 8;
  message.buf[0] = sequence;
  message.buf[1] = flags;
  putU16Le(message.buf, 2, clampU16((uint32_t)adcRaw));
  putU16Le(message.buf, 4, clampU16(measurement.highUs));
  putU16Le(message.buf, 6, clampU16(measurement.periodUs));

  Can1.write(message);
}

void sendPulseTelemetry(int pot1Raw, int pot2Raw,
                        const PulseMeasurement &m1,
                        const PulseMeasurement &m2) {
  const uint32_t nowUs = micros();
  if ((nowUs - lastPulseTelemetryTxUs) < PULSE_TELEMETRY_TX_INTERVAL_US) {
    return;
  }

  const uint8_t sequence = pulseTelemetrySequence++;
  sendPulseTelemetryFrame(PWM_1_TELEMETRY_ID, sequence, pot1Raw, m1,
                          digitalRead(PWM_1_MEASURE_PIN) == HIGH);
  sendPulseTelemetryFrame(PWM_2_TELEMETRY_ID, sequence, pot2Raw, m2,
                          digitalRead(PWM_2_MEASURE_PIN) == HIGH);
  lastPulseTelemetryTxUs = nowUs;
}

void updatePulseCapture(PulseCapture &capture, uint8_t pin) {
  const uint32_t nowUs = micros();
  capture.lastEdgeUs = nowUs;

  if (digitalRead(pin) == HIGH) {
    if (capture.seenRise) {
      capture.periodUs = nowUs - capture.lastRiseUs;
      capture.hasMeasurement = true;
    }
    capture.lastRiseUs = nowUs;
    capture.seenRise = true;
  } else if (capture.seenRise) {
    capture.highUs = nowUs - capture.lastRiseUs;
  }
}

void pwm1Interrupt() {
  updatePulseCapture(pwm1Capture, PWM_1_MEASURE_PIN);
}

void pwm2Interrupt() {
  updatePulseCapture(pwm2Capture, PWM_2_MEASURE_PIN);
}

PulseMeasurement readPulseMeasurement(PulseCapture &capture) {
  PulseMeasurement measurement;
  const uint32_t nowUs = micros();
  uint32_t lastEdgeUs = 0;

  noInterrupts();
  measurement.highUs = capture.highUs;
  measurement.periodUs = capture.periodUs;
  lastEdgeUs = capture.lastEdgeUs;
  measurement.valid = capture.hasMeasurement &&
                      capture.periodUs > 0 &&
                      capture.highUs <= capture.periodUs &&
                      (nowUs - lastEdgeUs) <= PULSE_TIMEOUT_US;
  interrupts();

  return measurement;
}

DashboardChannel buildChannel(const char *label,
                              uint8_t pwmPin,
                              uint8_t measurePin,
                              int adcRaw,
                              int pwmOutput,
                              const PulseMeasurement &measurement) {
  DashboardChannel ch{};
  ch.label = label;
  ch.pwmPin = pwmPin;
  ch.measurePin = measurePin;
  ch.adcRaw = adcRaw;
  ch.adcMax = ADC_MAX_VALUE;
  ch.voltage = adcRaw * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
  ch.pwmOutput = pwmOutput;
  ch.pwmMax = PWM_MAX_VALUE;
  ch.commandedDutyPercent = pwmOutput * 100.0f / PWM_MAX_VALUE;
  ch.measurement = measurement;
  ch.inputHigh = digitalRead(measurePin) == HIGH;
  return ch;
}

DashboardState buildDashboardState(int pot1Raw, int pot2Raw,
                                   int pwm1OutputValue, int pwm2OutputValue,
                                   const PulseMeasurement &m1,
                                   const PulseMeasurement &m2) {
  const uint32_t nowUs = micros();
  DashboardState s{};
  s.uptimeMs = millis();
  s.pwmEnabled = pwmEnabled;
  s.pwmFrequencyHz = PWM_FREQUENCY_HZ;

  s.ch1 = buildChannel("A0", PWM_1_PIN, PWM_1_MEASURE_PIN, pot1Raw, pwm1OutputValue, m1);
  s.ch2 = buildChannel("A1", PWM_2_PIN, PWM_2_MEASURE_PIN, pot2Raw, pwm2OutputValue, m2);

  s.canControlId = CAN_CONTROL_ID;
  s.canBaudRate = CAN_BAUD_RATE;
  s.pcHeartbeatId = PC_HEARTBEAT_ID;
  s.teensyHeartbeatId = TEENSY_HEARTBEAT_ID;
  s.heartbeatMagic = HEARTBEAT_MAGIC;
  s.heartbeatTxIntervalMs = HEARTBEAT_TX_INTERVAL_US / 1000;

  s.lastCanCommandName = canCommandName(lastCanCommand);
  s.hasReceivedCommand = canCommandCount > 0;
  s.canCommandCount = canCommandCount;
  s.canInvalidCommandCount = canInvalidCommandCount;
  s.lastCanCommandAgeMs = canCommandCount > 0 ? (nowUs - lastCanCommandUs) / 1000 : 0;

  s.canWatchdogTriggered = canWatchdogTriggered;
  s.canWatchdogTimeoutMs = CAN_WATCHDOG_TIMEOUT_US / 1000;

  s.pcHeartbeatCount = pcHeartbeatCount;
  s.teensyHeartbeatCount = teensyHeartbeatCount;
  s.pcHeartbeatSeen = lastPcHeartbeatUs != 0;
  s.lastPcHeartbeatAgeMs = s.pcHeartbeatSeen ? (nowUs - lastPcHeartbeatUs) / 1000 : 0;
  s.pcHeartbeatAlive = s.pcHeartbeatSeen &&
                       (nowUs - lastPcHeartbeatUs) <= CAN_WATCHDOG_TIMEOUT_US;

  return s;
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  analogReadResolution(ADC_RESOLUTION_BITS);
  analogReadAveraging(ADC_AVERAGING_SAMPLES);
  analogWriteResolution(PWM_RESOLUTION_BITS);

  pinMode(POT_1_PIN, INPUT);
  pinMode(POT_2_PIN, INPUT);
  pinMode(PWM_1_PIN, OUTPUT);
  pinMode(PWM_2_PIN, OUTPUT);
  pinMode(PWM_1_MEASURE_PIN, INPUT);
  pinMode(PWM_2_MEASURE_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  analogWriteFrequency(PWM_1_PIN, PWM_FREQUENCY_HZ);
  analogWriteFrequency(PWM_2_PIN, PWM_FREQUENCY_HZ);

  Can1.begin();
  Can1.setBaudRate(CAN_BAUD_RATE);

  attachInterrupt(digitalPinToInterrupt(PWM_1_MEASURE_PIN), pwm1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_2_MEASURE_PIN), pwm2Interrupt, CHANGE);
}

void loop() {
  readCanCommands();
  sendTeensyHeartbeat();

  if (pwmEnabled &&
      (lastPcHeartbeatUs == 0 || (micros() - lastPcHeartbeatUs) > CAN_WATCHDOG_TIMEOUT_US)) {
    pwmEnabled = false;
    canWatchdogTriggered = true;
  }

  const int pot1Raw = analogRead(POT_1_PIN);
  const int pot2Raw = analogRead(POT_2_PIN);
  const int pwm1CommandValue = map(pot1Raw, 0, ADC_MAX_VALUE, 0, PWM_MAX_VALUE);
  const int pwm2CommandValue = map(pot2Raw, 0, ADC_MAX_VALUE, 0, PWM_MAX_VALUE);
  const int pwm1OutputValue = pwmEnabled ? pwm1CommandValue : 0;
  const int pwm2OutputValue = pwmEnabled ? pwm2CommandValue : 0;

  analogWrite(PWM_1_PIN, pwm1OutputValue);
  analogWrite(PWM_2_PIN, pwm2OutputValue);
  digitalWrite(STATUS_LED_PIN, pwmEnabled ? HIGH : LOW);

  const PulseMeasurement pwm1Measurement = readPulseMeasurement(pwm1Capture);
  const PulseMeasurement pwm2Measurement = readPulseMeasurement(pwm2Capture);
  sendPulseTelemetry(pot1Raw, pot2Raw, pwm1Measurement, pwm2Measurement);

  const uint32_t nowMs = millis();
  if ((nowMs - lastDashboardRenderMs) >= READ_INTERVAL_MS) {
    const DashboardState state = buildDashboardState(
        pot1Raw, pot2Raw,
        pwm1OutputValue, pwm2OutputValue,
        pwm1Measurement, pwm2Measurement);
    dashboard::render(state);
    lastDashboardRenderMs = nowMs;
  }
}
