#include <Arduino.h>
#include <FlexCAN_T4.h>

const uint8_t POT_1_PIN = A0;
const uint8_t POT_2_PIN = A1;
const uint8_t PWM_1_PIN = 2;
const uint8_t PWM_2_PIN = 3;
const uint8_t PWM_1_MEASURE_PIN = 4;
const uint8_t PWM_2_MEASURE_PIN = 5;
const uint8_t STATUS_LED_PIN = LED_BUILTIN;

const uint32_t CAN_BAUD_RATE = 500000;
const uint32_t CAN_CONTROL_ID = 0x600;
const uint32_t PC_HEARTBEAT_ID = 0x610;
const uint32_t TEENSY_HEARTBEAT_ID = 0x611;
const uint8_t CAN_CMD_OFF = 0x00;
const uint8_t CAN_CMD_ON = 0x01;
const uint8_t CAN_CMD_TOGGLE = 0x02;
const uint8_t HEARTBEAT_MAGIC = 0xA5;

const float ADC_REFERENCE_VOLTAGE = 3.3f;
const int ADC_RESOLUTION_BITS = 12;
const int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1;
const int PWM_RESOLUTION_BITS = 12;
const int PWM_MAX_VALUE = (1 << PWM_RESOLUTION_BITS) - 1;
const float PWM_FREQUENCY_HZ = 800.0f;

const unsigned long SERIAL_BAUD_RATE = 115200;
const unsigned long READ_INTERVAL_MS = 200;
const uint32_t PULSE_TIMEOUT_US = 10000;
const uint32_t CAN_WATCHDOG_TIMEOUT_US = 5000000;
const uint32_t HEARTBEAT_TX_INTERVAL_US = 1000000;
const int BAR_WIDTH = 30;
const int WAVEFORM_WIDTH = 40;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
volatile bool pwmEnabled = false;
uint32_t lastCanCommandUs = 0;
uint32_t canCommandCount = 0;
uint32_t canInvalidCommandCount = 0;
uint8_t lastCanCommand = CAN_CMD_OFF;
bool canWatchdogTriggered = false;
uint32_t lastPcHeartbeatUs = 0;
uint32_t lastTeensyHeartbeatTxUs = 0;
uint32_t pcHeartbeatCount = 0;
uint32_t teensyHeartbeatCount = 0;

struct PulseCapture {
  volatile uint32_t lastRiseUs = 0;
  volatile uint32_t lastEdgeUs = 0;
  volatile uint32_t highUs = 0;
  volatile uint32_t periodUs = 0;
  volatile bool hasMeasurement = false;
  volatile bool seenRise = false;
};

struct PulseMeasurement {
  uint32_t highUs = 0;
  uint32_t periodUs = 0;
  bool valid = false;
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

void printBar(int value, int maxValue) {
  const int filledWidth = (value * BAR_WIDTH + maxValue / 2) / maxValue;

  Serial.print("[");
  for (int i = 0; i < BAR_WIDTH; i++) {
    Serial.print(i < filledWidth ? "#" : ".");
  }
  Serial.print("]");
}

void printSteadyWaveform(bool high) {
  Serial.print(high ? "|" : " ");
  for (int i = 0; i < WAVEFORM_WIDTH; i++) {
    Serial.print(high ? "-" : "_");
  }
  Serial.print(high ? "|" : " ");
}

void printPulseWaveform(const PulseMeasurement &measurement) {
  int highWidth = (measurement.highUs * WAVEFORM_WIDTH + measurement.periodUs / 2) / measurement.periodUs;
  highWidth = constrain(highWidth, 0, WAVEFORM_WIDTH);

  Serial.print("|");
  for (int i = 0; i < highWidth; i++) {
    Serial.print("-");
  }
  Serial.print("|");
  for (int i = highWidth; i < WAVEFORM_WIDTH; i++) {
    Serial.print("_");
  }
}

void printChannelValue(const char *name, uint8_t pwmPin, int analogValue, int pwmValue) {
  const float voltage = analogValue * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE;
  const float dutyPercent = pwmValue * 100.0f / PWM_MAX_VALUE;

  Serial.printf("%-4s | PWM %2u | ADC %4d | %5.3f V | duty %6.2f %% | ",
                name,
                pwmPin,
                analogValue,
                voltage,
                dutyPercent);
  printBar(pwmValue, PWM_MAX_VALUE);
  Serial.println();
}

void printMeasuredWaveform(const char *name, uint8_t inputPin, const PulseMeasurement &measurement) {
  Serial.printf("%-4s pin %-2u : ", name, inputPin);

  if (!measurement.valid) {
    printSteadyWaveform(digitalRead(inputPin) == HIGH);
    Serial.println();
    return;
  }

  printPulseWaveform(measurement);
  Serial.println();
}

void printPulseMeasurement(const char *name, uint8_t inputPin, const PulseMeasurement &measurement) {
  Serial.printf("%-4s | IN  %2u | ", name, inputPin);

  if (!measurement.valid) {
    const bool steadyHigh = digitalRead(inputPin) == HIGH;
    Serial.printf("%-12s | %-14s | %-10s | %-12s",
                  steadyHigh ? "steady HIGH" : "steady LOW",
                  "-",
                  "-",
                  "-");
    Serial.println();
    return;
  }

  const float frequencyHz = 1000000.0f / measurement.periodUs;
  const float dutyPercent = measurement.highUs * 100.0f / measurement.periodUs;

  Serial.printf("high %4lu us | period %4lu us | %7.2f Hz | duty %6.2f%%",
                (unsigned long)measurement.highUs,
                (unsigned long)measurement.periodUs,
                frequencyHz,
                dutyPercent);
  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  analogReadResolution(ADC_RESOLUTION_BITS);
  analogReadAveraging(16);
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

  if (canCommandCount > 0 &&
      pwmEnabled &&
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

  Serial.print("\r\033[2J\033[H");
  Serial.println("Teensy 4.1 Analog Input -> 800 Hz PWM Monitor");
  Serial.println("A0(pin 14) -> PWM pin 2, A1(pin 15) -> PWM pin 3");
  Serial.println("Jumper for measurement: PWM pin 2 -> input pin 4, PWM pin 3 -> input pin 5");
  Serial.println("CAN1: pin 22 TX -> WCMCU-230 CTX, pin 23 RX -> WCMCU-230 CRX");
  Serial.printf("ADC range = 0 ~ %d, PWM range = 0 ~ %d, PWM frequency = %.0f Hz\r\n",
                ADC_MAX_VALUE,
                PWM_MAX_VALUE,
                PWM_FREQUENCY_HZ);
  Serial.println();
  Serial.println("CAN CONTROL");
  Serial.printf("ID 0x%03lX | bitrate %lu bps | protocol: data[0] 0x00=OFF, 0x01=ON, 0x02=TOGGLE\r\n",
                (unsigned long)CAN_CONTROL_ID,
                (unsigned long)CAN_BAUD_RATE);
  Serial.printf("Heartbeat: PC -> 0x%03lX#%02X, Teensy -> 0x%03lX#%02X every %lu ms\r\n",
                (unsigned long)PC_HEARTBEAT_ID,
                HEARTBEAT_MAGIC,
                (unsigned long)TEENSY_HEARTBEAT_ID,
                HEARTBEAT_MAGIC,
                (unsigned long)(HEARTBEAT_TX_INTERVAL_US / 1000));
  Serial.printf("PWM output: %-3s | last command: %-7s | valid rx: %lu | invalid rx: %lu\r\n",
                pwmEnabled ? "ON" : "OFF",
                canCommandCount == 0 ? "-" : canCommandName(lastCanCommand),
                (unsigned long)canCommandCount,
                (unsigned long)canInvalidCommandCount);
  Serial.printf("Watchdog: %s (timeout %lu ms)\r\n",
                canWatchdogTriggered ? "TRIPPED -> forced OFF" : "ok",
                (unsigned long)(CAN_WATCHDOG_TIMEOUT_US / 1000));
  Serial.printf("Heartbeat rx: %lu | tx: %lu | PC heartbeat age: ",
                (unsigned long)pcHeartbeatCount,
                (unsigned long)teensyHeartbeatCount);
  if (lastPcHeartbeatUs == 0) {
    Serial.println("none");
  } else {
    Serial.printf("%lu ms\r\n", (unsigned long)((micros() - lastPcHeartbeatUs) / 1000));
  }
  Serial.printf("Last valid command age: ");
  if (canCommandCount == 0) {
    Serial.println("none");
  } else {
    Serial.printf("%lu ms\r\n", (unsigned long)((micros() - lastCanCommandUs) / 1000));
  }
  Serial.println();
  Serial.println("PWM OUTPUT");
  Serial.println("IN   | OUT   | ADC      | VOLTAGE | DUTY       | LEVEL");
  Serial.println("-----+-------+----------+---------+------------+--------------------------------");
  printChannelValue("A0", PWM_1_PIN, pot1Raw, pwm1OutputValue);
  printChannelValue("A1", PWM_2_PIN, pot2Raw, pwm2OutputValue);
  Serial.println("-----+-------+----------+---------+------------+--------------------------------");
  Serial.println();
  Serial.println("MEASURED PWM");
  Serial.printf("%-4s | %-6s | %-12s | %-14s | %-10s | %-12s\r\n",
                "SRC",
                "INPUT",
                "HIGH",
                "PERIOD",
                "FREQ",
                "DUTY");
  Serial.println("-----+--------+--------------+----------------+------------+--------------");
  printPulseMeasurement("PWM2", PWM_1_MEASURE_PIN, pwm1Measurement);
  printPulseMeasurement("PWM3", PWM_2_MEASURE_PIN, pwm2Measurement);
  Serial.println("-----+--------+--------------+----------------+------------+--------------");
  Serial.println();
  Serial.println("MEASURED WAVEFORM");
  printMeasuredWaveform("PWM2", PWM_1_MEASURE_PIN, pwm1Measurement);
  printMeasuredWaveform("PWM3", PWM_2_MEASURE_PIN, pwm2Measurement);

  delay(READ_INTERVAL_MS);
}
