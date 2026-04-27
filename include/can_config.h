#pragma once

#include <stdint.h>

constexpr uint32_t CAN_BAUD_RATE = 500000;

constexpr uint32_t CAN_CONTROL_ID = 0x600;
constexpr uint32_t PC_HEARTBEAT_ID = 0x610;
constexpr uint32_t TEENSY_HEARTBEAT_ID = 0x611;
constexpr uint32_t PWM_1_TELEMETRY_ID = 0x621;
constexpr uint32_t PWM_2_TELEMETRY_ID = 0x622;

constexpr uint8_t CAN_CMD_OFF = 0x00;
constexpr uint8_t CAN_CMD_ON = 0x01;
constexpr uint8_t CAN_CMD_TOGGLE = 0x02;

constexpr uint8_t HEARTBEAT_MAGIC = 0xA5;

constexpr uint32_t CAN_WATCHDOG_TIMEOUT_US = 5000000;
constexpr uint32_t HEARTBEAT_TX_INTERVAL_US = 1000000;
constexpr uint32_t PULSE_TELEMETRY_TX_INTERVAL_US = 20000;
