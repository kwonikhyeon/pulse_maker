#pragma once

#include <stdint.h>

constexpr int PWM_RESOLUTION_BITS = 12;
constexpr int PWM_MAX_VALUE = (1 << PWM_RESOLUTION_BITS) - 1;
constexpr int PWM_DEFAULT_DUTY_VALUE = (PWM_MAX_VALUE + 1) / 2;
constexpr float PWM_MIN_FREQUENCY_HZ = 800.0f;
constexpr float PWM_MAX_FREQUENCY_HZ = 3000.0f;

constexpr uint32_t PULSE_TIMEOUT_US = 10000;
