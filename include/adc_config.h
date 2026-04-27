#pragma once

#include <stdint.h>

constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr int ADC_RESOLUTION_BITS = 12;
constexpr int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1;
constexpr int ADC_AVERAGING_SAMPLES = 16;
