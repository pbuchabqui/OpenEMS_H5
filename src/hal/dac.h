#pragma once

#include <cstdint>

namespace ems::hal {

/**
 * @brief Initialize DAC1 for knock threshold output
 * 
 * OPT-13: DAC 12-bit for adaptive knock threshold
 * DAC1_OUT2 on PA5 — 12-bit (4096 levels) vs 64 levels on MK64F
 * Resolution: 0.8 mV/step @ 3.3V
 */
void dac_init() noexcept;

/**
 * @brief Set knock threshold value (12-bit)
 * 
 * @param value_12bit  Threshold 0..4095 (0.8 mV/step @ 3.3V)
 */
void dac_set_knock_threshold(uint16_t value_12bit) noexcept;

/**
 * @brief Get current knock threshold value
 * 
 * @return Current DAC output value 0..4095
 */
uint16_t dac_get_knock_threshold() noexcept;

#if defined(EMS_HOST_TEST)
void dac_test_reset() noexcept;
uint16_t dac_test_get_output() noexcept;
#endif

}  // namespace ems::hal