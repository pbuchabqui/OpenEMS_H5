#include "hal/dac.h"

#ifndef EMS_HOST_TEST

#include "hal/regs.h"

// DAC1 base address
#define DAC1_BASE       0x46010800UL
#define DAC_DHR12R2_OFF 0x14UL   // DAC channel2 12-bit right-aligned data holding register
#define DAC_CR_OFF      0x00UL   // DAC control register
#define DAC_SWTRIGR_OFF 0x04UL   // DAC software trigger register

#define DAC_DHR12R2 STM32_REG32(DAC1_BASE + DAC_DHR12R2_OFF)
#define DAC_CR      STM32_REG32(DAC1_BASE + DAC_CR_OFF)
#define DAC_SWTRIGR STM32_REG32(DAC1_BASE + DAC_SWTRIGR_OFF)

// DAC_CR bits for channel 2
#define DAC_CR_EN2      (1u << 16)   // DAC channel2 enable
#define DAC_CR_TEN2     (1u << 18)   // DAC channel2 trigger enable
#define DAC_CR_TSEL2_SW (0u << 19)   // DAC channel2 trigger selection: Software
#define DAC_CR_BOFF2    (1u << 17)   // DAC channel2 output buffer enable

// DAC->CR MASK for CH2
#define DAC_CR_CH2_MASK 0x00070000u

static uint16_t g_dac_knock_threshold = 0u;

namespace ems::hal {

void dac_init() noexcept {
    // Enable DAC1 clock
    RCC_APB1LENR |= (1u << 29);  // DAC1EN (bit 29 em APB1LENR)

    // Configure PA5 as analog (DAC1_OUT2)
    // PA5 = pin 21 on LQFP64
    // Already configured as analog by adc_init() if needed
    // Explicitly set as analog here for safety
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (5u * 2u))) | (3u << (5u * 2u));

    // Configure DAC channel 2
    // - Enable channel 2
    // - Output buffer enabled (BOFF2 = 0)
    // - Software trigger (TSEL2 = 000)
    // - Trigger disabled initially (TEN2 = 0)
    DAC_CR = DAC_CR_EN2 | DAC_CR_BOFF2;

    // Set initial output to 0
    DAC_DHR12R2 = 0u;
    g_dac_knock_threshold = 0u;
}

void dac_set_knock_threshold(uint16_t value_12bit) noexcept {
    // Clamp to 12-bit range
    if (value_12bit > 4095u) {
        value_12bit = 4095u;
    }

    // Write 12-bit right-aligned value
    DAC_DHR12R2 = value_12bit;
    g_dac_knock_threshold = value_12bit;
}

uint16_t dac_get_knock_threshold() noexcept {
    return g_dac_knock_threshold;
}

}  // namespace ems::hal

#else

namespace ems::hal {

static uint16_t g_mock_dac_output = 0u;

void dac_init() noexcept {
    g_mock_dac_output = 0u;
}

void dac_set_knock_threshold(uint16_t value_12bit) noexcept {
    if (value_12bit > 4095u) {
        value_12bit = 4095u;
    }
    g_mock_dac_output = value_12bit;
}

uint16_t dac_get_knock_threshold() noexcept {
    return g_mock_dac_output;
}

void dac_test_reset() noexcept {
    g_mock_dac_output = 0u;
}

uint16_t dac_test_get_output() noexcept {
    return g_mock_dac_output;
}

}  // namespace ems::hal

#endif