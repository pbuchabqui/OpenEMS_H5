#pragma once

#include <cstdint>

// =============================================================================
// ADC HAL para STM32H562RGT6 — OpenEMS v2.2
//
// Canalização conforme OpenEMS_v2.2.json pinout:
//
//   ADC1 (MAP/TPS/O2 — amostragem sincronizada ao dente CKP):
//     MAP  → PC0  → ADC1_IN11  (×64 oversampling → 16-bit efetivo)
//     TPS  → PC2  → ADC1_IN13  (×16 oversampling → 14-bit efetivo)
//     O2   → PA4  → ADC1_IN5   (×16 oversampling → 14-bit efetivo)
//
//   ADC2 (CLT/IAT/fuel/oil — amostragem por software poll):
//     CLT        → PC4  → ADC2_IN11  (×16 oversampling)
//     IAT        → PC5  → ADC2_IN12  (×16 oversampling)
//     fuel_press → PA2  → ADC2_IN14  (×16 oversampling)
//     oil_press  → PA3  → ADC2_IN15  (×16 oversampling)
//
// Trigger: TIM2 TRGO (Update Event) → cada dente CKP
// =============================================================================

namespace ems::hal {

// Canais ADC1 — amostragem síncrona ao CKP (via TIM2 TRGO + GPDMA)
enum class Adc1Channel : uint8_t {
    MAP = 0,        // PC0  → ADC1_IN11
    TPS = 1,        // PC2  → ADC1_IN13
    O2  = 2,        // PA4  → ADC1_IN5
    COUNT = 3,
};

// Canais ADC2 — amostragem por software poll
enum class Adc2Channel : uint8_t {
    CLT        = 0,  // PC4  → ADC2_IN11
    IAT        = 1,  // PC5  → ADC2_IN12
    FUEL_PRESS = 2,  // PA2  → ADC2_IN14
    OIL_PRESS  = 3,  // PA3  → ADC2_IN15
    COUNT = 4,
};

void     adc_init() noexcept;
void     adc_pdb_on_tooth(uint16_t tooth_period_ticks) noexcept;

uint16_t adc1_read(Adc1Channel ch) noexcept;
uint16_t adc2_read(Adc2Channel ch) noexcept;

#if defined(EMS_HOST_TEST)
void     adc_test_set_raw_adc1(Adc1Channel ch, uint16_t raw) noexcept;
void     adc_test_set_raw_adc2(Adc2Channel ch, uint16_t raw) noexcept;
uint16_t adc_test_last_pdb_mod() noexcept;
#endif

}  // namespace ems::hal