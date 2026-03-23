#pragma once

#include <cstdint>

namespace ems::hal {

void icache_init() noexcept;

void tim2_init() noexcept;
void tim5_init() noexcept;
void tim1_init() noexcept;
void tim8_init() noexcept;
void tim15_init() noexcept;
void tim3_pwm_init(uint32_t freq_hz) noexcept;
void tim12_pwm_init(uint32_t freq_hz) noexcept;

uint32_t tim2_count() noexcept;
uint32_t tim5_count() noexcept;

void tim1_set_compare(uint8_t ch, uint16_t ticks) noexcept;
void tim8_set_compare(uint8_t ch, uint16_t ticks) noexcept;
void tim15_set_compare(uint16_t ticks) noexcept;
void tim1_clear_ccf(uint8_t ch) noexcept;
void tim8_clear_ccf(uint8_t ch) noexcept;
void tim15_clear_ccf() noexcept;
void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;
void tim12_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept;
void bkin_rearm_tim1() noexcept;
void bkin_rearm_tim8() noexcept;
bool bkin_test_tim1() noexcept;
bool bkin_test_tim8() noexcept;

extern "C" void TIM1_CC_IRQHandler();
extern "C" void TIM8_CC_IRQHandler();
extern "C" void TIM1_BRK_TIM15_IRQHandler();
extern "C" void TIM2_IRQHandler();

#if defined(EMS_HOST_TEST)
uint16_t tim_test_get_compare(uint8_t timer_group, uint8_t ch) noexcept;
void tim_test_set_counter(uint8_t timer_group, uint32_t value) noexcept;
void tim_test_clear_all() noexcept;
#endif

}  // namespace ems::hal
