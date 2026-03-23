#include "hal/tim.h"

#ifndef EMS_HOST_TEST

#include "drv/ckp.h"
#include "drv/scheduler.h"
#include "hal/regs.h"

namespace {

constexpr uint32_t kTimerClockHz = 250000000u;

void init_tim2_capture() noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;

    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 0u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 1u, GPIO_AF1);

    TIM2_CR1 = 0u;
    TIM2_PSC = 0u;
    TIM2_ARR = 0xFFFFFFFFu;
    TIM2_EGR = 1u;
    TIM2_CCMR1 = TIM_CCMR1_CC1S_TI1 |
                 TIM_CCMR1_IC1F_NONE |
                 TIM_CCMR1_CC2S_TI2 |
                 TIM_CCMR1_IC2F_NONE;
    TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    // ⚡ CORREÇÃO H9: CC2IE desabilitado — CC2 (CMP cam sensor) é tratado via
    // ckp_capture_secondary_isr() chamado do handler TIM2_IRQHandler após
    // verificação de flag. CC2IE geraria interrupções desnecessárias.
    TIM2_DIER = TIM_DIER_CC1IE;
    nvic_set_priority(IRQ_TIM2, 1u);
    nvic_enable_irq(IRQ_TIM2);
    TIM2_CR1 = TIM_CR1_CEN;
}

void init_tim5_timebase() noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_TIM5EN;

    TIM5_CR1 = 0u;
    TIM5_PSC = 0u;
    TIM5_ARR = 0xFFFFFFFFu;
    TIM5_CR2 = TIM_CR2_MMS_UPDATE;
    TIM5_EGR = 1u;
    TIM5_CR1 = TIM_CR1_CEN;
}

void init_tim1_compare() noexcept {
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 8u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 9u, GPIO_AF1);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 10u, GPIO_AF1);
    // ⚡ CORREÇÃO C3: Configurar PB12 como TIM1_BKIN (pin 33, AF1)
    // Fonte: OpenEMS v2.2 Seção 0.4, tabela BKIN:
    //   PB12 B AF1 = TIM1_BKIN (pin 33) — protege INJ1-4
    // Sem esta configuração, o break input fica floating e o shutdown
    // de emergência dos injetores NÃO funciona (risco de incêndio/hydrolock).
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 12u, GPIO_AF1);

    TIM1_CR1 = 0u;
    TIM1_PSC = 0u;
    TIM1_ARR = 0xFFFFu;
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FROZEN | TIM_CCMR1_OC2M_FROZEN;
    TIM1_CCMR2 = 0u;
    TIM1_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1_BDTR = (1u << 15) | (1u << 12) | (2u << 8);
    TIM1_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE;
    // ⚡ CORREÇÃO C4: ITR sync TIM5 → TIM1
    // Fonte: OpenEMS v2.2 Seção 0.4, OPT-8:
    //   TIM5 como master: TRGO on Update
    //   TIM1 como slave: TS = ITR (TIM5), SMS = 100 (reset mode)
    // RESULTADO: os 16 bits baixos de TIM5->CNT == TIM1->CNT a qualquer instante
    // Isso permite a conversão (uint16_t)(abs_ticks) no scheduler.
    // STM32H5 ITR mapping: TIM5 → TIM1 = ITR3 (TS=011)
    TIM1_SMCR = (3u << 4) | (1u << 2);  // TS=011 (ITR3=TIM5), SMS=100 (reset mode)
    TIM1_EGR = 1u;
    nvic_set_priority(IRQ_TIM1_CC, 4u);
    nvic_enable_irq(IRQ_TIM1_CC);
    TIM1_CR1 = TIM_CR1_CEN;
}

void init_tim8_compare() noexcept {
    RCC_APB2ENR |= RCC_APB2ENR_TIM8EN;

    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 6u, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 7u, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 8u, GPIO_AF3);
    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 9u, GPIO_AF3);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 6u, GPIO_AF3);

    TIM8_CR1 = 0u;
    TIM8_PSC = 0u;
    TIM8_ARR = 0xFFFFu;
    TIM8_CCMR1 = TIM_CCMR1_OC1M_FROZEN | TIM_CCMR1_OC2M_FROZEN;
    TIM8_CCMR2 = 0u;
    TIM8_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM8_BDTR = (1u << 15) | (1u << 12) | (2u << 8);
    TIM8_DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;
    // ⚡ CORREÇÃO C4: ITR sync TIM5 → TIM8
    // Fonte: OpenEMS v2.2 Seção 0.4, OPT-8:
    //   TIM5 como master: TRGO on Update
    //   TIM8 como slave: TS = ITR (TIM5), SMS = 100 (reset mode)
    // STM32H5 ITR mapping: TIM5 → TIM8 = ITR2 (TS=010)
    TIM8_SMCR = (2u << 4) | (1u << 2);  // TS=010 (ITR2=TIM5), SMS=100 (reset mode)
    TIM8_EGR = 1u;
    nvic_set_priority(IRQ_TIM8_CC, 4u);
    nvic_enable_irq(IRQ_TIM8_CC);
    TIM8_CR1 = TIM_CR1_CEN;
}

void init_tim15_compare() noexcept {
    RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;

    gpio_set_af(&GPIOC_MODER, &GPIOC_AFRL, &GPIOC_AFRH, &GPIOC_OSPEEDR, 12u, GPIO_AF2);

    TIM15_CR1 = 0u;
    TIM15_PSC = 0u;
    TIM15_ARR = 0xFFFFu;
    TIM15_CCMR1 = TIM_CCMR1_OC1M_FROZEN;
    TIM15_CCER = TIM_CCER_CC1E;
    TIM15_DIER = TIM_DIER_CC1IE;
    TIM15_SMCR = 0u;
    TIM15_EGR = 1u;
    nvic_set_priority(IRQ_TIM15, 4u);
    nvic_enable_irq(IRQ_TIM15);
    TIM15_CR1 = TIM_CR1_CEN;
}

void init_tim3_pwm(uint32_t freq_hz) noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_TIM3EN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 0u, GPIO_AF2);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 1u, GPIO_AF2);

    uint32_t arr = (freq_hz == 0u) ? 0xFFFFu : (kTimerClockHz / freq_hz);
    if (arr == 0u) {
        arr = 1u;
    }
    if (arr > 0x10000u) {
        arr = 0x10000u;
    }
    arr -= 1u;

    TIM3_CR1 = 0u;
    TIM3_PSC = 0u;
    TIM3_ARR = arr;
    TIM3_CCMR2 = TIM_CCMR2_OC3M_PWM1 |
                 TIM_CCMR2_OC3PE |
                 TIM_CCMR2_OC4M_PWM1 |
                 TIM_CCMR2_OC4PE;
    TIM3_CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3_CCR3 = 0u;
    TIM3_CCR4 = 0u;
    TIM3_EGR = 1u;
    TIM3_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void init_tim12_pwm(uint32_t freq_hz) noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_TIM12EN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 14u, GPIO_AF2);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 15u, GPIO_AF2);

    uint32_t arr = (freq_hz == 0u) ? 0xFFFFu : (kTimerClockHz / freq_hz);
    if (arr == 0u) {
        arr = 1u;
    }
    if (arr > 0x10000u) {
        arr = 0x10000u;
    }
    arr -= 1u;

    TIM12_CR1 = 0u;
    TIM12_PSC = 0u;
    TIM12_ARR = arr;
    TIM12_CCMR1 = TIM_CCMR1_OC1M_PWM1 |
                  TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_PWM1 |
                  TIM_CCMR1_OC2PE;
    TIM12_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM12_CCR1 = 0u;
    TIM12_CCR2 = 0u;
    TIM12_EGR = 1u;
    TIM12_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void clear_cc_flag(volatile uint32_t& sr, uint32_t flag) noexcept {
    sr &= ~flag;
}

void clear_tim2_ccf(uint8_t ch) noexcept {
    if (ch == 0u) {
        clear_cc_flag(TIM2_SR, TIM_SR_CC1IF);
    } else if (ch == 1u) {
        clear_cc_flag(TIM2_SR, TIM_SR_CC2IF);
    }
}

}  // namespace

namespace ems::hal {

void icache_init() noexcept {
    // ⚡ OPT-9: Habilitar Instruction Cache 8KB + Flash Prefetch
    // Referência: RM0481 §7.3.1 — FLASH_ACR
    // ICACHE no STM32H5 é controlado via FLASH_ACR bits ICEN/DCEN
    FLASH_ACR = FLASH_ACR_LATENCY_5WS |  // 5 wait-states @ 250 MHz
                FLASH_ACR_PRFTEN |        // Prefetch enable
                FLASH_ACR_ICEN;           // Instruction cache enable (8KB, 2-way)
    // Após ICACHE enable, ISR executa a 250 MHz (vs ~42 MHz sem cache)
}

void tim2_init() noexcept { init_tim2_capture(); }
void tim5_init() noexcept { init_tim5_timebase(); }
void tim1_init() noexcept { init_tim1_compare(); }
void tim8_init() noexcept { init_tim8_compare(); }
void tim15_init() noexcept { init_tim15_compare(); }
void tim3_pwm_init(uint32_t freq_hz) noexcept { init_tim3_pwm(freq_hz); }
void tim12_pwm_init(uint32_t freq_hz) noexcept { init_tim12_pwm(freq_hz); }

uint32_t tim2_count() noexcept { return TIM2_CNT; }
uint32_t tim5_count() noexcept { return TIM5_CNT; }

void tim1_set_compare(uint8_t ch, uint16_t ticks) noexcept {
    switch (ch) {
        case 0u: TIM1_CCR1 = ticks; break;
        case 1u: TIM1_CCR2 = ticks; break;
        case 2u: TIM1_CCR3 = ticks; break;
        default: break;
    }
}

void tim8_set_compare(uint8_t ch, uint16_t ticks) noexcept {
    switch (ch) {
        case 0u: TIM8_CCR1 = ticks; break;
        case 1u: TIM8_CCR2 = ticks; break;
        case 2u: TIM8_CCR3 = ticks; break;
        case 3u: TIM8_CCR4 = ticks; break;
        default: break;
    }
}

void tim15_set_compare(uint16_t ticks) noexcept {
    TIM15_CCR1 = ticks;
}

void tim1_clear_ccf(uint8_t ch) noexcept {
    switch (ch) {
        case 0u: clear_cc_flag(TIM1_SR, TIM_SR_CC1IF); break;
        case 1u: clear_cc_flag(TIM1_SR, TIM_SR_CC2IF); break;
        case 2u: clear_cc_flag(TIM1_SR, TIM_SR_CC3IF); break;
        default: break;
    }
}

void tim8_clear_ccf(uint8_t ch) noexcept {
    switch (ch) {
        case 0u: clear_cc_flag(TIM8_SR, TIM_SR_CC1IF); break;
        case 1u: clear_cc_flag(TIM8_SR, TIM_SR_CC2IF); break;
        case 2u: clear_cc_flag(TIM8_SR, TIM_SR_CC3IF); break;
        case 3u: clear_cc_flag(TIM8_SR, TIM_SR_CC4IF); break;
        default: break;
    }
}

void tim15_clear_ccf() noexcept {
    clear_cc_flag(TIM15_SR, TIM_SR_CC1IF);
}

void tim3_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM3_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM3_CCR3 = ccr;
    } else if (ch == 1u) {
        TIM3_CCR4 = ccr;
    }
}

void tim12_set_duty(uint8_t ch, uint16_t duty_pct_x10) noexcept {
    const uint32_t arr = TIM12_ARR;
    const uint32_t ccr = ((arr + 1u) * duty_pct_x10) / 1000u;
    if (ch == 0u) {
        TIM12_CCR1 = ccr;
    } else if (ch == 1u) {
        TIM12_CCR2 = ccr;
    }
}

void bkin_rearm_tim1() noexcept {
    TIM1_BDTR |= (1u << 15);
}

void bkin_rearm_tim8() noexcept {
    TIM8_BDTR |= (1u << 15);
}

bool bkin_test_tim1() noexcept {
    // Test TIM1 BKIN: verify BKE bit is set and BKIN pin (PB12) is configured
    // Check BDTR.BKE (bit 12) is set
    if ((TIM1_BDTR & (1u << 12u)) == 0u) {
        return false;
    }
    // Check PB12 is configured as AF1 (TIM1_BKIN)
    // MODER bits [25:24] should be 10b (AF mode)
    // AFRH bits [19:16] should be 0001b (AF1)
    const uint32_t moder = GPIOB_MODER;
    const uint32_t afrh = GPIOB_AFRH;
    if (((moder >> 24u) & 0x03u) != 0x02u) {  // Not AF mode
        return false;
    }
    if (((afrh >> 16u) & 0x0Fu) != 0x01u) {  // Not AF1
        return false;
    }
    return true;
}

bool bkin_test_tim8() noexcept {
    // Test TIM8 BKIN: verify BKE bit is set and BKIN pin (PA6) is configured
    // Check BDTR.BKE (bit 12) is set
    if ((TIM8_BDTR & (1u << 12u)) == 0u) {
        return false;
    }
    // Check PA6 is configured as AF3 (TIM8_BKIN)
    // MODER bits [13:12] should be 10b (AF mode)
    // AFRL bits [27:24] should be 0011b (AF3)
    const uint32_t moder = GPIOA_MODER;
    const uint32_t afrl = GPIOA_AFRL;
    if (((moder >> 12u) & 0x03u) != 0x02u) {  // Not AF mode
        return false;
    }
    if (((afrl >> 24u) & 0x0Fu) != 0x03u) {  // Not AF3
        return false;
    }
    return true;
}

extern "C" void TIM2_IRQHandler() {
    const uint32_t sr = TIM2_SR;
    if ((sr & TIM_SR_CC1IF) != 0u) {
        clear_tim2_ccf(0u);
        ems::drv::ckp_capture_primary_isr();
    }
    if ((sr & TIM_SR_CC2IF) != 0u) {
        clear_tim2_ccf(1u);
        ems::drv::ckp_capture_secondary_isr();
    }
}

extern "C" void TIM1_CC_IRQHandler() {
    ems::drv::sched_isr_tim1();
}

extern "C" void TIM8_CC_IRQHandler() {
    ems::drv::sched_isr_tim8();
}

extern "C" void TIM1_BRK_TIM15_IRQHandler() {
    // ⚡ CORREÇÃO C6: TIM15 compartilha vetor com TIM1_BRK no STM32H5
    // Verificar flag TIM15 CC1IF antes de processar
    if ((TIM15_SR & TIM_SR_CC1IF) != 0u) {
        ems::drv::sched_isr_tim15();
    }
    // TIM1_BRK flag seria tratado aqui se necessário
}

}  // namespace ems::hal

#else

namespace ems::hal {

static uint32_t g_mock_tim2_cnt = 0u;
static uint32_t g_mock_tim5_cnt = 0u;
static uint16_t g_mock_tim1_ccr[3] = {};
static uint16_t g_mock_tim8_ccr[4] = {};
static uint16_t g_mock_tim15_ccr = 0u;

void icache_init() noexcept {}
void tim2_init() noexcept {}
void tim5_init() noexcept {}
void tim1_init() noexcept {}
void tim8_init() noexcept {}
void tim15_init() noexcept {}
void tim3_pwm_init(uint32_t) noexcept {}
void tim12_pwm_init(uint32_t) noexcept {}
uint32_t tim2_count() noexcept { return g_mock_tim2_cnt; }
uint32_t tim5_count() noexcept { return g_mock_tim5_cnt; }
void tim1_set_compare(uint8_t ch, uint16_t ticks) noexcept { if (ch < 3u) { g_mock_tim1_ccr[ch] = ticks; } }
void tim8_set_compare(uint8_t ch, uint16_t ticks) noexcept { if (ch < 4u) { g_mock_tim8_ccr[ch] = ticks; } }
void tim15_set_compare(uint16_t ticks) noexcept { g_mock_tim15_ccr = ticks; }
void tim1_clear_ccf(uint8_t) noexcept {}
void tim8_clear_ccf(uint8_t) noexcept {}
void tim15_clear_ccf() noexcept {}
void tim3_set_duty(uint8_t, uint16_t) noexcept {}
void tim12_set_duty(uint8_t, uint16_t) noexcept {}
void bkin_rearm_tim1() noexcept {}
void bkin_rearm_tim8() noexcept {}
extern "C" void TIM1_CC_IRQHandler() {}
extern "C" void TIM8_CC_IRQHandler() {}
extern "C" void TIM1_BRK_TIM15_IRQHandler() {}
extern "C" void TIM2_IRQHandler() {}

uint16_t tim_test_get_compare(uint8_t timer_group, uint8_t ch) noexcept {
    if (timer_group == 1u) {
        return (ch < 3u) ? g_mock_tim1_ccr[ch] : 0u;
    }
    if (timer_group == 8u) {
        return (ch < 4u) ? g_mock_tim8_ccr[ch] : 0u;
    }
    if (timer_group == 15u) {
        return (ch == 0u) ? g_mock_tim15_ccr : 0u;
    }
    return 0u;
}

void tim_test_set_counter(uint8_t timer_group, uint32_t value) noexcept {
    if (timer_group == 2u) {
        g_mock_tim2_cnt = value;
    } else if (timer_group == 5u) {
        g_mock_tim5_cnt = value;
    }
}

void tim_test_clear_all() noexcept {
    g_mock_tim2_cnt = 0u;
    g_mock_tim5_cnt = 0u;
    for (uint8_t i = 0u; i < 3u; ++i) {
        g_mock_tim1_ccr[i] = 0u;
    }
    for (uint8_t i = 0u; i < 4u; ++i) {
        g_mock_tim8_ccr[i] = 0u;
    }
    g_mock_tim15_ccr = 0u;
}

}  // namespace ems::hal

#endif
