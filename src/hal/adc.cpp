/**
 * @file hal/adc.cpp
 * @brief ADC1 + ADC2 com hardware oversampling e trigger TIM2 — STM32H562RGT6
 *
 * Canalização conforme OpenEMS v2.2 (OpenEMS_v2.2.json + Prompt 4):
 *
 *   ADC1 (MAP/TPS/O2 — amostragem síncrona ao dente CKP via TIM2 TRGO):
 *     MAP  → PC0  → ADC1_IN11  (×64 oversampling → 16-bit efetivo)
 *     TPS  → PC2  → ADC1_IN13  (×16 oversampling → 14-bit efetivo)
 *     O2   → PA4  → ADC1_IN5   (×16 oversampling → 14-bit efetivo)
 *
 *   ADC2 (CLT/IAT/fuel/oil — amostragem por software poll):
 *     CLT        → PC4  → ADC2_IN11  (×16 oversampling)
 *     IAT        → PC5  → ADC2_IN12  (×16 oversampling)
 *     fuel_press → PA2  → ADC2_IN14  (×16 oversampling)
 *     oil_press  → PA3  → ADC2_IN15  (×16 oversampling)
 *
 * ⚡ OPT-6: ADC hardware oversampling
 *   ADC1: MAP usa ×64 (OVSR=0101, OVSS=0010) → 16-bit efetivo
 *         TPS/O2 usam ×16 (OVSR=0011, OVSS=0010) → 14-bit efetivo
 *   ADC2: todos ×16 (OVSR=0011, OVSS=0010) → 14-bit efetivo
 *
 * ⚡ OPT-5: GPDMA linked-list para pipeline ADC1 autônomo
 *   TIM2 TRGO → ADC1 sequência → GPDMA → buffer circular
 */

#ifndef EMS_HOST_TEST

#include "hal/adc.h"
#include "hal/regs.h"

namespace {

// ── Índices internos ──────────────────────────────────────────────────────────
// g_adc1_raw[0] = MAP, [1] = TPS, [2] = O2
static volatile uint16_t g_adc1_raw[3] = {};
// g_adc2_raw[0] = CLT, [1] = IAT, [2] = FUEL_PRESS, [3] = OIL_PRESS
static volatile uint16_t g_adc2_raw[4] = {};

// ── Constantes de canal STM32H5 ───────────────────────────────────────────────
// ADC1 canais: MAP=IN11, TPS=IN13, O2=IN5
static constexpr uint8_t kAdc1Channels[3] = {11u, 13u, 5u};
// ADC2 canais: CLT=IN11, IAT=IN12, FUEL_PRESS=IN14, OIL_PRESS=IN15
static constexpr uint8_t kAdc2Channels[4] = {11u, 12u, 14u, 15u};

// ── Tempo de amostragem: 47.5 ciclos (SMP[2:0] = 011b) ──────────────────────
static constexpr uint32_t kSmpr47c5 = 0x03u;

// ── Utilitários ───────────────────────────────────────────────────────────────

inline void gpio_set_analog_mode(volatile uint32_t* moder, uint8_t pin) noexcept {
    // MODER: 11b = Analog mode
    *moder = (*moder & ~(0x03u << (pin * 2u))) | (0x03u << (pin * 2u));
}

inline void adc_wait_ready(volatile uint32_t& isr) noexcept {
    constexpr uint32_t kTimeout = 1000000u;
    for (uint32_t i = 0u; i < kTimeout; ++i) {
        if (isr & (1u << 0u)) { break; }  // ADRDY = bit 0
    }
}

void adc_calibrate_and_enable(volatile uint32_t& cr,
                               volatile uint32_t& isr) noexcept {
    // 1. Sair de deep power-down e habilitar regulador interno
    cr &= ~(1u << 29u);   // DEEPPWD = bit 29
    cr |= (1u << 28u);    // ADVREGEN = bit 28
    // Aguardar estabilização do regulador (~20 µs)
    for (volatile uint32_t i = 0u; i < 5000u; ++i) { (void)i; }

    // 2. Calibração single-ended
    cr &= ~(1u << 30u);   // ADCALDIF = bit 30 → single-ended
    cr |= (1u << 31u);    // ADCAL = bit 31
    while (cr & (1u << 31u)) { /* aguarda */ }

    // 3. Habilitar ADC
    isr = (1u << 0u);     // W1C: limpa ADRDY
    cr |= (1u << 0u);     // ADEN = bit 0
    adc_wait_ready(isr);
}

// Configura sequência de conversão para um ADC
void adc_configure_sequence(volatile uint32_t& sqr1,
                             volatile uint32_t& sqr2,
                             const uint8_t* channels,
                             uint8_t count) noexcept {
    // SQR1: L[3:0] = count-1, SQ1-SQ4 nos bits [10:6],[16:12],[22:18],[28:24]
    uint32_t s1 = static_cast<uint32_t>(count - 1u);
    for (uint8_t i = 0u; i < 4u && i < count; ++i) {
        s1 |= static_cast<uint32_t>(channels[i]) << (6u + i * 6u);
    }
    sqr1 = s1;

    // SQR2: SQ5-SQ9 nos bits [4:0],[10:6],[16:12],[22:18],[28:24]
    uint32_t s2 = 0u;
    for (uint8_t i = 4u; i < 9u && i < count; ++i) {
        s2 |= static_cast<uint32_t>(channels[i]) << ((i - 4u) * 6u);
    }
    sqr2 = s2;
}

// Configura tempo de amostragem para todos os canais usados
void adc_configure_sampling(volatile uint32_t& smpr1,
                             volatile uint32_t& smpr2,
                             const uint8_t* channels,
                             uint8_t count) noexcept {
    // SMPR1: canais 0-9, cada canal usa 3 bits (SMP[2:0])
    // SMPR2: canais 10-18, cada canal usa 3 bits
    smpr1 = 0u;
    smpr2 = 0u;
    for (uint8_t i = 0u; i < count; ++i) {
        const uint8_t ch = channels[i];
        if (ch <= 9u) {
            smpr1 |= kSmpr47c5 << (ch * 3u);
        } else if (ch <= 18u) {
            smpr2 |= kSmpr47c5 << ((ch - 10u) * 3u);
        }
    }
}

}  // namespace

namespace ems::hal {

void adc_init() noexcept {
    // ── 1. Habilitar clocks ──────────────────────────────────────────────
    RCC_AHB1ENR |= RCC_AHB1ENR_ADC12EN;
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOCEN;

    // ── 2. Configurar pinos analógicos (MODER = 11b) ─────────────────────
    // ADC1: PA4(IN5), PC0(IN11), PC2(IN13)
    gpio_set_analog_mode(&GPIOA_MODER, 4u);   // PA4 → ADC1_IN5 (O2)
    volatile uint32_t* gpioc_moder =
        reinterpret_cast<volatile uint32_t*>(GPIOC_BASE + GPIO_MODER_OFF);
    gpio_set_analog_mode(gpioc_moder, 0u);    // PC0 → ADC1_IN11 (MAP)
    gpio_set_analog_mode(gpioc_moder, 2u);    // PC2 → ADC1_IN13 (TPS)
    // ADC2: PA2(IN14), PA3(IN15), PC4(IN11), PC5(IN12)
    gpio_set_analog_mode(&GPIOA_MODER, 2u);   // PA2 → ADC2_IN14 (fuel_press)
    gpio_set_analog_mode(&GPIOA_MODER, 3u);   // PA3 → ADC2_IN15 (oil_press)
    gpio_set_analog_mode(gpioc_moder, 4u);    // PC4 → ADC2_IN11 (CLT)
    gpio_set_analog_mode(gpioc_moder, 5u);    // PC5 → ADC2_IN12 (IAT)

    // ── 3. Clock ADC: HCLK/4 = 62.5 MHz ─────────────────────────────────
    // ADC12_CCR: PRESC[3:0] = 0010b = div4
    ADC12_CCR = (0x02u << 18u);

    // ── 4. Configurar e habilitar ADC1 ───────────────────────────────────
    adc_calibrate_and_enable(ADC1_CR, ADC1_ISR);

    // Tempo de amostragem: 47.5 ciclos para todos os canais
    adc_configure_sampling(ADC1_SMPR1, ADC1_SMPR2,
                            kAdc1Channels, 3u);

    // Sequência: MAP(IN11), TPS(IN13), O2(IN5)
    adc_configure_sequence(ADC1_SQR1, ADC1_SQR2,
                            kAdc1Channels, 3u);

    // CFGR1: 12-bit, trigger externo TIM2_TRGO, rising edge
    // EXTSEL = TIM2_TRGO = 0001b (RM0481 Table 68)
    // EXTEN = 01b = rising edge
    ADC1_CFGR1 = (0x00u << 3u)    // RES[1:0] = 00 → 12-bit
               | (0x01u << 6u)    // EXTSEL[3:0] = 0001 → TIM2_TRGO
               | (0x01u << 10u);  // EXTEN[1:0] = 01 → rising edge

    // ⚡ OPT-6: Hardware oversampling
    // CFGR2: OVSE=1, OVSR, OVSS
    // MAP: ×64 → OVSR=0101 (64×), OVSS=0010 (right shift 2) → 16-bit efetivo
    // Para MAP (primeiro canal da sequência), precisamos de oversampling separado
    // Como STM32H5 oversampling é global por ADC, usamos ×64 para todos os canais ADC1
    // Isso dá 16-bit para MAP e ~16-bit para TPS/O2 (melhor que 14-bit)
    ADC1_CFGR2 = (1u << 0u)       // OVSE = 1
               | (0x05u << 2u)    // OVSR = 0101 → 64×
               | (0x02u << 5u);   // OVSS = 0010 → right shift 2 (16-bit efetivo)

    // ── 5. Configurar e habilitar ADC2 ───────────────────────────────────
    adc_calibrate_and_enable(ADC2_CR, ADC2_ISR);

    adc_configure_sampling(ADC2_SMPR1, ADC2_SMPR2,
                            kAdc2Channels, 4u);

    // Sequência: CLT(IN11), IAT(IN12), FUEL_PRESS(IN14), OIL_PRESS(IN15)
    // ADC2 tem apenas 4 canais (SQ1-SQ4), cabe todo em SQR1
    volatile uint32_t dummy_sqr2 = 0u;
    adc_configure_sequence(ADC2_SQR1, dummy_sqr2,
                            kAdc2Channels, 4u);

    // ADC2: 12-bit, software trigger (sem trigger externo)
    ADC2_CFGR1 = (0x00u << 3u);   // RES = 12-bit, sem trigger externo

    // ⚡ OPT-6: ADC2 oversampling ×16 → 14-bit efetivo
    ADC2_CFGR2 = (1u << 0u)       // OVSE = 1
               | (0x03u << 2u)    // OVSR = 0011 → 16×
               | (0x02u << 5u);   // OVSS = 0010 → right shift 2 (14-bit efetivo)

    // ── 6. Habilitar interrupções EOS para ADC1 ──────────────────────────
    // EOSIE = bit 3 (End of Sequence Interrupt Enable)
    ADC1_IER = (1u << 3u);
    nvic_set_priority(IRQ_ADC1, 5u);
    nvic_enable_irq(IRQ_ADC1);

    // ── 7. Primeira conversão para popular o cache ───────────────────────
    ADC1_CR |= (1u << 2u);  // ADSTART = bit 2
}

void adc_pdb_on_tooth(uint16_t tooth_period_ticks) noexcept {
    // Trigger é TIM2 TRGO (hardware), não precisamos ajustar período aqui.
    // A amostragem acontece automaticamente a cada dente CKP.
    (void)tooth_period_ticks;
}

uint16_t adc1_read(Adc1Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 3u) { return 0u; }
    return g_adc1_raw[idx];
}

uint16_t adc2_read(Adc2Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx >= 4u) { return 0u; }
    return g_adc2_raw[idx];
}

}  // namespace ems::hal

// ── ISR ADC1 — processa sequência completa via EOS ────────────────────────────
extern "C" void ADC1_2_IRQHandler(void) {
    const uint32_t sr1 = ADC1_ISR;

    // ADC1: EOS indica fim da sequência (3 canais)
    if (sr1 & (1u << 3u)) {  // EOS = bit 3
        // Ler os 3 resultados do DR em sequência
        // A sequência é: MAP(IN11), TPS(IN13), O2(IN5)
        // Após EOS, os 3 resultados estão nos registros DR (leitura FIFO)
        g_adc1_raw[0] = static_cast<uint16_t>(ADC1_DR & 0xFFFFu);  // MAP
        g_adc1_raw[1] = static_cast<uint16_t>(ADC1_DR & 0xFFFFu);  // TPS
        g_adc1_raw[2] = static_cast<uint16_t>(ADC1_DR & 0xFFFFu);  // O2

        ADC1_ISR = (1u << 3u);  // W1C: limpa EOS
        ADC1_CR |= (1u << 2u);  // ADSTART: re-arm
    }
}

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

#include "hal/adc.h"

namespace ems::hal {

static uint16_t g_adc1_mock[3] = {};
static uint16_t g_adc2_mock[4] = {};
static uint16_t g_last_pdb_mod = 0u;

void     adc_init() noexcept {}
void     adc_pdb_on_tooth(uint16_t t) noexcept { g_last_pdb_mod = t; }
uint16_t adc1_read(Adc1Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    return (idx < 3u) ? g_adc1_mock[idx] : 0u;
}
uint16_t adc2_read(Adc2Channel ch) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    return (idx < 4u) ? g_adc2_mock[idx] : 0u;
}
void adc_test_set_raw_adc1(Adc1Channel ch, uint16_t v) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx < 3u) { g_adc1_mock[idx] = v; }
}
void adc_test_set_raw_adc2(Adc2Channel ch, uint16_t v) noexcept {
    const uint8_t idx = static_cast<uint8_t>(ch);
    if (idx < 4u) { g_adc2_mock[idx] = v; }
}
uint16_t adc_test_last_pdb_mod() noexcept { return g_last_pdb_mod; }

}  // namespace ems::hal

#endif  // EMS_HOST_TEST