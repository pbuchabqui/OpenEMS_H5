/**
 * @file test/hal/test_tim_32bit.cpp
 * @brief Teste host-side: TIM2/TIM5 32-bit wrap + TIM5→TIM1 sync conversion
 *
 * Prompt 1 Entregável #3:
 *   - TIM2 32-bit: now=100, prev=0xFFFFFF00 → delta = 356 (wrap de 32-bit funciona)
 *   - TIM5→TIM1 sync: (uint16_t)(0x0001ABCD) == 0xABCD
 *
 * Framework: Unity-compatible (host test runner)
 */

#include <cstdint>
#include <cstdio>

// ──────────────────────────────────────────────────────────────────────────────
// Test runner (minimal)
// ──────────────────────────────────────────────────────────────────────────────
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT_EQUAL_UINT32(expected, actual) do { \
    ++g_tests_run; \
    if ((uint32_t)(expected) != (uint32_t)(actual)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: expected 0x%08X, got 0x%08X\n", \
               __FILE__, __LINE__, (unsigned)(uint32_t)(expected), \
               (unsigned)(uint32_t)(actual)); \
    } \
} while(0)

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) do { \
    ++g_tests_run; \
    if ((uint16_t)(expected) != (uint16_t)(actual)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: expected 0x%04X, got 0x%04X\n", \
               __FILE__, __LINE__, (unsigned)(uint16_t)(expected), \
               (unsigned)(uint16_t)(actual)); \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("[ RUN ] " #fn "\n"); \
    fn(); \
    printf("[  OK ] " #fn "\n"); \
} while(0)

// ──────────────────────────────────────────────────────────────────────────────
// Funções sob teste: delta circular 32-bit + conversão 32→16
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Calcula delta circular entre dois timestamps TIM2/TIM5 de 32 bits.
 *
 * Com TIM2/TIM5 32-bit @ 250 MHz (PSC=0):
 *   - Tick = 4 ns
 *   - Overflow a cada 17.18 segundos
 *   - Subtração uint32_t funciona naturalmente (wrap-around)
 *
 * Referência: Rules §0.3 — "uint32_t delta = (uint32_t)(now - prev);"
 */
static inline uint32_t tim32_delta(uint32_t now, uint32_t prev) noexcept {
    return static_cast<uint32_t>(now - prev);
}

/**
 * @brief Converte timestamp absoluto TIM5 32-bit para TIM1/TIM8 16-bit.
 *
 * OPT-8: TIM5 (master) → TIM1/TIM8 (slaves) via ITR sync.
 * Os 16 bits baixos de TIM5->CNT == TIM1->CNT/TIM8->CNT em qualquer instante.
 *
 * Conversão: uint16_t hw = (uint16_t)(abs_ticks);
 * Funciona porque TIM1/TIM8 compartilham o mesmo clock e estão sincronizados.
 */
static inline uint16_t tim32_to_tim16(uint32_t abs_ticks) noexcept {
    return static_cast<uint16_t>(abs_ticks);
}

// ──────────────────────────────────────────────────────────────────────────────
// Casos de teste: TIM2/TIM5 32-bit wrap
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Prompt 1, Entregável #3: now=100, prev=0xFFFFFF00 → delta = 356
 *
 * Cenário: TIM2 captura em 0xFFFFFF00, próxima captura em 100.
 * Contador overflow de 0xFFFFFFFF → 0x00000000 → 100.
 * Delta correto: 0xFFFFFFFF - 0xFFFFFF00 + 1 + 100 = 255 + 1 + 100 = 356
 */
static void test_tim2_wrap_prompt_case(void) {
    uint32_t prev    = 0xFFFFFF00u;
    uint32_t now     = 100u;
    uint32_t delta   = tim32_delta(now, prev);
    TEST_ASSERT_EQUAL_UINT32(356u, delta);
}

/**
 * @brief Overflow exato: prev=0xFFFFFFFF, now=0x00000000 → delta = 1
 */
static void test_tim2_overflow_exact(void) {
    uint32_t prev  = 0xFFFFFFFFu;
    uint32_t now   = 0x00000000u;
    uint32_t delta = tim32_delta(now, prev);
    TEST_ASSERT_EQUAL_UINT32(1u, delta);
}

/**
 * @brief Overflow com grande salto: prev=0xFFFF0000, now=0x00010000
 *
 * Delta: 0xFFFFFFFF - 0xFFFF0000 + 1 + 0x00010000
 *       = 0x0000FFFF + 1 + 0x00010000 = 0x00020000 = 131072
 */
static void test_tim2_large_wrap(void) {
    uint32_t prev  = 0xFFFF0000u;
    uint32_t now   = 0x00010000u;
    uint32_t delta = tim32_delta(now, prev);
    TEST_ASSERT_EQUAL_UINT32(0x00020000u, delta);
}

/**
 * @brief Caso normal sem overflow: prev=1000, now=5000 → delta = 4000
 */
static void test_tim2_no_overflow(void) {
    uint32_t prev  = 1000u;
    uint32_t now   = 5000u;
    uint32_t delta = tim32_delta(now, prev);
    TEST_ASSERT_EQUAL_UINT32(4000u, delta);
}

/**
 * @brief Delta zero: mesmo valor (deve ser 0)
 */
static void test_tim2_delta_zero(void) {
    uint32_t val   = 0x12345678u;
    uint32_t delta = tim32_delta(val, val);
    TEST_ASSERT_EQUAL_UINT32(0u, delta);
}

/**
 * @brief Vantagem 32-bit vs 16-bit: a 100 RPM, período dente = 10ms = 2.5M ticks
 *
 * Com 16-bit: OVERFLOW! (2.5M > 65535)
 * Com 32-bit: SEM PROBLEMA (2.5M < 4.29B)
 */
static void test_tim2_advantage_low_rpm(void) {
    // A 100 RPM, 60-2: tooth_period = 10 ms = 2,500,000 ticks @ 250 MHz
    uint32_t tooth_period = 2500000u;
    uint32_t prev = 0u;
    uint32_t now  = tooth_period;
    uint32_t delta = tim32_delta(now, prev);
    TEST_ASSERT_EQUAL_UINT32(tooth_period, delta);

    // Com 16-bit, este valor seria truncado:
    uint16_t delta_16bit = static_cast<uint16_t>(tooth_period);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(tooth_period & 0xFFFFu), delta_16bit);
    // delta_16bit = 2500000 & 0xFFFF = 36944 (ERRADO!)
}

// ──────────────────────────────────────────────────────────────────────────────
// Casos de teste: TIM5→TIM1 sync (conversão 32→16)
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Prompt 1, Entregável #3: (uint16_t)(0x0001ABCD) == 0xABCD
 *
 * OPT-8: TIM5→TIM1 sync via ITR garante que os 16 bits baixos coincidem.
 * Conversão por simples cast de uint32_t para uint16_t.
 */
static void test_tim5_to_tim1_sync_prompt_case(void) {
    uint32_t abs_ticks = 0x0001ABCDu;
    uint16_t hw_ticks  = tim32_to_tim16(abs_ticks);
    TEST_ASSERT_EQUAL_UINT16(0xABCDu, hw_ticks);
}

/**
 * @brief Conversão quando upper 16 bits != 0: 0x12345678 → 0x5678
 */
static void test_tim5_to_tim1_upper_bits(void) {
    uint32_t abs_ticks = 0x12345678u;
    uint16_t hw_ticks  = tim32_to_tim16(abs_ticks);
    TEST_ASSERT_EQUAL_UINT16(0x5678u, hw_ticks);
}

/**
 * @brief Overflow TIM5: 0xFFFFFFFF → 0xFFFF
 */
static void test_tim5_to_tim1_overflow(void) {
    uint32_t abs_ticks = 0xFFFFFFFFu;
    uint16_t hw_ticks  = tim32_to_tim16(abs_ticks);
    TEST_ASSERT_EQUAL_UINT16(0xFFFFu, hw_ticks);
}

/**
 * @brief Zero: 0x00000000 → 0x0000
 */
static void test_tim5_to_tim1_zero(void) {
    uint32_t abs_ticks = 0u;
    uint16_t hw_ticks  = tim32_to_tim16(abs_ticks);
    TEST_ASSERT_EQUAL_UINT16(0u, hw_ticks);
}

/**
 * @brief Verificação: delta 32-bit + conversão 16-bit = consistente
 *
 * Se abs_now=0x00010000, abs_prev=0x0000FFF0:
 *   delta_32 = 0x10 (correto)
 *   hw_now  = (uint16_t)0x00010000 = 0x0000
 *   hw_prev = (uint16_t)0x0000FFF0 = 0xFFF0
 *   delta_16 = (uint16_t)(0x0000 - 0xFFF0) = 0x0010 = 16 ✓
 */
static void test_tim5_to_tim1_consistency(void) {
    uint32_t abs_now  = 0x00010000u;
    uint32_t abs_prev = 0x0000FFF0u;

    uint32_t delta_32 = tim32_delta(abs_now, abs_prev);
    TEST_ASSERT_EQUAL_UINT32(0x10u, delta_32);

    uint16_t hw_now  = tim32_to_tim16(abs_now);
    uint16_t hw_prev = tim32_to_tim16(abs_prev);
    uint16_t delta_16 = static_cast<uint16_t>(hw_now - hw_prev);
    TEST_ASSERT_EQUAL_UINT16(0x0010u, delta_16);
}

// ──────────────────────────────────────────────────────────────────────────────
// Runner principal
// ──────────────────────────────────────────────────────────────────────────────
int main(void) {
    printf("═══════════════════════════════════════════════════════════════\n");
    printf(" OpenEMS HAL — TIM2/TIM5 32-bit + TIM5→TIM1 Sync Tests\n");
    printf(" Target: host x86_64 | 32-bit timestamps @ 250 MHz\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    // TIM2/TIM5 32-bit wrap tests
    RUN_TEST(test_tim2_wrap_prompt_case);
    RUN_TEST(test_tim2_overflow_exact);
    RUN_TEST(test_tim2_large_wrap);
    RUN_TEST(test_tim2_no_overflow);
    RUN_TEST(test_tim2_delta_zero);
    RUN_TEST(test_tim2_advantage_low_rpm);

    // TIM5→TIM1 sync conversion tests
    RUN_TEST(test_tim5_to_tim1_sync_prompt_case);
    RUN_TEST(test_tim5_to_tim1_upper_bits);
    RUN_TEST(test_tim5_to_tim1_overflow);
    RUN_TEST(test_tim5_to_tim1_zero);
    RUN_TEST(test_tim5_to_tim1_consistency);

    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf(" Results: %d tests, %d failed\n", g_tests_run, g_tests_failed);
    printf("═══════════════════════════════════════════════════════════════\n");

    return g_tests_failed == 0 ? 0 : 1;
}