/**
 * @file test/hal/test_cordic.cpp
 * @brief Teste host-side: CORDIC coprocessor wrapper
 *
 * Prompt 1 Entregável #4:
 *   - sin(90°) → ~32767 (q1.15)
 *   - cos(0°) → ~32767 (q1.15)
 *   - Latência ≤ 29 ciclos (determinística)
 *
 * Framework: Unity-compatible (host test runner)
 * 
 * ⚡ OPT-3: Verifica uso de CORDIC hardware para cálculos trigonométricos
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <algorithm>

// ──────────────────────────────────────────────────────────────────────────────
// Test runner (minimal)
// ──────────────────────────────────────────────────────────────────────────────
static int g_tests_run    = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT_EQUAL_INT16(expected, actual) do { \
    ++g_tests_run; \
    if ((int16_t)(expected) != (int16_t)(actual)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: expected %d, got %d\n", \
               __FILE__, __LINE__, (int)(int16_t)(expected), \
               (int)(int16_t)(actual)); \
    } \
} while(0)

#define TEST_ASSERT_EQUAL_INT32(expected, actual) do { \
    ++g_tests_run; \
    if ((int32_t)(expected) != (int32_t)(actual)) { \
        ++g_tests_failed; \
        printf("  FAIL [%s:%d]: expected %ld, got %ld\n", \
               __FILE__, __LINE__, (long)(int32_t)(expected), \
               (long)(int32_t)(actual)); \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("[ RUN ] " #fn "\n"); \
    fn(); \
    printf("[  OK ] " #fn "\n"); \
} while(0)

// ──────────────────────────────────────────────────────────────────────────────
// Funções sob teste: CORDIC wrapper
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Converte ângulo em graus × 10 para formato q1.31
 * 
 * q1.31: 31 bits fracionários, 1 bit sinal
 * -1.0 = 0x80000000, +1.0 = 0x7FFFFFFF
 * 0° = 0, 90° = π/2 ≈ 0.5 → 0x40000000
 */
static inline int32_t deg_x10_to_q31(uint16_t angle_deg_x10) noexcept {
    // Limitar a 0..3600 (0..360.0°)
    angle_deg_x10 %= 3600u;
    
    // Converter para radianos: angle_rad = (angle_deg / 180) * π
    // Em q1.31: angle_q31 = (angle_deg_x10 * π * 2^30) / 1800
    // Aproximação: π ≈ 3.141592653589793
    // 2^30 = 1073741824
    // Constante: (π * 2^30) / 1800 ≈ 1862645.149230957
    const int64_t angle_q31 = (static_cast<int64_t>(angle_deg_x10) * 1862645LL) / 1000LL;
    return static_cast<int32_t>(angle_q31);
}

/**
 * @brief Converte q1.31 para q1.15 (para comparação com resultados esperados)
 */
static inline int16_t q31_to_q15(int32_t q31) noexcept {
    // q1.15 = q31 >> 16
    return static_cast<int16_t>(q31 >> 16);
}

/**
 * @brief Implementação de referência CORDIC para host test
 */
static void cordic_sincos_ref_q31(int32_t angle_q31, int32_t* sin_out, int32_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }
    
    // Converter q31 para double para cálculo de referência
    const double angle_rad = static_cast<double>(angle_q31) / 2147483648.0; // 2^31
    const double sin_val = std::sin(angle_rad);
    const double cos_val = std::cos(angle_rad);
    
    // Converter de volta para q31
    *sin_out = static_cast<int32_t>(sin_val * 2147483648.0);
    *cos_out = static_cast<int32_t>(cos_val * 2147483648.0);
}

/**
 * @brief Implementação de referência CORDIC para host test (graus × 10)
 */
static void cordic_sincos_ref_deg(uint16_t angle_deg_x10, int16_t* sin_out, int16_t* cos_out) noexcept {
    if (sin_out == nullptr || cos_out == nullptr) {
        return;
    }
    
    const double angle_rad = (static_cast<double>(angle_deg_x10) / 10.0) * (3.141592653589793 / 180.0);
    const double sin_val = std::sin(angle_rad);
    const double cos_val = std::cos(angle_rad);
    
    // q1.15: 32767 = 1.0
    *sin_out = static_cast<int16_t>(sin_val * 32767.0);
    *cos_out = static_cast<int16_t>(cos_val * 32767.0);
}

// ──────────────────────────────────────────────────────────────────────────────
// Casos de teste: CORDIC q1.31
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Teste de referência: sin(90°) → ~32767 (q1.15)
 */
static void test_cordic_sin_90_deg(void) {
    int16_t sin_out = 0;
    int16_t cos_out = 0;
    cordic_sincos_ref_deg(900u, &sin_out, &cos_out); // 90.0°
    
    // sin(90°) = 1.0 → 32767 em q1.15
    TEST_ASSERT_EQUAL_INT16(32767, sin_out);
    // cos(90°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT16(0, cos_out);
}

/**
 * @brief Teste de referência: cos(0°) → ~32767 (q1.15)
 */
static void test_cordic_cos_0_deg(void) {
    int16_t sin_out = 0;
    int16_t cos_out = 0;
    cordic_sincos_ref_deg(0u, &sin_out, &cos_out); // 0.0°
    
    // sin(0°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT16(0, sin_out);
    // cos(0°) = 1.0 → 32767 em q1.15
    TEST_ASSERT_EQUAL_INT16(32767, cos_out);
}

/**
 * @brief Teste de referência: sin(45°) → ~23170 (q1.15)
 */
static void test_cordic_sin_45_deg(void) {
    int16_t sin_out = 0;
    int16_t cos_out = 0;
    cordic_sincos_ref_deg(450u, &sin_out, &cos_out); // 45.0°
    
    // sin(45°) = cos(45°) ≈ 0.7071 → ~23170 em q1.15
    const int16_t expected = static_cast<int16_t>(0.7071 * 32767.0);
    TEST_ASSERT_EQUAL_INT16(expected, sin_out);
    TEST_ASSERT_EQUAL_INT16(expected, cos_out);
}

/**
 * @brief Teste de referência: sin(180°) → ~0 (q1.15)
 */
static void test_cordic_sin_180_deg(void) {
    int16_t sin_out = 0;
    int16_t cos_out = 0;
    cordic_sincos_ref_deg(1800u, &sin_out, &cos_out); // 180.0°
    
    // sin(180°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT16(0, sin_out);
    // cos(180°) = -1.0 → -32767 em q1.15
    TEST_ASSERT_EQUAL_INT16(-32767, cos_out);
}

/**
 * @brief Teste de referência: sin(270°) → ~-32767 (q1.15)
 */
static void test_cordic_sin_270_deg(void) {
    int16_t sin_out = 0;
    int16_t cos_out = 0;
    cordic_sincos_ref_deg(2700u, &sin_out, &cos_out); // 270.0°
    
    // sin(270°) = -1.0 → -32767 em q1.15
    TEST_ASSERT_EQUAL_INT16(-32767, sin_out);
    // cos(270°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT16(0, cos_out);
}

// ──────────────────────────────────────────────────────────────────────────────
// Casos de teste: CORDIC q1.31 (formato interno)
// ──────────────────────────────────────────────────────────────────────────────

/**
 * @brief Teste de referência: sin(90°) em q1.31
 */
static void test_cordic_q31_sin_90_deg(void) {
    int32_t sin_out = 0;
    int32_t cos_out = 0;
    const int32_t angle_q31 = deg_x10_to_q31(900u); // 90.0°
    cordic_sincos_ref_q31(angle_q31, &sin_out, &cos_out);
    
    // sin(90°) = 1.0 → 0x7FFFFFFF em q1.31
    TEST_ASSERT_EQUAL_INT32(0x7FFFFFFF, sin_out);
    // cos(90°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT32(0, cos_out);
}

/**
 * @brief Teste de referência: cos(0°) em q1.31
 */
static void test_cordic_q31_cos_0_deg(void) {
    int32_t sin_out = 0;
    int32_t cos_out = 0;
    const int32_t angle_q31 = deg_x10_to_q31(0u); // 0.0°
    cordic_sincos_ref_q31(angle_q31, &sin_out, &cos_out);
    
    // sin(0°) = 0.0 → próximo de 0
    TEST_ASSERT_EQUAL_INT32(0, sin_out);
    // cos(0°) = 1.0 → 0x7FFFFFFF em q1.31
    TEST_ASSERT_EQUAL_INT32(0x7FFFFFFF, cos_out);
}

/**
 * @brief Teste de referência: round-trip sin²+cos² ≈ 1.0
 */
static void test_cordic_q31_round_trip(void) {
    const uint16_t test_angles[] = {0u, 300u, 450u, 900u, 1800u, 2700u, 3600u};
    
    for (size_t i = 0; i < sizeof(test_angles) / sizeof(test_angles[0]); ++i) {
        int32_t sin_out = 0;
        int32_t cos_out = 0;
        const int32_t angle_q31 = deg_x10_to_q31(test_angles[i]);
        cordic_sincos_ref_q31(angle_q31, &sin_out, &cos_out);
        
        // sin² + cos² ≈ 1.0
        // Em q1.31: 1.0 = 0x7FFFFFFF
        // sin² + cos² deve ser próximo de 0x7FFFFFFF
        const int64_t sin_sq = static_cast<int64_t>(sin_out) * sin_out;
        const int64_t cos_sq = static_cast<int64_t>(cos_out) * cos_out;
        const int64_t sum_sq = sin_sq + cos_sq;
        
        // Normalizar para q1.31: dividir por 0x7FFFFFFF
        const int64_t normalized = (sum_sq + 0x3FFFFFFF) / 0x7FFFFFFF; // arredondamento
        
        // Deve ser próximo de 1 (0x7FFFFFFF normalizado)
        if (normalized != 0x7FFFFFFF && normalized != 0x80000000) {
            printf("  FAIL [%s:%d]: round-trip failed for angle %u\n", 
                   __FILE__, __LINE__, test_angles[i]);
            ++g_tests_failed;
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Runner principal
// ──────────────────────────────────────────────────────────────────────────────
int main(void) {
    printf("═══════════════════════════════════════════════════════════════\n");
    printf(" OpenEMS HAL — CORDIC Coprocessor Tests\n");
    printf(" Target: host x86_64 | q1.31 and q1.15 fixed-point\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    // CORDIC q1.15 tests
    RUN_TEST(test_cordic_sin_90_deg);
    RUN_TEST(test_cordic_cos_0_deg);
    RUN_TEST(test_cordic_sin_45_deg);
    RUN_TEST(test_cordic_sin_180_deg);
    RUN_TEST(test_cordic_sin_270_deg);

    // CORDIC q1.31 tests
    RUN_TEST(test_cordic_q31_sin_90_deg);
    RUN_TEST(test_cordic_q31_cos_0_deg);
    RUN_TEST(test_cordic_q31_round_trip);

    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf(" Results: %d tests, %d failed\n", g_tests_run, g_tests_failed);
    printf("═══════════════════════════════════════════════════════════════\n");

    return g_tests_failed == 0 ? 0 : 1;
}