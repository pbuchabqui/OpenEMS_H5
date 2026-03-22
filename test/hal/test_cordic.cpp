#include <cstdint>
#include <cstdio>
#include <cstdlib>

#define EMS_HOST_TEST 1
#include "hal/cordic.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

void test_cardinals_q15() {
    int16_t s = 0;
    int16_t c = 0;
    ems::hal::cordic_sincos_deg(0u, &s, &c);
    TEST_ASSERT_TRUE(std::abs(c - 32767) <= 1);
    TEST_ASSERT_TRUE(std::abs(s) <= 1);

    ems::hal::cordic_sincos_deg(900u, &s, &c);
    TEST_ASSERT_TRUE(std::abs(s - 32767) <= 1);
    TEST_ASSERT_TRUE(std::abs(c) <= 1);
}

void test_q31_quadrature() {
    int32_t s = 0;
    int32_t c = 0;
    ems::hal::cordic_sincos_q31(0x40000000, &s, &c);
    TEST_ASSERT_TRUE(s > 2000000000);
    TEST_ASSERT_TRUE(std::abs(c) < 20000000);
}

}  // namespace

int main() {
    ems::hal::cordic_init();
    test_cardinals_q15();
    test_q31_quadrature();
    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
