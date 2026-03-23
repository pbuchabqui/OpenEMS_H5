#include <cstdio>
#include <cstdint>
#include <cstring>

#define EMS_HOST_TEST 1
#include "engine/fuel_calc.h"
#include "hal/flash_nvm.h"
#include "hal/nvm_error.h"

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

void test_calc_base_pw_zero_inputs() {
    // All zero-guard paths
    TEST(ems::engine::calc_base_pw_us(0u, 100u, 100u, 100u) == 0u);
    TEST(ems::engine::calc_base_pw_us(8000u, 0u, 100u, 100u) == 0u);
    TEST(ems::engine::calc_base_pw_us(8000u, 100u, 100u, 0u) == 0u);
}

void test_calc_base_pw_map_over_300() {
    // MAP > 300 kPa: sensor fault, should return 0
    TEST(ems::engine::calc_base_pw_us(8000u, 100u, 301u, 100u) == 0u);
    // MAP = 300 should still work
    TEST(ems::engine::calc_base_pw_us(8000u, 100u, 300u, 100u) > 0u);
}

void test_calc_base_pw_req_fuel_over_50000() {
    // req_fuel > 50ms: absurd, return 0
    TEST(ems::engine::calc_base_pw_us(50001u, 100u, 100u, 100u) == 0u);
    // req_fuel = 50000 should work
    TEST(ems::engine::calc_base_pw_us(50000u, 100u, 100u, 100u) > 0u);
}

void test_calc_base_pw_saturation() {
    // Max realistic: 50000us * 255 VE * 300 MAP / 100 ref = 382500000 / 100 = way over 100000
    // Should clamp to 100000
    const uint32_t result = ems::engine::calc_base_pw_us(50000u, 255u, 300u, 100u);
    TEST(result <= 100000u);
}

void test_corr_vbatt_below_and_above_range() {
    // Below 9000mV: should clamp to table minimum (1400us dead time)
    const uint16_t low = ems::engine::corr_vbatt(6000u);
    TEST(low == 1400u);

    // Above 16000mV: should clamp to table maximum (600us dead time)
    const uint16_t high = ems::engine::corr_vbatt(18000u);
    TEST(high == 600u);
}

void test_calc_final_pw_zero_base() {
    // Zero base PW should return just dead time
    const uint32_t result = ems::engine::calc_final_pw_us(0u, 256u, 256u, 900u);
    TEST(result == 900u);
}

void test_ltft_round_trip() {
    ems::hal::nvm_test_reset();
    ems::engine::fuel_reset_adaptives();

    // Write a known STFT/LTFT value and verify round-trip through NVM
    // fuel_update_stft will update LTFT cells
    // Direct NVM round-trip:
    TEST(ems::hal::nvm_write_ltft(0u, 0u, 5));
    TEST(ems::hal::nvm_read_ltft(0u, 0u) == 5);

    TEST(ems::hal::nvm_write_ltft(15u, 15u, -12));
    TEST(ems::hal::nvm_read_ltft(15u, 15u) == -12);

    // Out of bounds should return 0
    TEST(ems::hal::nvm_read_ltft(16u, 0u) == 0);
    TEST(ems::hal::nvm_read_ltft(0u, 16u) == 0);
}

void test_stft_open_loop_decay() {
    ems::engine::fuel_reset_adaptives();

    // Force a non-zero STFT by running closed loop
    for (int i = 0; i < 50; ++i) {
        ems::engine::fuel_update_stft(
            3000u, 80u, 1000, 900, 800, true, false, false);
    }
    const int16_t stft_before = ems::engine::fuel_get_stft_pct_x10();
    TEST(stft_before != 0);

    // Open loop (CLT too cold): STFT should decay toward 0
    for (int i = 0; i < 100; ++i) {
        ems::engine::fuel_update_stft(
            3000u, 80u, 1000, 900, 500, true, false, false);
    }
    const int16_t stft_after = ems::engine::fuel_get_stft_pct_x10();
    // Should be closer to 0 than before
    TEST(stft_after == 0 ||
         (stft_after > 0 ? stft_after < stft_before : stft_after > stft_before));
}

}  // namespace

int main() {
    test_calc_base_pw_zero_inputs();
    test_calc_base_pw_map_over_300();
    test_calc_base_pw_req_fuel_over_50000();
    test_calc_base_pw_saturation();
    test_corr_vbatt_below_and_above_range();
    test_calc_final_pw_zero_base();
    test_ltft_round_trip();
    test_stft_open_loop_decay();

    std::printf("\ntests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return g_tests_failed > 0 ? 1 : 0;
}
