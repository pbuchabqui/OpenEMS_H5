#include <cassert>
#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "drv/ckp.h"

extern volatile uint32_t ems_test_ckp_capture_ch0;
extern volatile uint32_t ems_test_ckp_capture_ch1;
extern volatile uint32_t ems_test_ckp_gpio_idr;

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;
// TIM2 de captura é 32-bit; o mock acompanha esse contrato.
// O ISR agora usa aritmética uint32_t; g_capture deve acumular sem overflow de 16 bits.
uint32_t g_capture = 0u;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

void test_reset() {
    ems::drv::ckp_test_reset();
    g_capture = 0u;
}

void feed_ckp(uint32_t period_ticks) {
    ems_test_ckp_gpio_idr = (1u << 0u);
    g_capture += period_ticks;  // acumulação uint32_t sem overflow artificial
    ems_test_ckp_capture_ch0 = g_capture;
    ems::drv::ckp_capture_primary_isr();
}

void feed_cam_edge() {
    ems_test_ckp_gpio_idr = (1u << 1u);
    ems::drv::ckp_capture_secondary_isr();
}

void sync_with_two_gaps() {
    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);

    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);
}

void sync_with_one_gap() {
    for (int i = 0; i < 58; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);
}

void test_sync_after_two_gaps() {
    test_reset();
    sync_with_two_gaps();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::SYNCED);
}

void test_false_gap_ignored_before_tooth55() {
    test_reset();
    for (int i = 0; i < 20; ++i) {
        feed_ckp(1000u);
    }
    feed_ckp(1600u);

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::WAIT);
}

void test_rpm_formula() {
    test_reset();
    // 800 RPM × 10 = 8000. Com tick de 4 ns:
    // rpm_x10 = 25862068 / period_ticks
    // Para 8000 rpm_x10: period_ticks = 25862068 / 8000 = 3232.76 ≈ 3233 ticks
    const uint32_t rpm_x10 = ems::drv::ckp_test_rpm_x10_from_period_ticks(3233u);
    TEST_ASSERT_TRUE(rpm_x10 >= 7990u && rpm_x10 <= 8010u);
}

void test_circular_subtraction() {
    test_reset();
    ems_test_ckp_gpio_idr = (1u << 0u);
    // Wrap test (32-bit): 0x00000000 - 0xFFFFFF9C = 100 ticks (unsigned wrap).
    // (uint32_t)(0 - 0xFFFFFF9C) = 100 ✓  (0xFFFFFF9C = UINT32_MAX - 99)
    // Testa aritmética circular uint32_t no wrap natural do TIM2 32-bit.
    ems_test_ckp_capture_ch0 = 0xFFFFFF9Cu;
    ems::drv::ckp_capture_primary_isr();  // delta = 0xFFFFFF9C (from 0, bootstrap accepted)

    ems_test_ckp_capture_ch0 = 0u;
    ems::drv::ckp_capture_primary_isr();  // delta = (uint32_t)(0 - 0xFFFFFF9C) = 100 ticks

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_EQ_U32(100u, snap.tooth_period_ticks);
}

void test_tooth_count_over_60_loses_sync() {
    test_reset();
    sync_with_two_gaps();

    auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::SYNCED);

    // kMaxTeethBeforeLoss aumentado de 60 para 63 (SCH-04 / margem de desaceleração).
    // Precisa de 64 dentes sem gap para disparar LOSS_OF_SYNC.
    for (int i = 0; i < 64; ++i) {
        feed_ckp(1000u);
    }

    snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::LOSS_OF_SYNC);
}

// P7: RPM deve ser zero antes de qualquer dente
void test_rpm_zero_before_sync() {
    test_reset();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_EQ_U32(0u, snap.rpm_x10);
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::WAIT);
}

// P7: phase_A deve alternar a cada disparo de CH1 (cam sensor)
void test_phase_a_toggles_on_ch1() {
    test_reset();
    sync_with_two_gaps();

    const auto snap0 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap0.state == ems::drv::SyncState::SYNCED);
    const bool phase_initial = snap0.phase_A;

    // Simular rising edge no cam sensor (CH1, bit 1 de GPIOD)
    ems_test_ckp_gpio_idr = (1u << 1u);
    ems::drv::ckp_capture_secondary_isr();

    const auto snap1 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap1.phase_A != phase_initial);

    // Segundo disparo: deve voltar ao valor original
    ems::drv::ckp_capture_secondary_isr();
    const auto snap2 = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap2.phase_A == phase_initial);
}

// P7: tooth_period_ticks deve ser não-zero após ao menos um par de dentes
void test_tooth_period_nonzero_after_two_teeth() {
    test_reset();
    feed_ckp(1000u);  // primeiro dente — sem período ainda
    feed_ckp(1000u);  // segundo dente — período calculado

    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.tooth_period_ticks > 0u);
}

void test_seeded_fast_reacquire_promotes_on_first_gap() {
    test_reset();
    ems::drv::ckp_seed_arm(true);
    sync_with_one_gap();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::SYNCED);
    TEST_ASSERT_TRUE(snap.phase_A == true);
    TEST_ASSERT_EQ_U32(1u, ems::drv::ckp_seed_loaded_count());
}

void test_seeded_fast_reacquire_confirms_on_cam_edge() {
    test_reset();
    ems::drv::ckp_seed_arm(false);
    sync_with_one_gap();
    TEST_ASSERT_EQ_U32(0u, ems::drv::ckp_seed_confirmed_count());
    feed_cam_edge();
    TEST_ASSERT_EQ_U32(1u, ems::drv::ckp_seed_confirmed_count());
    TEST_ASSERT_EQ_U32(0u, ems::drv::ckp_seed_rejected_count());
}

void test_seeded_fast_reacquire_rejects_without_cam() {
    test_reset();
    ems::drv::ckp_seed_arm(false);
    sync_with_one_gap();
    for (int i = 0; i < 71; ++i) {
        feed_ckp(1000u);
    }
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::SYNCING);
    TEST_ASSERT_EQ_U32(1u, ems::drv::ckp_seed_rejected_count());
}

void test_seed_disarm_blocks_seeded_promotion() {
    test_reset();
    ems::drv::ckp_seed_arm(true);
    ems::drv::ckp_seed_disarm();
    sync_with_one_gap();
    const auto snap = ems::drv::ckp_snapshot();
    TEST_ASSERT_TRUE(snap.state == ems::drv::SyncState::SYNCING);
    TEST_ASSERT_EQ_U32(1u, ems::drv::ckp_seed_loaded_count());
}

}  // namespace

int main() {
    test_sync_after_two_gaps();
    test_false_gap_ignored_before_tooth55();
    test_rpm_formula();
    test_circular_subtraction();
    test_tooth_count_over_60_loses_sync();
    test_rpm_zero_before_sync();
    test_phase_a_toggles_on_ch1();
    test_tooth_period_nonzero_after_two_teeth();
    test_seeded_fast_reacquire_promotes_on_first_gap();
    test_seeded_fast_reacquire_confirms_on_cam_edge();
    test_seeded_fast_reacquire_rejects_without_cam();
    test_seed_disarm_blocks_seeded_promotion();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
