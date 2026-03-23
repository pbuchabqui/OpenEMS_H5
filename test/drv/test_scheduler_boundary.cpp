#include <cstdio>
#include <cstdint>

#define EMS_HOST_TEST 1
#include "drv/scheduler.h"
#include "hal/tim.h"

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

void reset() {
    ems::drv::sched_init();
    ems::hal::tim_test_clear_all();
}

void test_past_event_rejected() {
    reset();
    // Set TIM5 counter to 1000
    ems::hal::tim_test_set_counter(5u, 1000u);

    // Schedule in the past (now - 1)
    const bool ok = ems::drv::sched_event(
        ems::drv::Channel::INJ1, 999u, ems::drv::Action::SET);
    TEST(!ok);
    TEST(!ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
}

void test_future_event_accepted() {
    reset();
    ems::hal::tim_test_set_counter(5u, 1000u);

    // Schedule in the future (now + 100)
    const bool ok = ems::drv::sched_event(
        ems::drv::Channel::INJ1, 1100u, ems::drv::Action::SET);
    TEST(ok);
    TEST(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
}

void test_far_future_event_accepted() {
    reset();
    ems::hal::tim_test_set_counter(5u, 1000u);

    // Schedule at max positive delta (0x7FFFFFFF from now)
    const uint32_t target = 1000u + 0x7FFFFFFFu;
    const bool ok = ems::drv::sched_event(
        ems::drv::Channel::IGN1, target, ems::drv::Action::SET);
    TEST(ok);
    TEST(ems::drv::sched_test_is_armed(ems::drv::Channel::IGN1));
}

void test_wrap_around_future() {
    reset();
    // Counter near wrap: 0xFFFFFFF0
    ems::hal::tim_test_set_counter(5u, 0xFFFFFFF0u);

    // Target wraps around to 0x00000010 (delta = 0x20, which is in the future)
    const bool ok = ems::drv::sched_event(
        ems::drv::Channel::INJ2, 0x00000010u, ems::drv::Action::SET);
    TEST(ok);
    TEST(ems::drv::sched_test_is_armed(ems::drv::Channel::INJ2));
}

void test_cancel_unarmed_channel() {
    reset();
    // Cancel a channel that was never armed — should not crash
    ems::drv::sched_cancel(ems::drv::Channel::IGN4);
    TEST(!ems::drv::sched_test_is_armed(ems::drv::Channel::IGN4));
}

void test_cancel_all() {
    reset();
    ems::hal::tim_test_set_counter(5u, 0u);

    ems::drv::sched_event(ems::drv::Channel::INJ1, 100u, ems::drv::Action::SET);
    ems::drv::sched_event(ems::drv::Channel::IGN1, 200u, ems::drv::Action::SET);

    ems::drv::sched_cancel_all();
    TEST(!ems::drv::sched_test_is_armed(ems::drv::Channel::INJ1));
    TEST(!ems::drv::sched_test_is_armed(ems::drv::Channel::IGN1));
}

void test_all_channels_schedulable() {
    reset();
    ems::hal::tim_test_set_counter(5u, 0u);

    for (uint8_t i = 0; i < 8; ++i) {
        const auto ch = static_cast<ems::drv::Channel>(i);
        const bool ok = ems::drv::sched_event(ch, 1000u + i, ems::drv::Action::SET);
        TEST(ok);
        TEST(ems::drv::sched_test_is_armed(ch));
    }
}

}  // namespace

int main() {
    test_past_event_rejected();
    test_future_event_accepted();
    test_far_future_event_accepted();
    test_wrap_around_future();
    test_cancel_unarmed_channel();
    test_cancel_all();
    test_all_channels_schedulable();

    std::printf("\ntests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return g_tests_failed > 0 ? 1 : 0;
}
