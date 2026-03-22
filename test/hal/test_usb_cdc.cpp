#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "hal/usb_cdc.h"
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::drv {

static CkpSnapshot g_ckp = {0u, 0u, 0u, 0u, SyncState::WAIT, false};
static SensorData g_sensors = {0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u};

CkpSnapshot ckp_snapshot() noexcept {
    return g_ckp;
}

const SensorData& sensors_get() noexcept {
    return g_sensors;
}

}  // namespace ems::drv

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

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

void test_tx_drain() {
    ems::hal::usb_cdc_init();
    TEST_ASSERT_TRUE(ems::hal::usb_cdc_tx_byte('O'));
    TEST_ASSERT_TRUE(ems::hal::usb_cdc_tx_byte('K'));
    uint8_t out[8] = {};
    const uint16_t n = ems::hal::usb_cdc_test_drain_tx(out, sizeof(out));
    TEST_ASSERT_EQ_U32(2u, n);
    TEST_ASSERT_EQ_U32('O', out[0]);
    TEST_ASSERT_EQ_U32('K', out[1]);
}

void test_rx_feeds_tunerstudio() {
    ems::hal::usb_cdc_init();
    ems::app::ts_test_reset();
    const uint8_t q = static_cast<uint8_t>('Q');
    ems::hal::usb_cdc_test_feed_rx(&q, 1u);
    ems::hal::usb_cdc_poll_rx(8u);
    ems::app::ts_process();

    uint8_t out = 0u;
    uint16_t count = 0u;
    while (ems::app::ts_tx_pop(out)) {
        ++count;
    }
    TEST_ASSERT_TRUE(count > 0u);
}

}  // namespace

int main() {
    test_tx_drain();
    test_rx_feeds_tunerstudio();
    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
