#include "app/can_stack.h"

#include <cstdint>

#include "hal/fdcan.h"
#include "util/clamp.h"

namespace {

static uint16_t g_wbo2_rx_id      = 0x180u;
static uint16_t g_lambda_milli    = ems::app::WBO2_SAFE_LAMBDA_MILLI;
static uint8_t  g_wbo2_status     = 0u;
static uint32_t g_wbo2_last_rx_ms = 0u;
static bool     g_wbo2_seen       = false;
static bool     g_wbo2_fault      = true;
static uint32_t g_last_tx_400_ms  = 0u;

using ems::util::saturate_u16;
using ems::util::saturate_i16;
using ems::util::elapsed;

// Crank angle per tooth in millidegrees for a 60-2 wheel:
//   360° / 58 teeth × 1000 = 6206.897 ≈ 6207 millidegrees/tooth
static constexpr uint32_t kCrankAnglePerToothMdeg = 6207u;

inline void write_u16_le(uint8_t* dst, uint16_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFFu);
    dst[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
}

inline void write_u32_le(uint8_t* dst, uint32_t v) noexcept {
    dst[0] = static_cast<uint8_t>(v & 0xFFu);
    dst[1] = static_cast<uint8_t>((v >> 8u) & 0xFFu);
    dst[2] = static_cast<uint8_t>((v >> 16u) & 0xFFu);
    dst[3] = static_cast<uint8_t>((v >> 24u) & 0xFFu);
}

inline bool wbo2_timed_out(uint32_t now_ms) noexcept {
    if (!g_wbo2_seen) { return true; }
    return static_cast<uint32_t>(now_ms - g_wbo2_last_rx_ms) > 500u;
}

inline void process_rx(uint32_t now_ms) noexcept {
    ems::hal::FdcanFrame frame = {};
    while (ems::hal::fdcan_rx_pop(frame)) {
        if (frame.extended || frame.id != g_wbo2_rx_id || frame.dlc_bytes < 3u) {
            continue;
        }
        g_lambda_milli    = static_cast<uint16_t>(
            frame.data[0] | (static_cast<uint16_t>(frame.data[1]) << 8u));
        g_wbo2_status     = frame.data[2];
        g_wbo2_last_rx_ms = now_ms;
        g_wbo2_seen       = true;
        g_wbo2_fault      = false;
    }
}

inline void tx_0x400(const ems::drv::CkpSnapshot& ckp,
                     const ems::drv::SensorData& sensors,
                     int8_t advance_deg,
                     uint8_t pw_ms_x10,
                     int8_t stft_pct,
                     uint8_t status_bits,
                     uint32_t now_ms) noexcept {
    ems::hal::FdcanFrame out = {};
    out.id = 0x400u;
    out.dlc_bytes = 48u;
    out.extended = false;
    out.brs = false;

    const uint8_t effective_status = g_wbo2_fault
        ? static_cast<uint8_t>(status_bits | ems::app::STATUS_WBO2_FAULT)
        : static_cast<uint8_t>(status_bits & static_cast<uint8_t>(~ems::app::STATUS_WBO2_FAULT));

    const uint16_t rpm = saturate_u16(ckp.rpm_x10 / 10u);
    const uint16_t advance_x10 =
        static_cast<uint16_t>(saturate_i16(static_cast<int32_t>(advance_deg) * 10));
    const uint16_t pw_us = static_cast<uint16_t>(pw_ms_x10 * 100u);
    const uint16_t lambda_x1000 = ems::app::can_stack_lambda_milli_safe(now_ms);
    const int16_t stft_pct_x10 =
        saturate_i16(static_cast<int32_t>(stft_pct) * 10);
    const uint32_t abs_crank_angle = static_cast<uint32_t>(ckp.tooth_index) * kCrankAnglePerToothMdeg;

    write_u16_le(&out.data[0], rpm);
    write_u16_le(&out.data[2], sensors.map_kpa_x10);
    write_u16_le(&out.data[4], sensors.tps_pct_x10);
    write_u16_le(&out.data[6], static_cast<uint16_t>(sensors.clt_degc_x10));
    write_u16_le(&out.data[8], static_cast<uint16_t>(sensors.iat_degc_x10));
    write_u16_le(&out.data[10], advance_x10);
    write_u16_le(&out.data[12], pw_us);
    write_u16_le(&out.data[14], lambda_x1000);
    write_u16_le(&out.data[16], static_cast<uint16_t>(stft_pct_x10));
    write_u16_le(&out.data[18], sensors.fuel_press_kpa_x10);
    write_u16_le(&out.data[20], sensors.oil_press_kpa_x10);
    write_u16_le(&out.data[22], sensors.vbatt_mv);
    write_u32_le(&out.data[24], abs_crank_angle);
    write_u32_le(&out.data[28], ckp.tooth_period_ticks);
    out.data[32] = 0u;
    out.data[33] = 0u;
    out.data[34] = 0u;
    out.data[35] = 0u;
    out.data[36] = effective_status;
    out.data[37] = sensors.fault_bits;
    write_u32_le(&out.data[38], now_ms);
    for (uint8_t i = 42u; i < 48u; ++i) {
        out.data[i] = 0u;
    }

    static_cast<void>(ems::hal::fdcan_tx(out));
}

}  // namespace

namespace ems::app {

void can_stack_init(uint16_t wbo2_rx_id) noexcept {
    g_wbo2_rx_id      = static_cast<uint16_t>(wbo2_rx_id & 0x7FFu);
    g_lambda_milli    = WBO2_SAFE_LAMBDA_MILLI;
    g_wbo2_status     = 0u;
    g_wbo2_last_rx_ms = 0u;
    g_wbo2_seen       = false;
    g_wbo2_fault      = true;
    g_last_tx_400_ms  = 0u;
    ems::hal::fdcan_init();
}

void can_stack_set_wbo2_rx_id(uint16_t id) noexcept {
    g_wbo2_rx_id = static_cast<uint16_t>(id & 0x7FFu);
}

void can_stack_process(uint32_t now_ms,
                       const ems::drv::CkpSnapshot& ckp,
                       const ems::drv::SensorData& sensors,
                       int8_t advance_deg,
                       uint8_t pw_ms_x10,
                       int8_t stft_pct,
                       uint8_t vvt_intake_pct,
                       uint8_t vvt_exhaust_pct,
                       uint8_t status_bits) noexcept {
    (void)vvt_intake_pct;
    (void)vvt_exhaust_pct;

    process_rx(now_ms);
    g_wbo2_fault = wbo2_timed_out(now_ms);

    if (elapsed(now_ms, g_last_tx_400_ms, 10u)) {
        g_last_tx_400_ms = now_ms;
        tx_0x400(ckp, sensors, advance_deg, pw_ms_x10, stft_pct, status_bits, now_ms);
    }
}

uint16_t can_stack_lambda_milli() noexcept {
    return g_lambda_milli;
}

uint16_t can_stack_lambda_milli_safe(uint32_t now_ms) noexcept {
    if (wbo2_timed_out(now_ms)) {
        return WBO2_SAFE_LAMBDA_MILLI;
    }
    return g_lambda_milli;
}

bool can_stack_wbo2_fresh(uint32_t now_ms) noexcept {
    return !wbo2_timed_out(now_ms);
}

bool can_stack_wbo2_fault() noexcept {
    return g_wbo2_fault;
}

uint8_t can_stack_wbo2_status() noexcept {
    return g_wbo2_status;
}

#if defined(EMS_HOST_TEST)
void can_stack_test_reset() noexcept {
    can_stack_init(0x180u);
}
#endif

}  // namespace ems::app
