#include "engine/knock.h"

#include <cstdint>

#include "hal/dac.h"
#include "hal/flash_nvm.h"
#include "util/clamp.h"

namespace {

constexpr uint8_t  kDefaultEventThreshold = 3u;
constexpr uint16_t kRetardStepX10         = 20u;   // +2.0 deg per event above threshold
constexpr uint16_t kRetardMaxX10          = 100u;  // 10.0 deg max retard
constexpr uint16_t kRecoveryStepX10       = 1u;    // -0.1 deg per clean cycle
constexpr uint8_t  kRecoveryDelayCycles   = 10u;   // start recovery after 10 clean cycles

// 12-bit DAC threshold constants (STM32H5 DAC1_OUT2 on PA5)
// Scaled proportionally from the original 6-bit MK64F CMP0 VOSEL (0-63)
// kThresholdMax=4095, kThresholdDefault=2048 ≈ midpoint
// kThresholdStepUp=130 ≈ 2/63 × 4095 (raise threshold = less sensitive after knock)
// kThresholdStepDown=65 ≈ 1/63 × 4095 (lower threshold = more sensitive after clean cycles)
constexpr uint16_t kThresholdMax     = 4095u;
constexpr uint16_t kThresholdDefault = 2048u;
constexpr uint16_t kThresholdStepUp  = 130u;
constexpr uint16_t kThresholdStepDown = 65u;

struct KnockState {
    uint8_t  knock_count[ems::engine::kKnockCylinders];
    uint8_t  clean_cycles[ems::engine::kKnockCylinders];
    uint8_t  event_threshold;
    uint16_t dac_threshold;    // 12-bit DAC value (0..4095)
    uint8_t  global_clean_cycles;
    uint8_t  window_cyl;
    bool     window_active;
};

static KnockState g = {};

using ems::util::clamp_u16;

void set_dac_threshold(uint16_t threshold) noexcept {
    g.dac_threshold = (threshold > kThresholdMax) ? kThresholdMax : threshold;
    ems::hal::dac_set_knock_threshold(g.dac_threshold);
}

}  // namespace

namespace ems::engine {

volatile uint16_t knock_retard_x10[kKnockCylinders] = {};

void knock_init() noexcept {
    g = {};
    g.event_threshold = kDefaultEventThreshold;

    // Load persisted retard from NVM: rpm_i=0, load_i=cyl (4 cells)
    for (uint8_t i = 0u; i < kKnockCylinders; ++i) {
        const int8_t stored = ems::hal::nvm_read_knock(0u, i);
        knock_retard_x10[i] = (stored > 0)
            ? clamp_u16(static_cast<uint16_t>(stored), 0u, kRetardMaxX10)
            : 0u;
    }

    // Restore persisted 12-bit DAC threshold:
    // high byte at nvm_read_knock(1, 0), low byte at nvm_read_knock(1, 1)
    const int8_t thr_hi = ems::hal::nvm_read_knock(1u, 0u);
    const int8_t thr_lo = ems::hal::nvm_read_knock(1u, 1u);
    const uint16_t stored_thr = (static_cast<uint16_t>(static_cast<uint8_t>(thr_hi)) << 8)
                               | static_cast<uint16_t>(static_cast<uint8_t>(thr_lo));
    const uint16_t init_thr = (stored_thr > 0u && stored_thr <= kThresholdMax)
        ? stored_thr
        : kThresholdDefault;
    set_dac_threshold(init_thr);
}

void knock_save_to_nvm() noexcept {
    for (uint8_t i = 0u; i < kKnockCylinders; ++i) {
        // knock_retard_x10 is in ×10 (0.1°). nvm_write_knock expects int8_t deci-degrees.
        const int8_t val = static_cast<int8_t>(
            knock_retard_x10[i] > 127u ? 127u : knock_retard_x10[i]);
        ems::hal::nvm_write_knock(0u, i, val);
    }
    // Persist 12-bit DAC threshold as two int8_t NVM slots (rpm_i=1, load_i=0 and 1)
    const uint8_t thr_hi = static_cast<uint8_t>(g.dac_threshold >> 8);
    const uint8_t thr_lo = static_cast<uint8_t>(g.dac_threshold & 0xFFu);
    ems::hal::nvm_write_knock(1u, 0u, static_cast<int8_t>(thr_hi));
    ems::hal::nvm_write_knock(1u, 1u, static_cast<int8_t>(thr_lo));
}

void knock_set_event_threshold(uint8_t threshold) noexcept {
    g.event_threshold = threshold;
}

void knock_window_open(uint8_t cyl) noexcept {
    g.window_cyl = static_cast<uint8_t>(cyl & 0x3u);
    g.window_active = true;
    // DAC is always enabled after dac_init(); knock_cmp0_isr() guards via window_active
}

void knock_window_close(uint8_t cyl) noexcept {
    if ((g.window_cyl == static_cast<uint8_t>(cyl & 0x3u)) && g.window_active) {
        g.window_active = false;
    }
}

void knock_cmp0_isr() noexcept {
    if (!g.window_active) {
        return;
    }

    const uint8_t cyl = g.window_cyl;
    if (g.knock_count[cyl] < 255u) {
        ++g.knock_count[cyl];
    }
}

void knock_cycle_complete(uint8_t cyl) noexcept {
    const uint8_t c = static_cast<uint8_t>(cyl & 0x3u);
    const uint8_t count = g.knock_count[c];

    if (count > g.event_threshold) {
        const uint16_t next = static_cast<uint16_t>(knock_retard_x10[c] + kRetardStepX10);
        knock_retard_x10[c] = clamp_u16(next, 0u, kRetardMaxX10);
        g.clean_cycles[c] = 0u;
        g.global_clean_cycles = 0u;
        // Raise threshold (less sensitive) after knock event
        const uint16_t new_thr = (g.dac_threshold >= kThresholdStepUp)
            ? static_cast<uint16_t>(g.dac_threshold - kThresholdStepUp)
            : 0u;
        set_dac_threshold(new_thr);
    } else {
        if (g.clean_cycles[c] < 255u) {
            ++g.clean_cycles[c];
        }
        if ((g.clean_cycles[c] >= kRecoveryDelayCycles) && (knock_retard_x10[c] >= kRecoveryStepX10)) {
            knock_retard_x10[c] = static_cast<uint16_t>(knock_retard_x10[c] - kRecoveryStepX10);
        }

        if (g.global_clean_cycles < 255u) {
            ++g.global_clean_cycles;
        }
        if (g.global_clean_cycles >= 100u) {
            // Lower threshold (more sensitive) after many clean cycles
            const uint16_t new_thr = (g.dac_threshold <= kThresholdMax - kThresholdStepDown)
                ? static_cast<uint16_t>(g.dac_threshold + kThresholdStepDown)
                : kThresholdMax;
            set_dac_threshold(new_thr);
            g.global_clean_cycles = 0u;
        }
    }

    g.knock_count[c] = 0u;
}

uint16_t knock_get_retard_x10(uint8_t cyl) noexcept {
    return knock_retard_x10[static_cast<uint8_t>(cyl & 0x3u)];
}

uint16_t knock_get_threshold() noexcept {
    return g.dac_threshold;
}

#if defined(EMS_HOST_TEST)
uint8_t knock_test_get_knock_count(uint8_t cyl) noexcept {
    return g.knock_count[static_cast<uint8_t>(cyl & 0x3u)];
}

bool knock_test_window_active() noexcept {
    return g.window_active;
}

uint8_t knock_test_window_cyl() noexcept {
    return g.window_cyl;
}
#endif

}  // namespace ems::engine
