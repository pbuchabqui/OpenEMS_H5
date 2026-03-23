#include "engine/ecu_sched.h"

namespace {

// IVC angle in degrees ABDC (After Bottom Dead Center)
// Default: 50° ABDC (typical for naturally aspirated engines)
static uint8_t g_ivc_abdc_deg = 50u;

}  // namespace

namespace ems::engine {

void ecu_sched_set_ivc(uint8_t ivc_abdc_deg) noexcept {
    // Clamp to valid range 0-180°
    if (ivc_abdc_deg > 180u) {
        ivc_abdc_deg = 180u;
    }
    g_ivc_abdc_deg = ivc_abdc_deg;
}

uint8_t ecu_sched_get_ivc() noexcept {
    return g_ivc_abdc_deg;
}

}  // namespace ems::engine