#pragma once

#include <cstdint>

namespace ems::hal {

struct FdcanFrame {
    uint32_t id;
    uint8_t dlc_bytes;
    uint8_t data[64];
    bool extended;
    bool brs;
};

void fdcan_init() noexcept;
bool fdcan_tx(const FdcanFrame& frame) noexcept;
bool fdcan_rx_pop(FdcanFrame& out) noexcept;

#if defined(EMS_HOST_TEST)
void fdcan_test_reset() noexcept;
bool fdcan_test_inject_rx(const FdcanFrame& frame) noexcept;
bool fdcan_test_pop_tx(FdcanFrame& out) noexcept;
#endif

}  // namespace ems::hal
