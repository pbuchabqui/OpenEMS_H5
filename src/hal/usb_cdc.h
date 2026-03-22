#pragma once

#include <cstdint>

namespace ems::hal {

void usb_cdc_init() noexcept;
void usb_cdc_poll_rx(uint16_t max_bytes = 64u) noexcept;
bool usb_cdc_tx_ready() noexcept;
bool usb_cdc_tx_byte(uint8_t byte) noexcept;

#if defined(EMS_HOST_TEST)
void usb_cdc_test_feed_rx(const uint8_t* data, uint16_t len) noexcept;
uint16_t usb_cdc_test_drain_tx(uint8_t* out, uint16_t max_len) noexcept;
#endif

}  // namespace ems::hal
