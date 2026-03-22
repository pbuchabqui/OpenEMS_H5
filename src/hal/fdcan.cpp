#include "hal/fdcan.h"

#include <cstring>

#ifndef EMS_HOST_TEST
#include "hal/regs.h"

static constexpr uint32_t kSramStdFilters = 0x000u;
static constexpr uint32_t kSramRxFifo0    = 0x008u;
static constexpr uint32_t kSramTxBuf      = 0x03Eu;
static constexpr uint32_t kElemSizeWords  = 4u;

static inline volatile uint32_t& sram_word(uint32_t offset_bytes) noexcept {
    return *reinterpret_cast<volatile uint32_t*>(FDCAN_SRAM + offset_bytes);
}
#endif

namespace ems::hal {

#if !defined(EMS_HOST_TEST)

void fdcan_init() noexcept {
    RCC_APB1LENR |= RCC_APB1LENR_FDCAN1EN;

    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 8u, GPIO_AF9);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 9u, GPIO_AF9);

    FDCAN1_CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN1_CCCR & FDCAN_CCCR_INIT) == 0u) { }
    FDCAN1_CCCR |= FDCAN_CCCR_CCE;

    FDCAN1_NBTP = ((4u  & 0x7Fu) << 25)
               | ((4u  & 0x1FFu) << 16)
               | ((17u & 0xFFu)  << 8)
               | ((5u  & 0x7Fu)  << 0);

    for (uint32_t i = 0u; i < 128u; ++i) {
        sram_word(i * 4u) = 0u;
    }

    sram_word(kSramStdFilters + 0u) =
        (0x2u << 30)
      | (0x1u << 27)
      | (0x180u << 16)
      | (0x180u << 0);

    FDCAN1_RXGFC = (0u << 8)
                 | (1u << 0)
                 | (2u << 4)
                 | (2u << 2);

    FDCAN1_RXF0C = (kSramRxFifo0 / 4u)
                 | (3u << 16);

    FDCAN1_TXBC = (kSramTxBuf / 4u)
               | (2u << 16);

    FDCAN1_CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1_CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN1_CCCR & FDCAN_CCCR_INIT) { }
}

bool fdcan_tx(const FdcanFrame& frame) noexcept {
    if ((FDCAN1_TXFQS & (1u << 21)) != 0u) {
        return false;
    }

    const uint32_t tx_addr = kSramTxBuf;
    const uint32_t id_shifted = (frame.id & 0x7FFu) << 18;
    const uint8_t dlc = (frame.dlc_bytes > 8u) ? 8u : frame.dlc_bytes;

    sram_word(tx_addr + 0u) = id_shifted;
    sram_word(tx_addr + 4u) = (static_cast<uint32_t>(dlc) << 16);

    uint32_t d0 = 0u;
    uint32_t d1 = 0u;
    if (dlc >= 1u) { d0 |= static_cast<uint32_t>(frame.data[0]) << 0; }
    if (dlc >= 2u) { d0 |= static_cast<uint32_t>(frame.data[1]) << 8; }
    if (dlc >= 3u) { d0 |= static_cast<uint32_t>(frame.data[2]) << 16; }
    if (dlc >= 4u) { d0 |= static_cast<uint32_t>(frame.data[3]) << 24; }
    if (dlc >= 5u) { d1 |= static_cast<uint32_t>(frame.data[4]) << 0; }
    if (dlc >= 6u) { d1 |= static_cast<uint32_t>(frame.data[5]) << 8; }
    if (dlc >= 7u) { d1 |= static_cast<uint32_t>(frame.data[6]) << 16; }
    if (dlc >= 8u) { d1 |= static_cast<uint32_t>(frame.data[7]) << 24; }
    sram_word(tx_addr + 8u)  = d0;
    sram_word(tx_addr + 12u) = d1;

    FDCAN1_TXBAR = (1u << 0);
    return true;
}

bool fdcan_rx_pop(FdcanFrame& out) noexcept {
    const uint32_t fqs = FDCAN1_RXF0S;
    if ((fqs & 0x7Fu) == 0u) {
        return false;
    }

    const uint32_t get_idx = (fqs >> 8) & 0x3Fu;
    const uint32_t elem_addr = kSramRxFifo0 + get_idx * (kElemSizeWords * 4u);

    const uint32_t w0 = sram_word(elem_addr + 0u);
    const uint32_t w1 = sram_word(elem_addr + 4u);
    const uint32_t d0 = sram_word(elem_addr + 8u);
    const uint32_t d1 = sram_word(elem_addr + 12u);

    out.id = (w0 >> 18) & 0x7FFu;
    out.extended = false;
    out.dlc_bytes = static_cast<uint8_t>((w1 >> 16) & 0xFu);
    out.brs = false;
    std::memset(out.data, 0, sizeof(out.data));

    const uint8_t dlc = (out.dlc_bytes > 8u) ? 8u : out.dlc_bytes;
    if (dlc >= 1u) { out.data[0] = static_cast<uint8_t>(d0 >> 0); }
    if (dlc >= 2u) { out.data[1] = static_cast<uint8_t>(d0 >> 8); }
    if (dlc >= 3u) { out.data[2] = static_cast<uint8_t>(d0 >> 16); }
    if (dlc >= 4u) { out.data[3] = static_cast<uint8_t>(d0 >> 24); }
    if (dlc >= 5u) { out.data[4] = static_cast<uint8_t>(d1 >> 0); }
    if (dlc >= 6u) { out.data[5] = static_cast<uint8_t>(d1 >> 8); }
    if (dlc >= 7u) { out.data[6] = static_cast<uint8_t>(d1 >> 16); }
    if (dlc >= 8u) { out.data[7] = static_cast<uint8_t>(d1 >> 24); }

    FDCAN1_RXF0A = get_idx;
    return true;
}

#else

static FdcanFrame g_tx_buf[8] = {};
static FdcanFrame g_rx_buf[8] = {};
static uint8_t g_tx_cnt = 0u;
static uint8_t g_rx_cnt = 0u;
static uint8_t g_rx_pop_idx = 0u;

void fdcan_init() noexcept {}

bool fdcan_tx(const FdcanFrame& frame) noexcept {
    if (g_tx_cnt < 8u) {
        g_tx_buf[g_tx_cnt++] = frame;
    }
    return true;
}

bool fdcan_rx_pop(FdcanFrame& out) noexcept {
    if (g_rx_pop_idx >= g_rx_cnt) {
        return false;
    }
    out = g_rx_buf[g_rx_pop_idx++];
    return true;
}

void fdcan_test_reset() noexcept {
    g_tx_cnt = 0u;
    g_rx_cnt = 0u;
    g_rx_pop_idx = 0u;
}

bool fdcan_test_inject_rx(const FdcanFrame& frame) noexcept {
    if (g_rx_cnt >= 8u) {
        return false;
    }
    g_rx_buf[g_rx_cnt++] = frame;
    return true;
}

bool fdcan_test_pop_tx(FdcanFrame& out) noexcept {
    if (g_tx_cnt == 0u) {
        return false;
    }
    out = g_tx_buf[--g_tx_cnt];
    return true;
}

#endif

}  // namespace ems::hal
