#include "hal/fdcan.h"

#include <cstring>

#ifndef EMS_HOST_TEST
#include "hal/regs.h"

// CAN-FD Message RAM layout (STM32H562, FDCAN_SRAM = 0x4000AC00, 2.5 KB)
// Standard filter section: 1 entry × 1 word = 4 bytes (offset 0x000)
// RX FIFO 0: 3 entries × 18 words × 4 bytes = 216 bytes (offset 0x008)
//   kElemSizeWords = 18: 2 header words + 16 data words (64-byte CAN-FD payload)
// TX Buffer: 2 entries × 18 words × 4 bytes = 144 bytes (offset 0x0E0)
// Total SRAM used: 0x170 = 368 bytes (well within 2560 byte SRAM)
static constexpr uint32_t kSramStdFilters = 0x000u;
static constexpr uint32_t kSramRxFifo0    = 0x008u;
static constexpr uint32_t kElemSizeWords  = 18u;    // CAN-FD: 2 header + 16 data words
static constexpr uint32_t kRxFifo0Elems   = 3u;
static constexpr uint32_t kSramTxBuf      = kSramRxFifo0 + kRxFifo0Elems * kElemSizeWords * 4u;  // = 0x0E0

// Local register macros not defined in regs.h
#define FDCAN1_DBTP  STM32_REG32(FDCAN1_BASE + FDCAN_DBTP_OFF)         // Data Bit Timing (0x00C)
#define FDCAN1_RXESC STM32_REG32(FDCAN1_BASE + 0x0B8UL)                // RX Buffer/FIFO Element Size
#define FDCAN1_TXESC STM32_REG32(FDCAN1_BASE + 0x0C0UL)                // TX Buffer Element Size

// CAN-FD DLC encoding: code 0-8 = 0-8 bytes; 9-15 = 12,16,20,24,32,48,64 bytes
static constexpr uint8_t kDlcToBytes[16] = {
    0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
};

static uint8_t bytes_to_dlc(uint8_t nbytes, uint8_t& actual_len) noexcept {
    if (nbytes <= 8u)  { actual_len = nbytes; return nbytes; }
    if (nbytes <= 12u) { actual_len = 12u;    return 9u; }
    if (nbytes <= 16u) { actual_len = 16u;    return 10u; }
    if (nbytes <= 20u) { actual_len = 20u;    return 11u; }
    if (nbytes <= 24u) { actual_len = 24u;    return 12u; }
    if (nbytes <= 32u) { actual_len = 32u;    return 13u; }
    if (nbytes <= 48u) { actual_len = 48u;    return 14u; }
    actual_len = 64u; return 15u;
}

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
    // Enable CCE, FD Operation (FDOE), and Bit Rate Switch (BRSE)
    FDCAN1_CCCR |= FDCAN_CCCR_CCE | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;

    // Nominal bit timing: FDCAN clock = PLL1Q = 62.5 MHz
    // NBRP+1=5 → Tq=80ns; (NTSEG1+NTSEG2+3)=25 → 25×80ns=2000ns → 500 kbps
    FDCAN1_NBTP = ((4u  & 0x7Fu)  << 25)   // NSJW = 4
               | ((4u  & 0x1FFu) << 16)    // NBRP = 4 → prescaler = 5
               | ((17u & 0xFFu)  << 8)     // NTSEG1 = 17 → 18 TQ
               | ((5u  & 0x7Fu)  << 0);    // NTSEG2 = 5  → 6 TQ

    // Data bit timing: DBRP+1=1 → Tq=16ns; (DTSEG1+DTSEG2+3)=25 → 25×16ns=400ns → 2.5 Mbps
    FDCAN1_DBTP = ((0u  & 0x1Fu)  << 16)   // DBRP = 0 → prescaler = 1
               | ((18u & 0x1Fu)  << 8)     // DTSEG1 = 18 → 19 TQ (80% sample point)
               | ((4u  & 0xFu)   << 4)     // DTSEG2 = 4  → 5 TQ
               | ((3u  & 0xFu)   << 0);    // DSJW = 3

    // Clear SRAM (256 words = 1 KB, covers our 368-byte layout)
    for (uint32_t i = 0u; i < 256u; ++i) {
        sram_word(i * 4u) = 0u;
    }

    // Standard filter: accept 0x180 (WBO2 sensor) → store in RX FIFO0
    sram_word(kSramStdFilters + 0u) =
        (0x2u   << 30)    // SFEC = 010: store in Rx FIFO 0
      | (0x1u   << 27)    // SFT  = 01: dual ID filter
      | (0x180u << 16)    // SFID1 = 0x180
      | (0x180u << 0);    // SFID2 = 0x180

    // Global filter: reject non-matching frames
    FDCAN1_RXGFC = (0u << 8)    // LSS = 0 (standard filter count)
                 | (1u << 0)    // RRFE: reject remote frames with extended ID
                 | (2u << 4)    // ANFS: non-matching std → reject
                 | (2u << 2);   // ANFE: non-matching ext → reject

    // RX FIFO0: 3 elements × 18 words, F0DS set to 64 bytes via RXESC
    FDCAN1_RXESC = 7u;                          // F0DS = 111b → 64-byte elements for FIFO0
    FDCAN1_RXF0C = (kSramRxFifo0 / 4u)         // F0SA (word address)
                 | (kRxFifo0Elems << 16);       // F0S = 3 elements

    // TX Buffer: 2 dedicated buffers × 18 words, TBDS set to 64 bytes via TXESC
    FDCAN1_TXESC = 7u;                          // TBDS = 111b → 64-byte elements
    FDCAN1_TXBC  = (kSramTxBuf / 4u)            // TBSA (word address)
               | (2u << 16);                    // NDTB = 2 dedicated TX buffers

    FDCAN1_CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1_CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN1_CCCR & FDCAN_CCCR_INIT) { }
}

bool fdcan_tx(const FdcanFrame& frame) noexcept {
    if ((FDCAN1_TXFQS & (1u << 21)) != 0u) {
        return false;  // TX FIFO full
    }

    const uint32_t tx_addr = kSramTxBuf;
    const uint32_t id_shifted = (frame.id & 0x7FFu) << 18;

    uint8_t actual_len = 0u;
    const uint8_t dlc_code = bytes_to_dlc(frame.dlc_bytes, actual_len);

    // T0: standard ID (bits [28:18])
    sram_word(tx_addr + 0u) = id_shifted;
    // T1: DLC[19:16], FDF[21]=1 (CAN-FD format), BRS[20]
    sram_word(tx_addr + 4u) = (static_cast<uint32_t>(dlc_code) << 16)
                             | (1u << 21)                              // FDF = 1
                             | (frame.brs ? (1u << 20) : 0u);         // BRS

    // Data words (up to 16 words = 64 bytes)
    const uint32_t nwords = (static_cast<uint32_t>(actual_len) + 3u) / 4u;
    for (uint32_t w = 0u; w < nwords; ++w) {
        const uint32_t base = w * 4u;
        uint32_t val = 0u;
        if (base + 0u < actual_len) { val |= static_cast<uint32_t>(frame.data[base + 0u]) << 0; }
        if (base + 1u < actual_len) { val |= static_cast<uint32_t>(frame.data[base + 1u]) << 8; }
        if (base + 2u < actual_len) { val |= static_cast<uint32_t>(frame.data[base + 2u]) << 16; }
        if (base + 3u < actual_len) { val |= static_cast<uint32_t>(frame.data[base + 3u]) << 24; }
        sram_word(tx_addr + 8u + w * 4u) = val;
    }

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

    out.id       = (w0 >> 18) & 0x7FFu;
    out.extended = static_cast<bool>((w0 >> 30) & 1u);
    out.brs      = static_cast<bool>((w1 >> 20) & 1u);
    std::memset(out.data, 0, sizeof(out.data));

    // Decode CAN-FD DLC (codes 9-15 map to 12,16,20,24,32,48,64 bytes)
    const uint8_t dlc_code = static_cast<uint8_t>((w1 >> 16) & 0xFu);
    const uint8_t nbytes   = kDlcToBytes[dlc_code];
    out.dlc_bytes = nbytes;

    // Read data words (up to 16 words = 64 bytes)
    const uint32_t nwords = (static_cast<uint32_t>(nbytes) + 3u) / 4u;
    for (uint32_t w = 0u; w < nwords; ++w) {
        const uint32_t dword = sram_word(elem_addr + 8u + w * 4u);
        const uint32_t base  = w * 4u;
        if (base + 0u < nbytes) { out.data[base + 0u] = static_cast<uint8_t>(dword >> 0); }
        if (base + 1u < nbytes) { out.data[base + 1u] = static_cast<uint8_t>(dword >> 8); }
        if (base + 2u < nbytes) { out.data[base + 2u] = static_cast<uint8_t>(dword >> 16); }
        if (base + 3u < nbytes) { out.data[base + 3u] = static_cast<uint8_t>(dword >> 24); }
    }

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
