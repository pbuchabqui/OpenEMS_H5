#include "hal/usb_cdc.h"
#include "hal/usb_descriptors.h"

#include "app/tuner_studio.h"

#if !defined(EMS_HOST_TEST)

#include "hal/regs.h"

namespace ems::hal {
namespace {

constexpr uint16_t kRxSize = 256u;
constexpr uint16_t kTxSize = 512u;
constexpr uint16_t kRxMask = kRxSize - 1u;
constexpr uint16_t kTxMask = kTxSize - 1u;

static uint8_t g_rx_buf[kRxSize] = {};
static uint8_t g_tx_buf[kTxSize] = {};
static volatile uint16_t g_rx_head = 0u;
static volatile uint16_t g_rx_tail = 0u;
static volatile uint16_t g_tx_head = 0u;
static volatile uint16_t g_tx_tail = 0u;
static volatile bool g_usb_ready = false;
static volatile bool g_tx_active = false;

// EP0 control state
static volatile uint8_t g_ep0_setup[8] = {};
static volatile bool g_ep0_setup_ready = false;
static volatile const uint8_t* g_ep0_tx_data = nullptr;
static volatile uint16_t g_ep0_tx_len = 0;
static volatile uint16_t g_ep0_tx_sent = 0;
static volatile bool g_ep0_tx_active = false;
static usb::CdcLineCoding g_line_coding = usb::kDefaultLineCoding;

inline void usb_enable_clocks() noexcept {
    RCC_CR |= RCC_CR_HSI48ON;
    while ((RCC_CR & RCC_CR_HSI48RDY) == 0u) {}
    RCC_AHB1ENR |= RCC_AHB1ENR_USBOTGFSEN;
}

inline void usb_configure_pins() noexcept {
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 11u, GPIO_AF10);
    gpio_set_af(&GPIOA_MODER, &GPIOA_AFRL, &GPIOA_AFRH, &GPIOA_OSPEEDR, 12u, GPIO_AF10);
}

inline void usb_core_reset() noexcept {
    while ((USB_OTG_GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0u) {}
    USB_OTG_GRSTCTL = USB_OTG_GRSTCTL_CSRST;
    while ((USB_OTG_GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0u) {}
}

inline bool tx_pop(uint8_t& out) noexcept {
    if (g_tx_head == g_tx_tail) {
        return false;
    }
    out = g_tx_buf[g_tx_tail];
    g_tx_tail = static_cast<uint16_t>((g_tx_tail + 1u) & kTxMask);
    return true;
}

inline void rx_push(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_rx_head + 1u) & kRxMask);
    if (next == g_rx_tail) {
        return;
    }
    g_rx_buf[g_rx_head] = byte;
    g_rx_head = next;
}

inline void usb_ep0_send(const uint8_t* data, uint16_t len, uint16_t wLength) noexcept {
    if (len > wLength) {
        len = wLength;
    }
    g_ep0_tx_data = data;
    g_ep0_tx_len = len;
    g_ep0_tx_sent = 0;
    g_ep0_tx_active = true;

    // Start sending first chunk
    if (len > 0) {
        const uint16_t chunk = (len > 64) ? 64 : len;
        USB_OTG_DIEPTSIZ0 = chunk;
        for (uint16_t i = 0; i < chunk; i += 4) {
            uint32_t word = 0;
            if (i < len) word |= static_cast<uint32_t>(data[i]);
            if (i + 1 < len) word |= static_cast<uint32_t>(data[i + 1]) << 8;
            if (i + 2 < len) word |= static_cast<uint32_t>(data[i + 2]) << 16;
            if (i + 3 < len) word |= static_cast<uint32_t>(data[i + 3]) << 24;
            USB_OTG_FIFO0 = word;
        }
        USB_OTG_DIEPCTL0 |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
        g_ep0_tx_sent = chunk;
    } else {
        // Zero-length packet
        USB_OTG_DIEPTSIZ0 = 0;
        USB_OTG_DIEPCTL0 |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    }
}

inline void usb_handle_setup() noexcept {
    const uint8_t bmRequestType = g_ep0_setup[0];
    const uint8_t bRequest = g_ep0_setup[1];
    const uint16_t wValue = static_cast<uint16_t>(g_ep0_setup[2] | (g_ep0_setup[3] << 8));
    const uint16_t wIndex = static_cast<uint16_t>(g_ep0_setup[4] | (g_ep0_setup[5] << 8));
    const uint16_t wLength = static_cast<uint16_t>(g_ep0_setup[6] | (g_ep0_setup[7] << 8));

    const uint8_t type = bmRequestType & 0x60;
    const uint8_t recipient = bmRequestType & 0x1F;

    if (type == 0x00) {
        // Standard request
        switch (bRequest) {
            case 0x06: {  // GET_DESCRIPTOR
                const uint8_t desc_type = static_cast<uint8_t>(wValue >> 8);
                const uint8_t desc_index = static_cast<uint8_t>(wValue & 0xFF);
                const uint8_t* data = nullptr;
                uint16_t len = 0;

                if (desc_type == 0x01 && desc_index == 0) {
                    // Device descriptor
                    data = usb::kDeviceDescriptor;
                    len = sizeof(usb::kDeviceDescriptor);
                } else if (desc_type == 0x02 && desc_index == 0) {
                    // Configuration descriptor
                    data = usb::kConfigDescriptor;
                    len = sizeof(usb::kConfigDescriptor);
                } else if (desc_type == 0x03) {
                    // String descriptor
                    if (desc_index == 0) {
                        data = usb::kStringDescriptor0;
                        len = sizeof(usb::kStringDescriptor0);
                    } else if (desc_index == 1) {
                        data = usb::kStringDescriptor1;
                        len = sizeof(usb::kStringDescriptor1);
                    } else if (desc_index == 2) {
                        data = usb::kStringDescriptor2;
                        len = sizeof(usb::kStringDescriptor2);
                    } else if (desc_index == 3) {
                        data = usb::kStringDescriptor3;
                        len = sizeof(usb::kStringDescriptor3);
                    }
                }

                if (data != nullptr) {
                    usb_ep0_send(data, len, wLength);
                } else {
                    // Stall on unknown descriptor
                    USB_OTG_DOEPCTL0 |= USB_OTG_DOEPCTL_STALL;
                }
                break;
            }
            case 0x05: {  // SET_ADDRESS
                // Address will be set after status stage
                USB_OTG_DCFG = (USB_OTG_DCFG & ~0x7F0) | ((wValue & 0x7F) << 4);
                usb_ep0_send(nullptr, 0, wLength);
                break;
            }
            case 0x09: {  // SET_CONFIGURATION
                // Enable data endpoints
                USB_OTG_DIEPCTL1 |= USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_CNAK;
                USB_OTG_DOEPCTL1 |= USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
                USB_OTG_DOEPTSIZ1 = 64;
                usb_ep0_send(nullptr, 0, wLength);
                break;
            }
            case 0x00: {  // GET_STATUS
                static const uint8_t status[] = {0x00, 0x00};
                usb_ep0_send(status, sizeof(status), wLength);
                break;
            }
            default:
                // Stall on unsupported request
                USB_OTG_DOEPCTL0 |= USB_OTG_DOEPCTL_STALL;
                break;
        }
    } else if (type == 0x20 && recipient == 0x01) {
        // CDC class request to interface
        switch (bRequest) {
            case 0x20: {  // SET_LINE_CODING
                // Expect 7 bytes of line coding data in data stage
                // For now, just accept it
                usb_ep0_send(nullptr, 0, wLength);
                break;
            }
            case 0x21: {  // GET_LINE_CODING
                usb_ep0_send(reinterpret_cast<const uint8_t*>(&g_line_coding),
                             sizeof(g_line_coding), wLength);
                break;
            }
            case 0x22: {  // SET_CONTROL_LINE_STATE
                // DTR/RTS signals - ignore for now
                usb_ep0_send(nullptr, 0, wLength);
                break;
            }
            default:
                USB_OTG_DOEPCTL0 |= USB_OTG_DOEPCTL_STALL;
                break;
        }
    } else {
        // Stall on unknown request type
        USB_OTG_DOEPCTL0 |= USB_OTG_DOEPCTL_STALL;
    }
}

inline void usb_kick_tx() noexcept {
    if (!g_usb_ready || g_tx_active) {
        return;
    }

    uint8_t byte = 0u;
    if (!tx_pop(byte)) {
        return;
    }

    USB_OTG_DIEPTSIZ1 = (1u << 19) | 1u;
    USB_OTG_FIFO1 = byte;
    USB_OTG_DIEPCTL1 |= USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;
    g_tx_active = true;
}

}  // namespace

void usb_cdc_init() noexcept {
    g_rx_head = 0u;
    g_rx_tail = 0u;
    g_tx_head = 0u;
    g_tx_tail = 0u;
    g_usb_ready = false;
    g_tx_active = false;

    usb_enable_clocks();
    usb_configure_pins();
    usb_core_reset();

    USB_OTG_GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    USB_OTG_DCFG = USB_OTG_DCFG_DSPD_FS;
    USB_OTG_GRXFSIZ = 128u;
    USB_OTG_DIEPTXF0 = (64u << 16) | 128u;
    USB_OTG_DIEPTXF1 = (64u << 16) | 192u;

    USB_OTG_DIEPMSK = USB_OTG_DIEPINT_XFRC;
    USB_OTG_DOEPMSK = USB_OTG_DOEPINT_XFRC | USB_OTG_DOEPINT_STUP;
    USB_OTG_DAINTMSK = 0x00010001u;
    USB_OTG_GINTMSK = USB_OTG_GINTSTS_USBRST |
                      USB_OTG_GINTSTS_ENUMDNE |
                      USB_OTG_GINTSTS_RXFLVL |
                      USB_OTG_GINTSTS_IEPINT |
                      USB_OTG_GINTSTS_OEPINT |
                      USB_OTG_GINTSTS_USBSUSP;
    USB_OTG_GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    USB_OTG_DIEPCTL0 = USB_OTG_DIEPCTL_USBAEP;
    USB_OTG_DOEPCTL0 = USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    USB_OTG_DOEPTSIZ0 = (1u << 29) | (1u << 19) | 64u;

    USB_OTG_DIEPCTL1 = USB_OTG_DIEPCTL_USBAEP;
    USB_OTG_DOEPCTL1 = USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
    USB_OTG_DOEPTSIZ1 = 64u;

    USB_OTG_DCTL &= ~USB_OTG_DCTL_SDIS;
    nvic_set_priority(IRQ_OTG_FS, 6u);
    nvic_enable_irq(IRQ_OTG_FS);
    g_usb_ready = true;
}

void usb_cdc_poll_rx(uint16_t max_bytes) noexcept {
    uint16_t count = 0u;
    while ((g_rx_tail != g_rx_head) && (count < max_bytes)) {
        const uint8_t byte = g_rx_buf[g_rx_tail];
        g_rx_tail = static_cast<uint16_t>((g_rx_tail + 1u) & kRxMask);
        ems::app::ts_rx_byte(byte);
        ++count;
    }
    usb_kick_tx();
}

bool usb_cdc_tx_ready() noexcept {
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxMask);
    return next != g_tx_tail;
}

bool usb_cdc_tx_byte(uint8_t byte) noexcept {
    const uint16_t next = static_cast<uint16_t>((g_tx_head + 1u) & kTxMask);
    if (next == g_tx_tail) {
        return false;
    }
    g_tx_buf[g_tx_head] = byte;
    g_tx_head = next;
    usb_kick_tx();
    return true;
}

extern "C" void OTG_FS_IRQHandler() {
    const uint32_t gint = USB_OTG_GINTSTS & USB_OTG_GINTMSK;

    if ((gint & USB_OTG_GINTSTS_USBRST) != 0u) {
        USB_OTG_GINTSTS = USB_OTG_GINTSTS_USBRST;
        USB_OTG_DOEPTSIZ0 = (1u << 29) | (1u << 19) | 64u;
        USB_OTG_DOEPCTL0 |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
        g_usb_ready = true;
    }

    if ((gint & USB_OTG_GINTSTS_ENUMDNE) != 0u) {
        USB_OTG_GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        g_usb_ready = true;
    }

    if ((gint & USB_OTG_GINTSTS_IEPINT) != 0u) {
        USB_OTG_DIEPINT1 = USB_OTG_DIEPINT_XFRC;
        g_tx_active = false;
        usb_kick_tx();
        USB_OTG_GINTSTS = USB_OTG_GINTSTS_IEPINT;
    }

    if ((gint & USB_OTG_GINTSTS_OEPINT) != 0u) {
        USB_OTG_DOEPINT0 = USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_XFRC;
        USB_OTG_DOEPINT1 = USB_OTG_DOEPINT_XFRC;
        USB_OTG_GINTSTS = USB_OTG_GINTSTS_OEPINT;
    }

    if ((gint & USB_OTG_GINTSTS_RXFLVL) != 0u) {
        const uint32_t status = USB_OTG_GRXSTSP;
        const uint8_t ep = static_cast<uint8_t>(status & 0x0Fu);
        const uint16_t count = static_cast<uint16_t>((status >> 4) & 0x07FFu);
        if ((ep == 1u) && (count > 0u)) {
            for (uint16_t i = 0u; i < count; i += 4u) {
                const uint32_t word = USB_OTG_FIFO0;
                rx_push(static_cast<uint8_t>(word & 0xFFu));
                if (i + 1u < count) rx_push(static_cast<uint8_t>((word >> 8) & 0xFFu));
                if (i + 2u < count) rx_push(static_cast<uint8_t>((word >> 16) & 0xFFu));
                if (i + 3u < count) rx_push(static_cast<uint8_t>((word >> 24) & 0xFFu));
            }
            USB_OTG_DOEPTSIZ1 = 64u;
            USB_OTG_DOEPCTL1 |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
        }
    }

    if ((gint & USB_OTG_GINTSTS_USBSUSP) != 0u) {
        USB_OTG_GINTSTS = USB_OTG_GINTSTS_USBSUSP;
    }
}

}  // namespace ems::hal

#else

namespace ems::hal {

static uint8_t g_tx_buf[256] = {};
static uint16_t g_tx_len = 0u;
static uint8_t g_rx_buf[256] = {};
static uint16_t g_rx_head = 0u;
static uint16_t g_rx_tail = 0u;

void usb_cdc_init() noexcept {
    g_tx_len = 0u;
    g_rx_head = 0u;
    g_rx_tail = 0u;
}

void usb_cdc_poll_rx(uint16_t max_bytes) noexcept {
    uint16_t count = 0u;
    while ((g_rx_tail != g_rx_head) && (count < max_bytes)) {
        ems::app::ts_rx_byte(g_rx_buf[g_rx_tail++]);
        ++count;
    }
}

bool usb_cdc_tx_ready() noexcept {
    return g_tx_len < sizeof(g_tx_buf);
}

bool usb_cdc_tx_byte(uint8_t byte) noexcept {
    if (g_tx_len < sizeof(g_tx_buf)) {
        g_tx_buf[g_tx_len++] = byte;
        return true;
    }
    return false;
}

void usb_cdc_test_feed_rx(const uint8_t* data, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len && g_rx_head < sizeof(g_rx_buf); ++i) {
        g_rx_buf[g_rx_head++] = data[i];
    }
}

uint16_t usb_cdc_test_drain_tx(uint8_t* out, uint16_t max_len) noexcept {
    const uint16_t n = (g_tx_len < max_len) ? g_tx_len : max_len;
    for (uint16_t i = 0u; i < n; ++i) {
        out[i] = g_tx_buf[i];
    }
    g_tx_len = 0u;
    return n;
}

}  // namespace ems::hal

#endif
