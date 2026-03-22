#pragma once

#include <cstdint>

namespace ems::hal::usb {

// ─── USB Device Descriptor ─────────────────────────────────────────────────
constexpr uint8_t kDeviceDescriptor[] = {
    18,                         // bLength
    0x01,                       // bDescriptorType (Device)
    0x00, 0x02,                 // bcdUSB (2.00)
    0x02,                       // bDeviceClass (CDC)
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    64,                         // bMaxPacketSize0
    0x83, 0x04,                 // idVendor (0x0483 = STMicroelectronics)
    0x40, 0x57,                 // idProduct (0x5740 = Virtual COM Port)
    0x00, 0x02,                 // bcdDevice (2.00)
    1,                          // iManufacturer
    2,                          // iProduct
    3,                          // iSerialNumber
    1                           // bNumConfigurations
};

// ─── USB Configuration Descriptor (CDC ACM) ─────────────────────────────────
// Configuration + IAD + CDC Interface + CDC Data Interface
constexpr uint8_t kConfigDescriptor[] = {
    // Configuration Descriptor (9 bytes)
    9,                          // bLength
    0x02,                       // bDescriptorType (Configuration)
    75, 0,                      // wTotalLength (75 bytes)
    2,                          // bNumInterfaces
    1,                          // bConfigurationValue
    0,                          // iConfiguration
    0x80,                       // bmAttributes (bus powered)
    250,                        // bMaxPower (500 mA)

    // Interface Association Descriptor (8 bytes)
    8,                          // bLength
    0x0B,                       // bDescriptorType (IAD)
    0,                          // bFirstInterface
    2,                          // bInterfaceCount
    0x02,                       // bFunctionClass (CDC)
    0x02,                       // bFunctionSubClass (ACM)
    0x01,                       // bFunctionProtocol (AT commands)
    0,                          // iFunction

    // CDC Interface Descriptor (9 bytes)
    9,                          // bLength
    0x04,                       // bDescriptorType (Interface)
    0,                          // bInterfaceNumber
    0,                          // bAlternateSetting
    1,                          // bNumEndpoints
    0x02,                       // bInterfaceClass (CDC)
    0x02,                       // bInterfaceSubClass (ACM)
    0x01,                       // bInterfaceProtocol (AT commands)
    0,                          // iInterface

    // Header Functional Descriptor (5 bytes)
    5,                          // bLength
    0x24,                       // bDescriptorType (CS_INTERFACE)
    0x00,                       // bDescriptorSubtype (Header)
    0x10, 0x01,                 // bcdCDC (1.10)

    // Call Management Functional Descriptor (5 bytes)
    5,                          // bLength
    0x24,                       // bDescriptorType (CS_INTERFACE)
    0x01,                       // bDescriptorSubtype (Call Management)
    0x00,                       // bmCapabilities
    1,                          // bDataInterface

    // ACM Functional Descriptor (4 bytes)
    4,                          // bLength
    0x24,                       // bDescriptorType (CS_INTERFACE)
    0x02,                       // bDescriptorSubtype (ACM)
    0x02,                       // bmCapabilities (line coding and serial state)

    // Union Functional Descriptor (5 bytes)
    5,                          // bLength
    0x24,                       // bDescriptorType (CS_INTERFACE)
    0x06,                       // bDescriptorSubtype (Union)
    0,                          // bMasterInterface
    1,                          // bSlaveInterface0

    // CDC Notification Endpoint Descriptor (7 bytes)
    7,                          // bLength
    0x05,                       // bDescriptorType (Endpoint)
    0x81,                       // bEndpointAddress (EP1 IN)
    0x03,                       // bmAttributes (Interrupt)
    8, 0,                       // wMaxPacketSize (8)
    16,                         // bInterval (16 ms)

    // CDC Data Interface Descriptor (9 bytes)
    9,                          // bLength
    0x04,                       // bDescriptorType (Interface)
    1,                          // bInterfaceNumber
    0,                          // bAlternateSetting
    2,                          // bNumEndpoints
    0x0A,                       // bInterfaceClass (CDC Data)
    0x00,                       // bInterfaceSubClass
    0x00,                       // bInterfaceProtocol
    0,                          // iInterface

    // Data OUT Endpoint Descriptor (7 bytes)
    7,                          // bLength
    0x05,                       // bDescriptorType (Endpoint)
    0x02,                       // bEndpointAddress (EP2 OUT)
    0x02,                       // bmAttributes (Bulk)
    64, 0,                      // wMaxPacketSize (64)
    0,                          // bInterval

    // Data IN Endpoint Descriptor (7 bytes)
    7,                          // bLength
    0x05,                       // bDescriptorType (Endpoint)
    0x82,                       // bEndpointAddress (EP2 IN)
    0x02,                       // bmAttributes (Bulk)
    64, 0,                      // wMaxPacketSize (64)
    0                           // bInterval
};

// ─── String Descriptors ─────────────────────────────────────────────────────
// Language ID (English US)
constexpr uint8_t kStringDescriptor0[] = {
    4,                          // bLength
    0x03,                       // bDescriptorType (String)
    0x09, 0x04                  // wLANGID (English US)
};

// Manufacturer: "OpenEMS"
constexpr uint8_t kStringDescriptor1[] = {
    14,                         // bLength
    0x03,                       // bDescriptorType (String)
    'O',0, 'p',0, 'e',0, 'n',0, 'E',0, 'M',0, 'S',0
};

// Product: "OpenEMS v2.2"
constexpr uint8_t kStringDescriptor2[] = {
    28,                         // bLength
    0x03,                       // bDescriptorType (String)
    'O',0, 'p',0, 'e',0, 'n',0, 'E',0, 'M',0, 'S',0, ' ',0, 'v',0, '2',0, '.',0, '2',0
};

// Serial Number: "000001"
constexpr uint8_t kStringDescriptor3[] = {
    14,                         // bLength
    0x03,                       // bDescriptorType (String)
    '0',0, '0',0, '0',0, '0',0, '0',0, '1',0
};

// ─── CDC Line Coding Structure ──────────────────────────────────────────────
struct CdcLineCoding {
    uint32_t dwDTERate;         // Baud rate
    uint8_t  bCharFormat;       // 0=1 stop bit, 1=1.5 stop bits, 2=2 stop bits
    uint8_t  bParityType;       // 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space
    uint8_t  bDataBits;         // 5, 6, 7, 8, or 16
} __attribute__((packed));

// Default line coding: 115200 8N1
constexpr CdcLineCoding kDefaultLineCoding = {
    115200,   // dwDTERate
    0,        // bCharFormat (1 stop bit)
    0,        // bParityType (none)
    8         // bDataBits
};

}  // namespace ems::hal::usb