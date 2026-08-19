/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 HiFiPhile
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "config.h"
#include "usb.h"

#ifndef ENABLE_SERIAL
#define ENABLE_SERIAL 0
#endif

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING_OUT,
    ITF_NUM_AUDIO_STREAMING_IN,
    ITF_NUM_HID,
#if ENABLE_SERIAL
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
#endif
#ifdef ENABLE_WAKE_HID
    ITF_NUM_HID_KBD,
#endif
    ITF_NUM_TOTAL,

    CONFIG_DESC_LEN_AUDIO_IAD =
#if ENABLE_SERIAL
        8,
#else
        0,
#endif
    CONFIG_DESC_LEN_BASE = 0x00E1 + CONFIG_DESC_LEN_AUDIO_IAD, // (DS4: 2-ch feature unit is 2 bytes shorter)
    // Keyboard interface adds 25 bytes:
    //   9 (interface) + 9 (HID class) + 7 (EP IN) = 25
    CONFIG_DESC_LEN_WAKE_KBD =
#ifdef ENABLE_WAKE_HID
        25,
#else
        0,
#endif
    // The audio function occupies everything between the configuration header
    // and the HID interface: the (serial-build only) IAD, the Audio Control
    // interface with its 71 bytes of class-specific descriptors, and the two
    // AudioStreaming interfaces at 52 bytes each. It can be dropped at runtime
    // -- see tud_descriptor_configuration_cb / usb_audio_exposed().
    CONFIG_DESC_HDR_LEN = 9,
    CONFIG_DESC_LEN_AUDIO = CONFIG_DESC_LEN_AUDIO_IAD + 9 + 71 + 52 + 52,
    CONFIG_DESC_LEN_HID = 32, // interface 9 + HID class 9 + 2 endpoints
    AUDIO_ITF_COUNT = 3,      // control + streaming OUT + streaming IN

    CONFIG_DESC_LEN_TOTAL = CONFIG_DESC_LEN_BASE + CONFIG_DESC_LEN_WAKE_KBD
#if ENABLE_SERIAL
        + TUD_CDC_DESC_LEN
#endif
};

// String Descriptor Index
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
#if ENABLE_SERIAL
    STRID_CDC,
#endif
};

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t desc_device =
{
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
#ifdef ENABLE_WAKE_HID
    .bcdUSB = 0x0210, // USB 2.1 -- required so the host requests BOS (carries our MS OS 2.0 descriptor)
#else
    .bcdUSB = 0x0200,
#endif

    // Use Interface Association Descriptor (IAD) for Audio
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
#if ENABLE_SERIAL
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
#endif
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = 0x054C,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    // .iSerialNumber = 0x00,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
    desc_device.idProduct = 0x09CC; // DualShock 4 v2 (CUH-ZCT2)
    desc_device.iSerialNumber = get_config().enable_usb_sn ? 0x03 : 0x00;
    // USB 2.1 (so the host requests the BOS / MS OS 2.0 selective-suspend opt-in)
    // only when wake is enabled AND the audio function is on the bus -- the
    // opt-in exists to stop the audio function from blocking selective suspend,
    // and with audio hidden there is nothing for the descriptor to point at.
    desc_device.bcdUSB = (get_config().enable_wake && usb_audio_exposed()) ? 0x0210 : 0x0200;
    return reinterpret_cast<uint8_t const *>(&desc_device);
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
uint8_t descriptor_configuration[] = {
    // --- CONFIGURATION DESCRIPTOR ---
    0x09, // bLength
    0x02, // bDescriptorType (CONFIGURATION)
    U16_TO_U8S_LE(CONFIG_DESC_LEN_TOTAL), // wTotalLength
    ITF_NUM_TOTAL, // bNumInterfaces
    0x01, // bConfigurationValue: 1
    0x00, // iConfiguration: 0
#ifdef ENABLE_WAKE_HID
    0xE0, // bmAttributes: SELF-POWERED + REMOTE-WAKEUP
#else
    0xC0, // bmAttributes: SELF-POWERED, NO REMOTE-WAKEUP
#endif
    0xFA, // bMaxPower: 500mA (250 * 2mA)

#if ENABLE_SERIAL
    // --- INTERFACE ASSOCIATION DESCRIPTOR: Audio function (interfaces 0-2) ---
    0x08, // bLength
    TUSB_DESC_INTERFACE_ASSOCIATION, // bDescriptorType
    ITF_NUM_AUDIO_CONTROL, // bFirstInterface
    0x03, // bInterfaceCount
    0x01, // bFunctionClass: Audio
    0x01, // bFunctionSubClass: Audio Control
    0x00, // bFunctionProtocol
    0x00, // iFunction

#endif
    // --- INTERFACE DESCRIPTOR (0.0): Audio Control ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x00, // bInterfaceNumber: 0
    0x00, // bAlternateSetting: 0
    0x00, // bNumEndpoints: 0
    0x01, // bInterfaceClass: Audio (0x01)
    0x01, // bInterfaceSubClass: Audio Control (0x01)
    0x00, // bInterfaceProtocol: 0x00
    0x00, // iInterface: 0

    // Class-specific AC Interface Header Descriptor
    0x0A, // bLength: 10
    0x24, // bDescriptorType: CS_INTERFACE (0x24)
    0x01, // bDescriptorSubtype: Header (0x01)
    0x00, 0x01, // bcdADC: 1.00
    0x47, 0x00, // wTotalLength: 71 (0x0047)
    0x02, // bInCollection: 2 streaming interfaces
    0x01, // baInterfaceNr(1): Interface 1
    0x02, // baInterfaceNr(2): Interface 2

    // Input Terminal Descriptor (Terminal ID 1: USB Streaming → Output to Speaker)
    0x0C, // bLength: 12
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Input Terminal
    0x01, // bTerminalID: 1
    0x01, 0x01, // wTerminalType: USB Streaming (0x0101)
    0x06, // bAssocTerminal: 6 (paired with USB OUT terminal)
    0x02, // bNrChannels: 2
    0x03, 0x00, // wChannelConfig: L/R Front (0x0003)
    0x00, // iChannelNames: 0
    0x00, // iTerminal: 0

    // Feature Unit Descriptor (Unit ID 2 ← from Terminal 1)
    0x0A, // bLength: 10
    0x24, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Feature Unit
    0x02, // bUnitID: 2
    0x01, // bSourceID: 1
    0x01, // bControlSize: 1 byte per control
    0x03, // bmaControls[0]: Master – Mute, Volume
    0x00, 0x00, 0x00, // bmaControls[1..2] + iFeature

    // Output Terminal Descriptor (Terminal ID 3: Speaker ← from Unit 2)
    0x09, // bLength: 9
    0x24, // bDescriptorType: CS_INTERFACE
    0x03, // bDescriptorSubtype: Output Terminal
    0x03, // bTerminalID: 3
    0x02, 0x04, // wTerminalType: Headset (0x0402), matches the real DS4 v2
    0x04, // bAssocTerminal: 4 (paired with mic input)
    0x02, // bSourceID: 2 (Feature Unit)
    0x00, // iTerminal: 0

    // Input Terminal Descriptor (Terminal ID 4: Headset Mic)
    0x0C, // bLength: 12
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Input Terminal
    0x04, // bTerminalID: 4
    0x02, 0x04, // wTerminalType: Headset (0x0402)
    0x03, // bAssocTerminal: 3 (paired with speaker)
    0x01, // bNrChannels: 1 (mono; matches the real DS4 v2 mic)
    0x00, 0x00, // wChannelConfig: none (master channel only)
    0x00, // iChannelNames: 0
    0x00, // iTerminal: 0

    // Feature Unit Descriptor (Unit ID 5 ← from Terminal 4)
    0x09, // bLength: 9
    0x24, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Feature Unit
    0x05, // bUnitID: 5
    0x04, // bSourceID: 4
    0x01, // bControlSize: 1
    0x03, // bmaControls[0]: Master – Mute, Volume
    0x00, // bmaControls[1]: Ch1 – no controls
    0x00, // iFeature: 0

    // Output Terminal Descriptor (Terminal ID 6: USB Streaming ← from Unit 5)
    0x09, // bLength: 9
    0x24, // bDescriptorType: CS_INTERFACE
    0x03, // bDescriptorSubtype: Output Terminal
    0x06, // bTerminalID: 6
    0x01, 0x01, // wTerminalType: USB Streaming (0x0101)
    0x01, // bAssocTerminal: 1
    0x05, // bSourceID: 5
    0x00, // iTerminal: 0

    // --- INTERFACE DESCRIPTOR (1.0): Audio Streaming (OUT - Alternate 0) ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x01, // bInterfaceNumber: 1
    0x00, // bAlternateSetting: 0
    0x00, // bNumEndpoints: 0
    0x01, // bInterfaceClass: Audio
    0x02, // bInterfaceSubClass: Audio Streaming
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // --- INTERFACE DESCRIPTOR (1.1): Audio Streaming (OUT - Alternate 1) ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x01, // bInterfaceNumber: 1
    0x01, // bAlternateSetting: 1
    0x01, // bNumEndpoints: 1
    0x01, // bInterfaceClass: Audio
    0x02, // bInterfaceSubClass: Audio Streaming
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // AS General Descriptor (for Interface 1.1)
    0x07, // bLength: 7
    0x24, // bDescriptorType: CS_INTERFACE
    0x01, // bDescriptorSubtype: AS_GENERAL
    0x01, // bTerminalLink: connected to Terminal ID 1
    0x01, // bDelay: 1 frame
    0x01, 0x00, // wFormatTag: PCM (0x0001)

    // Format Type Descriptor (2-channel, 16-bit, 32kHz - the DS4's native rate)
    0x0B, // bLength: 11
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: FORMAT_TYPE
    0x01, // bFormatType: TYPE_I
    0x02, // bNrChannels: 2
    0x02, // bSubframeSize: 2 bytes/sample
    0x10, // bBitResolution: 16 bits
    0x01, // bSamFreqType: 1 discrete frequency
    0x00, 0x7D, 0x00, // tSamFreq: 32000 Hz (0x007D00)

    // Endpoint Descriptor (Audio OUT: EP1)
    0x09, // bLength
    0x05, // bDescriptorType (ENDPOINT)
    0x01, // bEndpointAddress: OUT EP1
    0x09, // bmAttributes: Isochronous, Adaptive
    0x84, 0x00, // wMaxPacketSize: 132 bytes ((32+1) samples * 2 ch * 2 bytes)
    0x01, // bInterval: 1
    0x00, // bRefresh
    0x00, // bSynchAddress

    // Class-specific Audio Streaming Endpoint Descriptor (EP1)
    0x07, // bLength
    0x25, // bDescriptorType: CS_ENDPOINT
    0x01, // bDescriptorSubtype: GENERAL
    0x00, // Attributes: No pitch/sampling freq control
    0x00, // Lock Delay Units: Undefined
    0x00, 0x00, // Lock Delay: 0

    // --- INTERFACE DESCRIPTOR (2.0): Audio Streaming IN (Alternate 0) ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x02, // bInterfaceNumber: 2
    0x00, // bAlternateSetting: 0
    0x00, // bNumEndpoints: 0
    0x01, // bInterfaceClass: Audio
    0x02, // bInterfaceSubClass: Audio Streaming
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // --- INTERFACE DESCRIPTOR (2.1): Audio Streaming IN (Alternate 1) ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x02, // bInterfaceNumber: 2
    0x01, // bAlternateSetting: 1
    0x01, // bNumEndpoints: 1
    0x01, // bInterfaceClass: Audio
    0x02, // bInterfaceSubClass: Audio Streaming
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // AS General Descriptor (for Interface 2.1)
    0x07, // bLength: 7
    0x24, // bDescriptorType: CS_INTERFACE
    0x01, // bDescriptorSubtype: AS_GENERAL
    0x06, // bTerminalLink: connected to Terminal ID 6
    0x01, // bDelay: 1 frame
    0x01, 0x00, // wFormatTag: PCM (0x0001)

    // Format Type Descriptor (1-channel, 16-bit, 16kHz - the real DS4 v2's mic format)
    0x0B, // bLength: 11
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: FORMAT_TYPE
    0x01, // bFormatType: TYPE_I
    0x01, // bNrChannels: 1
    0x02, // bSubframeSize: 2
    0x10, // bBitResolution: 16
    0x01, // bSamFreqType: 1
    0x80, 0x3E, 0x00, // tSamFreq: 16000 Hz

    // Endpoint Descriptor (Audio IN: EP2)
    0x09, // bLength
    0x05, // bDescriptorType (ENDPOINT)
    0x82, // bEndpointAddress: IN EP2
    0x05, // bmAttributes: Isochronous, Asynchronous
    // 1 ms packets are fine here: the EP is reloaded from the class driver's
    // completion ISR (tud_audio_tx_done_isr in audio.cpp), so a frame's data
    // is always staged before the next arm. Feeding from the main loop
    // instead misses alternate frames and halves the delivered rate.
    0x22, 0x00, // wMaxPacketSize: 34 bytes ((16+1) samples * 1 ch * 2 bytes)
    0x01, // bInterval: 1
    0x00, // bRefresh
    0x00, // bSynchAddress

    // Class-specific Audio Streaming Endpoint Descriptor (EP2)
    0x07, // bLength
    0x25, // bDescriptorType: CS_ENDPOINT
    0x01, // bDescriptorSubtype: GENERAL
    0x00, // Attributes: No controls
    0x00, // Lock Delay Units
    0x00, 0x00, // Lock Delay

    // --- INTERFACE DESCRIPTOR (3.0): HID (DS4 Gamepad + Touchpad) ---
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x03, // bInterfaceNumber: 3
    0x00, // bAlternateSetting: 0
    0x02, // bNumEndpoints: 2 (IN + OUT)
    0x03, // bInterfaceClass: HID
    0x00, // bInterfaceSubClass: None
    0x00, // bInterfaceProtocol: None
    0x00, // iInterface

    // HID Descriptor
    0x09, // bLength: 9
    0x21, // bDescriptorType (HID)
    0x11, 0x01, // bcdHID: 1.11
    0x00, // bCountryCode: Not localized
    0x01, // bNumDescriptors: 1 report descriptor
    0x22, // bDescriptorType: Report
    0xFB, 0x01, // wDescriptorLength: 507 (the real DS4 v2's report descriptor)

    // Endpoint Descriptor (HID IN: EP4)
    0x07, // bLength
    0x05, // bDescriptorType (ENDPOINT)
    0x84, // bEndpointAddress: IN EP4
    0x03, // bmAttributes: Interrupt
    0x40, 0x00, // wMaxPacketSize: 64
    0x01, // bInterval: 1 (polling every 4ms -> 1ms)

    // Endpoint Descriptor (HID OUT: EP3)
    0x07, // bLength
    0x05, // bDescriptorType (ENDPOINT)
    0x03, // bEndpointAddress: OUT EP3
    0x03, // bmAttributes: Interrupt
    0x40, 0x00, // wMaxPacketSize: 64
    0x01, // bInterval: 1 (polling every 4ms -> 1ms)

#if ENABLE_SERIAL
    // --- CDC ACM (USB Serial) ---
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, 0x85, 0x08, 0x06, 0x86, 0x40),
#endif
#ifdef ENABLE_WAKE_HID
    // --- INTERFACE DESCRIPTOR (HID Boot Keyboard, wake key only) ---
    // EP IN 0x87 (chosen to avoid collision with CDC notification EP 0x85
    // when ENABLE_SERIAL is also defined).
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    ITF_NUM_HID_KBD, // bInterfaceNumber
    0x00, // bAlternateSetting: 0
    0x01, // bNumEndpoints: 1 (IN only)
    0x03, // bInterfaceClass: HID
    0x01, // bInterfaceSubClass: Boot
    0x01, // bInterfaceProtocol: Keyboard
    0x00, // iInterface

    // HID Descriptor (keyboard)
    0x09, // bLength
    0x21, // bDescriptorType (HID)
    0x11, 0x01, // bcdHID: 1.11
    0x00, // bCountryCode
    0x01, // bNumDescriptors
    0x22, // bDescriptorType: Report
    0x2D, 0x00, // wDescriptorLength: 45 (sizeof desc_hid_report_kbd)

    // Endpoint Descriptor (HID IN: EP7)
    0x07, // bLength
    0x05, // bDescriptorType (ENDPOINT)
    0x87, // bEndpointAddress: IN EP7
    0x03, // bmAttributes: Interrupt
    0x08, 0x00, // wMaxPacketSize: 8 (boot keyboard report)
    0x0A, // bInterval: 10ms
#endif
};

TU_VERIFY_STATIC(CONFIG_DESC_LEN_BASE == CONFIG_DESC_HDR_LEN + CONFIG_DESC_LEN_AUDIO + CONFIG_DESC_LEN_HID,
                 "audio/HID block lengths do not add up to the base configuration descriptor");

// Scratch for the audio-less variant of the configuration descriptor, built on
// demand from descriptor_configuration above.
static uint8_t descriptor_configuration_no_audio[CONFIG_DESC_LEN_TOTAL - CONFIG_DESC_LEN_AUDIO];

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index; // for multiple configurations
    auto bInterval = 0x01;
    switch (get_config().polling_rate_mode) {
        case 0:
            bInterval = 0x05; // stock: the real DS4 v2's HID bInterval
            break;
        case 1:
            bInterval = 0x02;
            break;
        case 2:
            bInterval = 0x01;
            break;
    }
    constexpr auto offset = CONFIG_DESC_LEN_BASE;
    descriptor_configuration[offset - 1] = bInterval;
    descriptor_configuration[offset - 8] = bInterval;

    // Wake / Game Bar are runtime features. Advertise REMOTE_WAKEUP only when wake is
    // on, and include the keyboard interface (the LAST descriptor block) only when wake
    // OR the Game Bar shortcut is on. With both off this is byte-identical to the base.
    const bool wake = get_config().enable_wake;
    const bool kbd = wake || get_config().ps_shortcut_enabled;
    descriptor_configuration[7] = wake ? 0xE0 : 0xC0; // bmAttributes (REMOTE_WAKEUP bit)
    const uint16_t total = kbd ? CONFIG_DESC_LEN_TOTAL
                               : (uint16_t) (CONFIG_DESC_LEN_TOTAL - CONFIG_DESC_LEN_WAKE_KBD);
    descriptor_configuration[2] = (uint8_t) (total & 0xFF);                  // wTotalLength lo
    descriptor_configuration[3] = (uint8_t) (total >> 8);                    // wTotalLength hi
    descriptor_configuration[4] = kbd ? ITF_NUM_TOTAL : (ITF_NUM_TOTAL - 1); // bNumInterfaces
    if (usb_audio_exposed()) return descriptor_configuration;

    // Jack empty (audio_follow_jack, see usb.cpp): serve the same descriptor
    // with the audio function cut out, so the host enumerates a plain gamepad
    // with no audio device at all.
    // Cheap guard on the hand-maintained byte offsets above: the byte right
    // after the audio block must be the HID interface descriptor. If an edit to
    // descriptor_configuration ever moves that, serve the full descriptor
    // rather than a malformed one.
    if (descriptor_configuration[CONFIG_DESC_HDR_LEN + CONFIG_DESC_LEN_AUDIO + 1] != TUSB_DESC_INTERFACE) {
        printf("[USB] audio block length mismatch -- keeping the audio function\n");
        return descriptor_configuration;
    }

    uint8_t *out = descriptor_configuration_no_audio;
    const uint16_t tail = (uint16_t) (total - CONFIG_DESC_HDR_LEN - CONFIG_DESC_LEN_AUDIO);
    memcpy(out, descriptor_configuration, CONFIG_DESC_HDR_LEN);
    memcpy(out + CONFIG_DESC_HDR_LEN,
           descriptor_configuration + CONFIG_DESC_HDR_LEN + CONFIG_DESC_LEN_AUDIO, tail);
    const uint16_t no_audio_total = (uint16_t) (CONFIG_DESC_HDR_LEN + tail);
    out[2] = (uint8_t) (no_audio_total & 0xFF); // wTotalLength lo
    out[3] = (uint8_t) (no_audio_total >> 8);   // wTotalLength hi
    out[4] -= AUDIO_ITF_COUNT;                  // bNumInterfaces
    // Interface numbers must stay a gapless 0..n-1 run, so pull everything that
    // followed the audio function down by three. Interface and IAD descriptors
    // both carry the number at offset 2 (bInterfaceNumber / bFirstInterface).
    for (uint16_t i = CONFIG_DESC_HDR_LEN; i + 2 < no_audio_total && out[i]; i += out[i]) {
        if (out[i + 1] == TUSB_DESC_INTERFACE || out[i + 1] == TUSB_DESC_INTERFACE_ASSOCIATION) {
            out[i + 2] -= AUDIO_ITF_COUNT;
        }
    }
    return out;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

// DualShock 4 (CUH-ZCT2) USB HID report descriptor, byte-identical to a real
// v2 controller (PID 054C:09CC). Sourced from a USB capture of the hardware;
// several DS4 variants exist with slightly different descriptors, so don't
// swap this for another dump without re-checking against a real v2.
uint8_t const desc_hid_report_ds4[] = {
    0x05, 0x01,             // Usage Page (Generic Desktop)
    0x09, 0x05,             // Usage (Game Pad)
    0xA1, 0x01,             // Collection (Application)
      0x85, 0x01,             // Report ID (1)
      0x09, 0x30,             // Usage (X)
      0x09, 0x31,             // Usage (Y)
      0x09, 0x32,             // Usage (Z)
      0x09, 0x35,             // Usage (Rz)
      0x15, 0x00,             // Logical Minimum (0)
      0x26, 0xFF, 0x00,       // Logical Maximum (255)
      0x75, 0x08,             // Report Size (8)
      0x95, 0x04,             // Report Count (4)
      0x81, 0x02,             // Input (Data,Var)
      0x09, 0x39,             // Usage (Hat switch)
      0x15, 0x00,             // Logical Minimum (0)
      0x25, 0x07,             // Logical Maximum (7)
      0x35, 0x00,             // Physical Minimum (0)
      0x46, 0x3B, 0x01,       // Physical Maximum (315)
      0x65, 0x14,             // Unit (0x14)
      0x75, 0x04,             // Report Size (4)
      0x95, 0x01,             // Report Count (1)
      0x81, 0x42,             // Input (Data,Var,Null)
      0x65, 0x00,             // Unit (0x0)
      0x05, 0x09,             // Usage Page (Button)
      0x19, 0x01,             // Usage Minimum (1)
      0x29, 0x0E,             // Usage Maximum (14)
      0x15, 0x00,             // Logical Minimum (0)
      0x25, 0x01,             // Logical Maximum (1)
      0x75, 0x01,             // Report Size (1)
      0x95, 0x0E,             // Report Count (14)
      0x81, 0x02,             // Input (Data,Var)
      0x06, 0x00, 0xFF,       // Usage Page (0xFF00)
      0x09, 0x20,             // Usage (0x20)
      0x75, 0x06,             // Report Size (6)
      0x95, 0x01,             // Report Count (1)
      0x15, 0x00,             // Logical Minimum (0)
      0x25, 0x7F,             // Logical Maximum (127)
      0x81, 0x02,             // Input (Data,Var)
      0x05, 0x01,             // Usage Page (Generic Desktop)
      0x09, 0x33,             // Usage (Rx)
      0x09, 0x34,             // Usage (Ry)
      0x15, 0x00,             // Logical Minimum (0)
      0x26, 0xFF, 0x00,       // Logical Maximum (255)
      0x75, 0x08,             // Report Size (8)
      0x95, 0x02,             // Report Count (2)
      0x81, 0x02,             // Input (Data,Var)
      0x06, 0x00, 0xFF,       // Usage Page (0xFF00)
      0x09, 0x21,             // Usage (0x21)
      0x95, 0x36,             // Report Count (54)
      0x81, 0x02,             // Input (Data,Var)
      0x85, 0x05,             // Report ID (5)
      0x09, 0x22,             // Usage (0x22)
      0x95, 0x1F,             // Report Count (31)
      0x91, 0x02,             // Output (Data,Var)
      0x85, 0x04,             // Report ID (4)
      0x09, 0x23,             // Usage (0x23)
      0x95, 0x24,             // Report Count (36)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x02,             // Report ID (2)
      0x09, 0x24,             // Usage (0x24)
      0x95, 0x24,             // Report Count (36)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x08,             // Report ID (8)
      0x09, 0x25,             // Usage (0x25)
      0x95, 0x03,             // Report Count (3)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x10,             // Report ID (16)
      0x09, 0x26,             // Usage (0x26)
      0x95, 0x04,             // Report Count (4)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x11,             // Report ID (17)
      0x09, 0x27,             // Usage (0x27)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x12,             // Report ID (18)
      0x06, 0x02, 0xFF,       // Usage Page (0xFF02)
      0x09, 0x21,             // Usage (0x21)
      0x95, 0x0F,             // Report Count (15)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x13,             // Report ID (19)
      0x09, 0x22,             // Usage (0x22)
      0x95, 0x16,             // Report Count (22)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x14,             // Report ID (20)
      0x06, 0x05, 0xFF,       // Usage Page (0xFF05)
      0x09, 0x20,             // Usage (0x20)
      0x95, 0x10,             // Report Count (16)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x15,             // Report ID (21)
      0x09, 0x21,             // Usage (0x21)
      0x95, 0x2C,             // Report Count (44)
      0xB1, 0x02,             // Feature (Data,Var)
      0x06, 0x80, 0xFF,       // Usage Page (0xFF80)
      0x85, 0x80,             // Report ID (128)
      0x09, 0x20,             // Usage (0x20)
      0x95, 0x06,             // Report Count (6)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x81,             // Report ID (129)
      0x09, 0x21,             // Usage (0x21)
      0x95, 0x06,             // Report Count (6)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x82,             // Report ID (130)
      0x09, 0x22,             // Usage (0x22)
      0x95, 0x05,             // Report Count (5)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x83,             // Report ID (131)
      0x09, 0x23,             // Usage (0x23)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x84,             // Report ID (132)
      0x09, 0x24,             // Usage (0x24)
      0x95, 0x04,             // Report Count (4)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x85,             // Report ID (133)
      0x09, 0x25,             // Usage (0x25)
      0x95, 0x06,             // Report Count (6)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x86,             // Report ID (134)
      0x09, 0x26,             // Usage (0x26)
      0x95, 0x06,             // Report Count (6)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x87,             // Report ID (135)
      0x09, 0x27,             // Usage (0x27)
      0x95, 0x23,             // Report Count (35)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x88,             // Report ID (136)
      0x09, 0x28,             // Usage (0x28)
      0x95, 0x22,             // Report Count (34)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x89,             // Report ID (137)
      0x09, 0x29,             // Usage (0x29)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x90,             // Report ID (144)
      0x09, 0x30,             // Usage (0x30)
      0x95, 0x05,             // Report Count (5)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x91,             // Report ID (145)
      0x09, 0x31,             // Usage (0x31)
      0x95, 0x03,             // Report Count (3)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x92,             // Report ID (146)
      0x09, 0x32,             // Usage (0x32)
      0x95, 0x03,             // Report Count (3)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0x93,             // Report ID (147)
      0x09, 0x33,             // Usage (0x33)
      0x95, 0x0C,             // Report Count (12)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA0,             // Report ID (160)
      0x09, 0x40,             // Usage (0x40)
      0x95, 0x06,             // Report Count (6)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA1,             // Report ID (161)
      0x09, 0x41,             // Usage (0x41)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA2,             // Report ID (162)
      0x09, 0x42,             // Usage (0x42)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA3,             // Report ID (163)
      0x09, 0x43,             // Usage (0x43)
      0x95, 0x30,             // Report Count (48)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA4,             // Report ID (164)
      0x09, 0x44,             // Usage (0x44)
      0x95, 0x0D,             // Report Count (13)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA5,             // Report ID (165)
      0x09, 0x45,             // Usage (0x45)
      0x95, 0x15,             // Report Count (21)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA6,             // Report ID (166)
      0x09, 0x46,             // Usage (0x46)
      0x95, 0x15,             // Report Count (21)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xF0,             // Report ID (240)
      0x09, 0x47,             // Usage (0x47)
      0x95, 0x3F,             // Report Count (63)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xF1,             // Report ID (241)
      0x09, 0x48,             // Usage (0x48)
      0x95, 0x3F,             // Report Count (63)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xF2,             // Report ID (242)
      0x09, 0x49,             // Usage (0x49)
      0x95, 0x0F,             // Report Count (15)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA7,             // Report ID (167)
      0x09, 0x4A,             // Usage (0x4A)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA8,             // Report ID (168)
      0x09, 0x4B,             // Usage (0x4B)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xA9,             // Report ID (169)
      0x09, 0x4C,             // Usage (0x4C)
      0x95, 0x08,             // Report Count (8)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAA,             // Report ID (170)
      0x09, 0x4E,             // Usage (0x4E)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAB,             // Report ID (171)
      0x09, 0x4F,             // Usage (0x4F)
      0x95, 0x39,             // Report Count (57)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAC,             // Report ID (172)
      0x09, 0x50,             // Usage (0x50)
      0x95, 0x39,             // Report Count (57)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAD,             // Report ID (173)
      0x09, 0x51,             // Usage (0x51)
      0x95, 0x0B,             // Report Count (11)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAE,             // Report ID (174)
      0x09, 0x52,             // Usage (0x52)
      0x95, 0x01,             // Report Count (1)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xAF,             // Report ID (175)
      0x09, 0x53,             // Usage (0x53)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xB0,             // Report ID (176)
      0x09, 0x54,             // Usage (0x54)
      0x95, 0x3F,             // Report Count (63)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xB1,             // Report ID (177)
      0x09, 0x55,             // Usage (0x55)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xB2,             // Report ID (178)
      0x09, 0x56,             // Usage (0x56)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xE0,             // Report ID (224)
      0x09, 0x57,             // Usage (0x57)
      0x95, 0x02,             // Report Count (2)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xB3,             // Report ID (179)
      0x09, 0x55,             // Usage (0x55)
      0x95, 0x3F,             // Report Count (63)
      0xB1, 0x02,             // Feature (Data,Var)
      0x85, 0xB4,             // Report ID (180)
      0x09, 0x55,             // Usage (0x55)
      0x95, 0x3F,             // Report Count (63)
      0xB1, 0x02,             // Feature (Data,Var)
    0xC0,                   // End Collection
};
static_assert(sizeof(desc_hid_report_ds4) == 507);

#ifdef ENABLE_WAKE_HID
// 41-byte boot-keyboard report descriptor (modifier byte + reserved + 6 keycodes,
// no Report ID -- boot protocol forbids one and avoids collision with the gamepad's Report ID 1).
uint8_t const desc_hid_report_kbd[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       //   Usage Minimum (Left Control)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data,Var,Abs) -- modifier byte
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x01,       //   Input (Const) -- reserved byte
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x65,       //   Logical Maximum (101)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0x65,       //   Usage Maximum (101)
    0x81, 0x00,       //   Input (Data,Array) -- 6 keycodes
    0xC0              // End Collection
};
_Static_assert(sizeof(desc_hid_report_kbd) == 45, "keyboard report descriptor length must match wDescriptorLength in config descriptor");
#endif

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf) {
#ifdef ENABLE_WAKE_HID
    // HID instance 1 is the wake-only boot keyboard added by ENABLE_WAKE_HID.
    if (itf == 1) return desc_hid_report_kbd;
#endif
    (void) itf;
    return desc_hid_report_ds4;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const *string_desc_arr[] =
{
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Sony Interactive Entertainment", // 1: Manufacturer
    NULL, // 2: Product
    NULL, // 3: Serials will use unique ID if possible
#if ENABLE_SERIAL
    "USB Serial", // 4: CDC interface
#endif
};

static uint16_t _desc_str[60 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    size_t chr_count;

    string_desc_arr[2] = "Wireless Controller";

    switch (index) {
        case STRID_LANGID:
            memcpy(&_desc_str[1], string_desc_arr[0], 2);
            chr_count = 1;
            break;

        case STRID_SERIAL:
            chr_count = board_usb_get_serial(_desc_str + 1, 32) + 1;
            _desc_str[chr_count] = '3'; // refresh windows cache (bumped for the real-DS4v2 mono 16 kHz mic)
            break;

        default:
            // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
            // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

            if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

            const char *str = string_desc_arr[index];

            // Cap at max char
            chr_count = strlen(str);
            size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
            if (chr_count > max_count) chr_count = max_count;

            // Convert ASCII string into UTF-16
            for (size_t i = 0; i < chr_count; i++) {
                _desc_str[1 + i] = str[i];
            }
            break;
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}

#ifdef ENABLE_WAKE_HID
//--------------------------------------------------------------------+
// Microsoft OS 2.0 descriptors (carried via BOS).
//
// Why this is here: the dongle is a composite device with USB Audio Class
// interfaces. By default Windows audio engine policy keeps USB audio devices
// at D0 even during system S3, blocking selective-suspend for the whole
// composite. Without selective-suspend the device never enters USB suspend,
// so tud_remote_wakeup() never works -- breaking wake-on-PS.
//
// MS OS 2.0 lets us tell Windows "yes, please selective-suspend this audio
// function": we set the registry property "SelectiveSuspendEnabled" = 1 on
// the audio function (interface 0). This causes Windows to write
//   HKLM\SYSTEM\CurrentControlSet\Enum\USB\<VID&PID>\<instance>
//        \Device Parameters\SelectiveSuspendEnabled = 1
// at enumeration time, opting our audio function in to selective suspend
// without breaking haptics.
//
// Reference: "Microsoft OS 2.0 Descriptors Specification".
//--------------------------------------------------------------------+

#define MS_OS_20_VENDOR_CODE 0x01

// Total length of the MS OS 2.0 descriptor set:
//   Set Header (10) + Config Subset (8) + Function Subset (8) +
//   Registry Property Feature (10 fixed + 48 name + 4 data = 62) = 88 bytes.
// Used in BOS platform capability descriptor; verified by static_assert below.
#define MS_OS_20_DESC_LEN    88

#define BOS_TOTAL_LEN        (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

uint8_t const desc_bos[] = {
    // BOS header
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    // Platform capability: MS OS 2.0
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, MS_OS_20_VENDOR_CODE)
};

uint8_t const *tud_descriptor_bos_cb(void) {
    // BOS carries the MS OS 2.0 selective-suspend opt-in, only meaningful for wake
    // and only while the audio function is exposed (its registry property names
    // interface 0, which is the audio function). When either is off the device is
    // USB 2.0 and the host won't ask -- guard anyway.
    if (!get_config().enable_wake || !usb_audio_exposed()) return nullptr;
    return desc_bos;
}

uint8_t const desc_ms_os_20[] = {
    // --- Set Header (10 bytes) ---
    U16_TO_U8S_LE(0x000A),                                    // wLength
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),            // wDescriptorType
    U32_TO_U8S_LE(0x06030000),                                // dwWindowsVersion = Win 8.1+
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN),                         // wTotalLength

    // --- Configuration Subset (8 bytes) ---
    U16_TO_U8S_LE(0x0008),                                    // wLength
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),      // wDescriptorType
    0x00,                                                     // bConfigurationValue (config index, 0)
    0x00,                                                     // bReserved
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),                  // wTotalLength of this subset

    // --- Function Subset for the Audio function (8 bytes) ---
    // Audio Control is interface 0; AudioStreaming OUT/IN are 1/2 -- this
    // subset covers all three because they belong to the same function.
    U16_TO_U8S_LE(0x0008),                                    // wLength
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),           // wDescriptorType
    0x00,                                                     // bFirstInterface (audio control)
    0x00,                                                     // bReserved
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),           // wSubsetLength

    // --- Feature: Registry Property "SelectiveSuspendEnabled" = 1 (62 bytes) ---
    U16_TO_U8S_LE(0x003E),                                    // wLength = 62
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),             // wDescriptorType
    U16_TO_U8S_LE(0x0004),                                    // wPropertyDataType = REG_DWORD_LITTLE_ENDIAN
    U16_TO_U8S_LE(48),                                        // wPropertyNameLength = 48 bytes (24 UTF-16 chars)
    // PropertyName "SelectiveSuspendEnabled\0" UTF-16LE (48 bytes)
    'S',0, 'e',0, 'l',0, 'e',0, 'c',0, 't',0, 'i',0, 'v',0,
    'e',0, 'S',0, 'u',0, 's',0, 'p',0, 'e',0, 'n',0, 'd',0,
    'E',0, 'n',0, 'a',0, 'b',0, 'l',0, 'e',0, 'd',0,  0,0,
    U16_TO_U8S_LE(0x0004),                                    // wPropertyDataLength = 4 bytes
    U32_TO_U8S_LE(0x00000001),                                // PropertyData = 1 (enabled)
};
TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "MS OS 2.0 descriptor length mismatch");

// Vendor-class control transfer hook. Windows reads BOS, sees the MS OS 2.0
// platform capability, then issues this vendor request to fetch the
// descriptor set itself.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    if (!get_config().enable_wake || !usb_audio_exposed()) return false;
    if (stage != CONTROL_STAGE_SETUP) return true;
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) return false;
    if (request->bRequest == MS_OS_20_VENDOR_CODE && request->wIndex == 7) {
        // wIndex == 7 -> MS_OS_20_DESCRIPTOR_INDEX
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)desc_ms_os_20, sizeof(desc_ms_os_20));
    }
    return false;
}
#endif // ENABLE_WAKE_HID
