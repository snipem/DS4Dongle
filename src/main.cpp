//
// Created by awalol on 2026/3/4.
// DS4Dongle fork: bridges a Bluetooth DualShock 4 to a wired-DS4 USB persona.
//

#include <cstdio>
#include <cstring>
#include "bsp/board_api.h"
#include "bt.h"
#include "button_functions.h"
#include "utils.h"
#include "audio.h"
#include "usb.h"
#if ENABLE_DEBUG
#include "debug.h"
#endif
#include "wake.h"
#ifdef ENABLE_WAKE_HID
#include "ps_shortcut.h"
#endif
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#if ENABLE_SERIAL
#include "pico/stdio_usb.h"
#endif
#include "config.h"
#include "cmd.h"
#if ENABLE_BATT_LED
#include "battery_led.h"
#endif

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

bool spk_active = false;

// Cached USB input report 0x01 payload (63 bytes). Starts neutral: centered
// sticks, hat released (0x08).
uint8_t interrupt_in_data[63] = {
    0x80, 0x80, 0x80, 0x80, // LX LY RX RY
    0x08, 0x00, 0x00,       // hat/face, misc, PS/TP + counter
    0x00, 0x00,             // L2 R2
};

critical_section_t report_cs;
volatile bool report_dirty = false;

void __not_in_flash_func(interrupt_loop)() {
    if (!tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send.
    uint8_t safe_report[63];


    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");

            // If the report failed to queue, restore the dirty flag
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

// DS4 BT input report 0x11: a1 11 c0 00 | 63+ byte payload that matches the
// USB report 0x01 payload byte-for-byte (sticks, buttons, IMU, battery,
// trackpad), so bridging is a plain copy from data+4.
void __not_in_flash_func(on_bt_data)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // Log unfamiliar interrupt reports (rate-limited): 0x11 is the normal
    // input; 0x12..0x19 are state + headset-mic audio in increasing sizes
    // (the controller picks the size; handled below).
    if (channel == INTERRUPT && len >= 2 && (data[1] < 0x11 || data[1] > 0x19)) {
        static uint32_t last_log_ms = 0;
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_log_ms >= 1000) {
            last_log_ms = now;
            printf("[BT] input report 0x%02X len=%u:", data[1], len);
            for (int i = 0; i < 16 && i < len; i++) printf(" %02x", data[i]);
            printf("\n");
        }
    }
    // Mic audio rides in reports 0x12..0x19 after a state block that is
    // layout-identical to 0x11's.
    if (channel == INTERRUPT && len >= 4 + 63 && data[1] >= 0x12 && data[1] <= 0x19) {
        audio_mic_bt_data(data, len);
    }
    if (channel == INTERRUPT && len >= 4 + 63 && data[1] >= 0x11 && data[1] <= 0x19) {
        // During duplex audio some report variants were seen with a state
        // block that is NOT 0x11-layout (hid-playstation rejected the
        // forwarded reports with num_touch_reports=213). Only forward states
        // that pass a touch-count sanity check; audio parsing above is
        // unaffected. num_touch_reports is at payload offset 32.
        if (data[1] != 0x11 && data[4 + 32] > 4) {
            return;
        }
        // Battery/ext byte: bit5 = headset plugged into the controller jack.
        set_headset((data[4 + 29] & 0x20) != 0);

        // Wake-on-PS must observe every BT input report regardless of polling
        // mode (see wake.cpp).
        wake_on_bt_input(data + 4, len - 4);
        #ifdef ENABLE_WAKE_HID
        ps_shortcut_tick(data + 4, len - 4);
        #endif

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 4, 63);
#if ENABLE_BATT_LED
            battery_led_note_report();
#endif
            return;
        }

        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 4, 63);
        report_dirty = true;
        critical_section_exit(&report_cs);
#if ENABLE_BATT_LED
        battery_led_note_report();
#endif
    }
}

// Translate the BT calibration feature report (0x05) into USB (0x02) layout.
// The six gyro plus/minus calibration words are grouped all-plus-then-all-minus
// (pitch+, yaw+, roll+, pitch-, yaw-, roll-) over BT, but interleaved per axis
// (pitch+, pitch-, yaw+, yaw-, roll+, roll-) over USB (see hid-playstation
// dualshock4_get_calibration_data). `bt` points at the payload after the report
// id. Bias words (bytes 0-5) and accel words (bytes 18-35) share the same
// layout on both transports, so the memcpy handles them; only the gyro range
// words need reordering.
static void ds4_calib_bt_to_usb(const uint8_t *bt, uint8_t *usb) {
    memcpy(usb, bt, 36);
    usb[6] = bt[6];   usb[7] = bt[7];   // gyro pitch plus
    usb[8] = bt[12];  usb[9] = bt[13];  // gyro pitch minus
    usb[10] = bt[8];  usb[11] = bt[9];  // gyro yaw plus
    usb[12] = bt[14]; usb[13] = bt[15]; // gyro yaw minus
    usb[14] = bt[10]; usb[15] = bt[11]; // gyro roll plus
    usb[16] = bt[16]; usb[17] = bt[17]; // gyro roll minus
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        if (reqlen >= 8) {
            memset(buffer, 0, 8);
            return 8;
        }
        return 0;
    }
#endif
    (void) itf;
    (void) report_type;

    if (is_pico_cmd(report_id)) {
        return pico_cmd_get(report_id, buffer, reqlen);
    }

    // Auth (0xF0-0xF3): we can't proxy PS4 license auth; stall.
    if (report_id >= 0xF0 && report_id <= 0xF3) {
        return 0;
    }

    // 0x12 / 0x81: controller MAC (+ paired host MAC for 0x12). Synthesized
    // from the BT layer instead of a controller round-trip.
    if (report_id == 0x12 && reqlen >= 15) {
        uint8_t mac[6], host[6];
        bt_get_controller_mac(mac);
        bt_get_local_mac(host);
        memset(buffer, 0, 15);
        // MAC byte order is reversed (LSB first) in the report.
        for (int i = 0; i < 6; i++) buffer[i] = mac[5 - i];
        buffer[6] = 0x08;
        buffer[7] = 0x25;
        buffer[8] = 0x00;
        for (int i = 0; i < 6; i++) buffer[9 + i] = host[5 - i];
        return 15;
    }
    if (report_id == 0x81 && reqlen >= 6) {
        uint8_t mac[6];
        bt_get_controller_mac(mac);
        for (int i = 0; i < 6; i++) buffer[i] = mac[5 - i];
        return 6;
    }

    // Calibration: host asks for USB report 0x02; the data lives in BT report
    // 0x05 (already fetched at connect; re-request if the cache is cold).
    if (report_id == 0x02) {
        std::vector<uint8_t> calib = get_feature_data(0x05, 41);
        // Cached vector: [0x05][36 bytes calibration][CRC-32]
        if (calib.size() < 1 + 36 || reqlen < 36) {
            return 0;
        }
        ds4_calib_bt_to_usb(calib.data() + 1, buffer);
        return 36;
    }

    // Everything else: forward to the controller under the same report id.
    // BT feature responses carry a trailing CRC-32 that USB must not see.
    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (feature_data.size() <= 1 + 4) {
        return 0;
    }
    uint16_t payload_len = feature_data.size() - 1 - 4;
    if (payload_len > reqlen) payload_len = reqlen;
    memcpy(buffer, feature_data.data() + 1, payload_len);

    // 0xA3 firmware info: the BT payload is 3 bytes shorter than the USB
    // report. hid-playstation ("expected 49 got 45") and the dualshock-tools
    // clone check both require the full 48+id bytes; every field they parse
    // (build date, hw/sw versions, all < offset 0x2d) fits in the BT payload,
    // so zero-pad the tail.
    constexpr uint16_t A3_USB_PAYLOAD_LEN = 48;
    if (report_id == 0xA3 && payload_len < A3_USB_PAYLOAD_LEN && reqlen >= A3_USB_PAYLOAD_LEN) {
        memset(buffer + payload_len, 0, A3_USB_PAYLOAD_LEN - payload_len);
        payload_len = A3_USB_PAYLOAD_LEN;
    }
    return payload_len;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex); // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue); // bAlternateSetting

    if (itf == 1) {
        printf("[AUDIO] Set interface Speaker to alternate setting %d\n", alt);
        spk_active = alt;
    }
    if (itf == 2) { // ITF_NUM_AUDIO_STREAMING_IN (microphone)
        printf("[AUDIO] Set interface Microphone to alternate setting %d\n", alt);
        set_mic_active(alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        // Drop keyboard SET_REPORT (host LED state).
        return;
    }
#endif
    (void) itf;
    (void) report_type;

    if (is_pico_cmd(report_id)) {
#if ENABLE_VERBOSE
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n", buffer[0]);
#endif
        pico_cmd_set(report_id, buffer, bufsize);
        return;
    }

    // INTERRUPT OUT: USB output report 0x05 (rumble/LED/flash) maps 1:1 onto
    // BT report 0x11 bytes 3.. -- forward the payload as-is.
    if (report_id == 0 && bufsize >= 2 && buffer[0] == 0x05) {
        uint8_t payload[31]{};
        uint16_t n = bufsize - 1;
        if (n > sizeof(payload)) n = sizeof(payload);
        memcpy(payload, buffer + 1, n);
        if (get_config().lock_volume) {
            payload[0] &= 0x0F; // strip the volume-enable flag bits
        }
        ds4_output(payload, sizeof(payload));
    }
    // SET_REPORT path (report id in the request, buffer = payload only).
    if (report_id == 0x05) {
        uint8_t payload[31]{};
        uint16_t n = bufsize;
        if (n > sizeof(payload)) n = sizeof(payload);
        memcpy(payload, buffer, n);
        if (get_config().lock_volume) {
            payload[0] &= 0x0F;
        }
        ds4_output(payload, sizeof(payload));
    }
}

int main() {
#if SYS_CLOCK_KHZ != 150000
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
#endif

    board_init();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#if !ENABLE_SERIAL
    sleep_ms(150);
    tud_disconnect();
#endif
    board_init_after_tusb();
#if ENABLE_SERIAL
    stdio_usb_init();
#endif

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

#if ENABLE_BATT_LED
    battery_led_init();
#endif

#if !ENABLE_SERIAL
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            } else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
#endif

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    wake_init();

    config_load();

    bt_init();
    bt_register_data_callback(on_bt_data);

    audio_init();

#if !ENABLE_SERIAL
    watchdog_enable(1000, true);
#endif

    while (1) {
#if !ENABLE_SERIAL
        watchdog_update();
#endif
        cyw43_arch_poll();
        tud_task();
        wake_task();
        audio_loop();
        usb_audio_exposure_task();
#if ENABLE_DEBUG
        debug_log_core1_stack_usage();
#endif
        interrupt_loop();
#if ENABLE_BATT_LED
        battery_led_tick();
#endif
        button_check();
        bt_inquiring_led();
    }
}
