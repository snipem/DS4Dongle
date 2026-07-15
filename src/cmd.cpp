//
// Created by awalol on 2026/5/4.
//

#include "cmd.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "bt.h"
#include "config.h"
#include "device/usbd.h"
#include "pico/time.h"
#include "audio.h"
#include "wake.h"

extern bool spk_active;

bool is_pico_cmd(uint8_t report_id) {
    if (report_id == 0xf6 ||
        report_id == 0xf7 ||
        report_id == 0xf8 ||
        report_id == 0xf9
    ) {
        return true;
    }
    return false;
}

uint16_t pico_cmd_get(uint8_t report_id, uint8_t *buffer, uint16_t reqlen) {
    if (report_id == 0xf7) {
        printf("[HID] Receive 0xf7 getting config\n");
        if (sizeof(Config_body) > reqlen) {
            printf("[Config] Warning: Config_body overflow\n");
        }
        const auto len = std::min(sizeof(Config_body),static_cast<size_t>(reqlen));
        memcpy(buffer,&get_config(),len);
        return len;
    }
    if (report_id == 0xf8) {
        printf("[HID] Receive 0xf8 getting firmware version\n");
        const auto len = std::min(strlen(PICO_PROGRAM_VERSION_STRING), static_cast<size_t>(reqlen));
        memcpy(buffer, PICO_PROGRAM_VERSION_STRING, len);
        return len;
    }
    if (report_id == 0xf9) {
        // [-128,0]
        int8_t rssi = 0;
        bt_get_signal_strength(&rssi);
        if (reqlen == 0) {
            return 0;
        }
        buffer[0] = rssi;
        // byte 1: real audio gating state, for the config UI to display.
        //   bit7 = valid marker (firmware without this byte leaves it 0)
        //   bit0 = controller mic actually streaming (host opened it AND mic_select != disable)
        //   bit1 = controller speaker actually driven (host opened it AND speaker_select != disable)
        if (reqlen >= 2) {
            uint8_t flags = 0x80;
            if (audio_mic_active() && get_config().mic_select != 3) flags |= 0x01;
            if (spk_active && get_config().speaker_select != 3) flags |= 0x02;
            buffer[1] = flags;
            return 2;
        }
#if ENABLE_VERBOSE
        printf("[HID] 0xf9 RSSI=%d raw=0x%02X\n", rssi, buffer[0]);
#endif
        return 1;
    }
    return 0;
}

void pico_cmd_set(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize) {
    (void) report_id;
    if (bufsize == 0) {
        return;
    }

    // 0x01 update config in variable
    // 0x02 write config to flash
    // 0x03 reconnect tinyusb device;
    // 0x05 set the raw DS4 volume byte (debug: bypasses the USB dB mapping,
    //      for the jack->line-in RMS sweep; volatile, next USB volume event
    //      overwrites it)
    // 0x06 send a raw DS4 0x11 output-report payload (debug: poke
    //      flag bytes from the host, e.g. for the mic-enable research)
    // 0x07 set the 0x11 output header byte 2 for subsequent outputs
    //      (debug: bits 0-2 = EnableMic, bit7 = EnableAudio)
    if (buffer[0] == 0x01) {
#if ENABLE_VERBOSE
        printf("[CMD] Enter config set func\n");
#endif
        set_config(buffer + 1, bufsize - 1);
    }
    if (buffer[0] == 0x02) {
        printf("[CMD] Enter config save func\n");
        config_save();
    }
    if (buffer[0] == 0x03) {
        printf("[CMD] Enter tud reconnect func\n");
        wake_note_usb_reconnect();   // this disconnect is intentional, not a host sleep
        tud_disconnect();
        sleep_ms(150);
        tud_connect();
    }
    if (buffer[0] == 0x05 && bufsize >= 2) {
        printf("[CMD] Set raw volume byte %u\n", buffer[1]);
        ds4_set_volume(buffer[1], buffer[1], buffer[1]);
    }
    if (buffer[0] == 0x06 && bufsize >= 2) {
        uint8_t payload[31]{};
        uint16_t n = bufsize - 1;
        if (n > sizeof(payload)) n = sizeof(payload);
        memcpy(payload, buffer + 1, n);
        printf("[CMD] Raw output report, flags 0x%02X\n", payload[0]);
        ds4_output(payload, sizeof(payload));
    }
    if (buffer[0] == 0x07 && bufsize >= 2) {
        printf("[CMD] Output header byte2 = 0x%02X\n", buffer[1]);
        ds4_set_output_hdr2(buffer[1]);
    }
}
