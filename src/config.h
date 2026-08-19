//
// Created by awalol on 2026/5/4.
//

#ifndef DS5_BRIDGE_CONFIG_H
#define DS5_BRIDGE_CONFIG_H

#include <cstdint>

struct __attribute__((packed)) Config_body {
    uint8_t config_version; // Config Version
    float haptics_gain; // [1.0,2.0]
    uint8_t speaker_volume; // [0,127] // unused
    uint8_t headset_volume; // [0,127] // max 0x7f // unused
    uint8_t speaker_gain; // [0,7] (0: auto)
    uint8_t inactive_time; // [0,60] min (0: disable)
    uint8_t disable_pico_led; // bool
    uint8_t polling_rate_mode; // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audio_buffer_length; // [16,127]
    uint8_t controller_mode; // 0: DS5, 1: DSE, 2: Auto
    uint8_t enable_usb_sn; // 0: disable,1: enable
    uint8_t ps_shortcut_enabled; // 0: disabled, 1: enabled (Xbox Game Bar via HID keyboard)
    uint8_t mic_select; // 0: auto, 1: builtin, 2: headphone, 3: disable
    uint8_t speaker_select; // 0: auto, 1: builtin, 2: headphone, 3: disable
    uint8_t enable_wake; // bool: 0 disabled (default), 1 wake host on PS press (USB remote wakeup)
    uint8_t trigger_reduce; // [0,10] (0: auto)
    uint8_t lock_volume; // bool
    uint8_t audio_follow_jack; // bool: 1 (default) only exposes the USB audio device while a
                               // headset is plugged into the controller's 3.5 mm jack
};

struct __attribute__((packed)) Config {
    uint32_t magic;
    uint32_t crc32; // Config_body crc32, only calc and verify when save
    uint16_t size;  // Config_body size
    Config_body body;
};

void config_default();
void config_load();
bool config_save();
Config_body& get_config();
void set_config(const uint8_t *new_config, const uint16_t len);
void config_valid();
void set_config(const Config_body &new_config);
#endif //DS5_BRIDGE_CONFIG_H
