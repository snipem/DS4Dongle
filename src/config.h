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

#if OPINIONATED
// Wi-Fi Wake-on-LAN settings (opinionated build only). Stored in the config
// flash page behind Config, with its own magic + CRC so it never forces a
// Config_body reset, and deliberately NOT part of Config_body: the password
// would not fit the 62-byte 0xF6 update payload, and it must never be readable
// back over GET_REPORT 0xF7. Set field-wise via 0xF6 funcid 0x08, read (minus
// the password) via 0xFA, persisted by the same funcid 0x02 save.
constexpr uint8_t WOL_SSID_MAX = 32;
constexpr uint8_t WOL_PASSWORD_MAX = 63; // WPA2 passphrase limit

struct __attribute__((packed)) WolConfig {
    uint32_t magic;
    uint32_t crc32;           // over everything after this field
    uint8_t enabled;          // bool
    uint8_t mac[6];           // target PC NIC
    uint8_t ssid_len;
    char ssid[WOL_SSID_MAX];
    uint8_t password_len;     // 0: open network
    char password[WOL_PASSWORD_MAX + 1];
};

WolConfig& get_wol_config();
#endif

void config_default();
void config_load();
bool config_save();
Config_body& get_config();
void set_config(const uint8_t *new_config, const uint16_t len);
void config_valid();
void set_config(const Config_body &new_config);
#endif //DS5_BRIDGE_CONFIG_H
