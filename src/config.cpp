//
// Created by awalol on 2026/5/4.
//

#include "config.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

#include "bt.h"
#include "utils.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/cyw43_arch.h"
#include "pico/flash.h"
#include "pico/btstack_flash_bank.h"

constexpr uint32_t CONFIG_MAGIC = 0x66ccff00;
constexpr uint16_t CONFIG_VERSION = 6; // 如果想要强制重置配置，再更新 CONFIG_VERSION。
// The sector right below BTstack's pairing storage -- NOT the last sector of
// flash. picotool appends an RP2350-E10 workaround block at 0x10ffff00 (the top
// of a 16 MB address space) to every UF2; on the Pico 2 W's 4 MB chip that
// address wraps onto the last sector, so every UF2 flash erased a config kept
// there. The SDK keeps the BTstack bank clear of that sector for the same
// reason (PICO_FLASH_BANK_STORAGE_OFFSET), so sit directly below it.
constexpr uint32_t CONFIG_FLASH_OFFSET = PICO_FLASH_BANK_STORAGE_OFFSET - FLASH_SECTOR_SIZE;
static Config config{};

#if OPINIONATED
constexpr uint32_t WOL_CONFIG_MAGIC = 0x574f4c01; // "WOL" v1
// WolConfig lives in the same flash page as Config, at a fixed offset so Config
// can still grow without moving it.
constexpr uint32_t WOL_CONFIG_PAGE_OFFSET = 128;
static WolConfig wol_config{};
static_assert(sizeof(Config) <= WOL_CONFIG_PAGE_OFFSET);
static_assert(WOL_CONFIG_PAGE_OFFSET + sizeof(WolConfig) <= FLASH_PAGE_SIZE);
#endif

// 编译期保护
// 判断Config结构体是否能放进flash 256bytes
static_assert(sizeof(Config) <= FLASH_PAGE_SIZE);
// 配置区起始地址必须按 flash sector 对齐。
static_assert(CONFIG_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);

uint32_t calc_config_crc(const Config &con) {
    return crc32(reinterpret_cast<const uint8_t *>(&con.body), sizeof(Config_body));
}

const Config *flash_config() {
    return reinterpret_cast<const Config *>(XIP_BASE + CONFIG_FLASH_OFFSET);
}

void config_valid() {
    // valid config and set default value
    if (config.magic != CONFIG_MAGIC) {
        config.magic = CONFIG_MAGIC;
        printf("[Config] Config Magic Header is invalid\n");
    }
    if (config.size != sizeof(Config_body)) {
        config.size = sizeof(Config_body);
        printf("[Config] Config Body size is invalid\n");
    }
    auto body = &config.body;
    if (body->config_version != CONFIG_VERSION) {
        memset(body, 0xFF, sizeof(Config_body));
        body->config_version = CONFIG_VERSION;
        printf("[Config] Warning: Config may breaking change. Reset to default\n");
    }
    if (std::isnan(body->haptics_gain) || body->haptics_gain < 1.0f || body->haptics_gain > 2.0f) {
        body->haptics_gain = 1.0f;
        printf("[Config] Haptics Gain value is invalid\n");
    }
    if (body->speaker_volume < 0 || body->speaker_volume > 127) {
        body->speaker_volume = 100;
        printf("[Config] Speaker Volume is invalid\n");
    }
    if (body->headset_volume < 0 || body->headset_volume > 127) {
        body->headset_volume = 100;
        printf("[Config] Headset Volume is invalid\n");
    }
    if (body->speaker_gain < 0 || body->speaker_gain > 7) {
        body->speaker_gain = 2;
        printf("[Config] speaker_gain is invalid\n");
    }
    if (body->inactive_time < 0 || body->inactive_time > 60) {
        body->inactive_time = 30;
        printf("[Config] Inactive time is invalid\n");
    }
    if (body->disable_pico_led > 1) {
        body->disable_pico_led = 0;
        printf("[Config] disable_pico_led is invalid\n");
    }
    if (body->polling_rate_mode > 2) {
        // Vanilla defaults to stock 250 Hz: a real DS4 v2's HID endpoints
        // advertise bInterval 5, so faster modes make the dongle
        // distinguishable over USB. Higher rates stay available as an explicit
        // opt-in. The opinionated build defaults to real-time (1 kHz).
        body->polling_rate_mode = OPINIONATED ? 2 : 0;
        printf("[Config] polling_rate_mode is invalid\n");
    }
    if (body->audio_buffer_length < 16 || body->audio_buffer_length > 128) {
        body->audio_buffer_length = 64;
        printf("[Config] haptics_buffer_length is invalid\n");
    }
    if (body->controller_mode > 2) {
        body->controller_mode = 2;
        printf("[Config] controller_mode is invalid\n");
    }
    if (body->enable_usb_sn > 1) {
        body->enable_usb_sn = 0;
        printf("[Config] Warning: enable_usb_sn is invalid\n");
    }
    if (body->ps_shortcut_enabled > 1) {
        body->ps_shortcut_enabled = 0;
        printf("[Config] ps_shortcut_enabled is invalid\n");
    }
    if (body->mic_select > 3) {
        body->mic_select = 0;
        printf("[Config] mic_select is invalid\n");
    }
    if (body->speaker_select > 3) {
        body->speaker_select = 0;
        printf("[Config] speaker_select is invalid\n");
    }
    if (body->enable_wake > 1) {
        body->enable_wake = 0;
        printf("[Config] enable_wake is invalid\n");
    }
    if (body->trigger_reduce > 10) {
        body->trigger_reduce = 0;
        printf("[Config] trigger_reduce is invalid\n");
    }
    if (body->lock_volume > 1) {
        body->lock_volume = 0;
        printf("[Config] lock_volume is invalid\n");
    }
    // Default on (a config written by an older firmware reads 0xFF here and so
    // lands on the default): hide the USB audio device until a headset is in the
    // controller's jack, so the host auto-switches to it on plug like a real
    // headset. Costs a USB re-enumeration on every jack change (the host briefly
    // re-detects the controller); set to 0 to keep the audio device always on.
    if (body->audio_follow_jack > 1) {
        body->audio_follow_jack = 1;
        printf("[Config] audio_follow_jack is invalid\n");
    }
}

#if OPINIONATED
static uint32_t calc_wol_crc(const WolConfig &w) {
    const auto *p = reinterpret_cast<const uint8_t *>(&w);
    constexpr size_t skip = offsetof(WolConfig, crc32) + sizeof(w.crc32);
    return crc32(p + skip, sizeof(WolConfig) - skip);
}

// Blank and disabled: Wake-on-LAN is configured from config_web.html /
// config_tool.py.
static void wol_config_default() {
    memset(&wol_config, 0, sizeof(wol_config));
    wol_config.magic = WOL_CONFIG_MAGIC;
}

static void wol_config_load() {
    memcpy(&wol_config, reinterpret_cast<const uint8_t *>(flash_config()) + WOL_CONFIG_PAGE_OFFSET,
           sizeof(wol_config));
    if (wol_config.magic != WOL_CONFIG_MAGIC || wol_config.crc32 != calc_wol_crc(wol_config)) {
        printf("[Config] No valid Wake-on-LAN config in flash, using defaults\n");
        wol_config_default();
    }
    wol_config.enabled = wol_config.enabled ? 1 : 0;
    wol_config.ssid_len = std::min(wol_config.ssid_len, WOL_SSID_MAX);
    wol_config.password_len = std::min(wol_config.password_len, WOL_PASSWORD_MAX);
    wol_config.password[wol_config.password_len] = 0;
}

WolConfig& get_wol_config() {
    return wol_config;
}
#endif

void config_load() {
    memcpy(&config, flash_config(), sizeof(Config));

    config_valid();
#if OPINIONATED
    wol_config_load();
#endif
}

// Runs with core1 parked (flash_safe_execute) and core0 interrupts disabled, so
// neither core touches XIP flash while the sector is erased/programmed. Without
// the core1 park this races the audio core and corrupts audio (buzzing).
static void config_save_flash_op(void *param) {
    const uint8_t *page = static_cast<const uint8_t *>(param);
    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);
}

bool config_save() {
    config.crc32 = calc_config_crc(config);
    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &config, sizeof(Config));
#if OPINIONATED
    wol_config.magic = WOL_CONFIG_MAGIC;
    wol_config.crc32 = calc_wol_crc(wol_config);
    memcpy(page + WOL_CONFIG_PAGE_OFFSET, &wol_config, sizeof(WolConfig));
#endif

    const int rc = flash_safe_execute(config_save_flash_op, page, 1000);
    if (rc != PICO_OK) {
        printf("[Config] config_save flash_safe_execute failed: %d\n", rc);
        return false;
    }

    Config verify{};
    memcpy(&verify, flash_config(), sizeof(verify));
    const auto verify_crc32 = calc_config_crc(verify);
    if (verify_crc32 == config.crc32) {
        printf("[Config] Config write flash verify success\n");
        return true;
    }
    printf("[Config] Config write flash verify failed\n");
    return false;
}

Config_body& get_config() {
    return config.body;
}

void set_config(const uint8_t *new_config, const uint16_t len) {
    const auto copy_len = len < sizeof(Config_body) ? len : sizeof(Config_body);
    memcpy(&config.body, new_config, copy_len);
    config_valid();
    if (config.body.disable_pico_led) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    }else {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    }
    // DS4: no controller-side state to push on config change (trigger power
    // levels / mic select are DualSense concepts).
}

void set_config(const Config_body &new_config) {
    config.body = new_config;
    config_valid();
}
