//
// Created by awalol on 2026/3/4.
//

#ifndef DS4_BRIDGE_BT_H
#define DS4_BRIDGE_BT_H

#include <cstdint>
#include <vector>

enum CHANNEL_TYPE {
    INTERRUPT,
    CONTROL
};

typedef void (*bt_data_callback_t)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len);

int bt_init();
void bt_register_data_callback(bt_data_callback_t callback);
void bt_send_packet(uint8_t *data, uint16_t len);
void bt_send_control(uint8_t *data, uint16_t len);
void bt_power_off_controller();
bool bt_disconnect();
bool bt_is_connected();
void bt_set_scan_idle();
void bt_set_scan_active();
void bt_write(const uint8_t *data, uint16_t len);
void bt_get_signal_strength(int8_t *rssi);
std::vector<uint8_t> get_feature_data(uint8_t reportId,uint16_t len);
void init_feature();
void set_feature_data(uint8_t reportId, uint8_t* data,uint16_t len);
void bt_inquiring_led();
// BOOTSEL button actions, dispatched from button_functions.cpp.
void bt_bootsel_click_action();
void bt_bootsel_hold_action();
// DS4 output report 0x11 helpers (payload = USB-0x05-style, flags byte first).
void ds4_output(const uint8_t *payload, uint16_t payload_len);
void bt_get_controller_mac(uint8_t mac[6]);
void bt_get_local_mac(uint8_t mac[6]);
void ds4_set_led(uint8_t r, uint8_t g, uint8_t b);
void ds4_set_volume(uint8_t left, uint8_t right, uint8_t speaker);
void ds4_set_output_hdr2(uint8_t v); // debug: 0x11 output header byte 2 (mic-enable bits)
uint8_t ds4_get_output_hdr2();
void ds4_enable_mic(bool on); // start/stop headset-mic streaming (input report 0x13)

#endif //DS4_BRIDGE_BT_H
