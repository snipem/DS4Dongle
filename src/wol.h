//
// Wi-Fi Wake-on-LAN (opinionated build only, see wol.cpp).
//

#ifndef DS4_BRIDGE_WOL_H
#define DS4_BRIDGE_WOL_H

#include <cstdint>

#if OPINIONATED
// Main-loop task: watches for "controller connected but no USB data connection"
// and runs the Wi-Fi join + magic-packet loop. Keeps the WLAN radio fully down
// otherwise.
void wol_task();
// Config tool "test": join Wi-Fi and send a short magic-packet burst now,
// regardless of the USB state.
void wol_start_test();
// 0xFA GET_REPORT payload: config (never the password) + live status.
uint16_t wol_get_status(uint8_t *buffer, uint16_t reqlen);
// 0xF6 funcid 0x08 payload (after the funcid): [field][offset][len][data...]
void wol_set_field(const uint8_t *buffer, uint16_t len);
#else
static inline void wol_task() {}
#endif

#endif //DS4_BRIDGE_WOL_H
