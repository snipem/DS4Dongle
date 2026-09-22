//
// Wi-Fi Wake-on-LAN (opinionated build only).
//
// When the controller connects (PS press) while the dongle has no USB data
// connection -- the PC is off, hibernating, or asleep without USB remote wakeup
// -- the dongle joins the configured Wi-Fi network and broadcasts Wake-on-LAN
// magic packets for the configured PC until the PC enumerates the dongle.
//
// "Data connection" is what the dongle can see on its USB port: the host has
// enumerated it (tud_mounted) and the bus is not suspended. A PC that is off
// but still powering the port (USB standby power) never enumerates; a sleeping
// PC suspends the bus. Both read as "no data connection". Note the dongle needs
// that standby power to run at all -- a port that goes dark on shutdown gives
// it no chance to send anything.
//
// Only a *fresh* controller connection arms it. A controller that stays
// connected while the PC shuts down (the bus drops but no new BT connection
// happens) must not wake the PC straight back up.
//
// Wi-Fi is off whenever there is a data connection: the station leaves the
// network the moment the PC enumerates the dongle. An unassociated station
// neither scans nor tracks beacons, so BT gets the shared radio's airtime while
// the controller is in use. (The WLAN core itself is brought up on the first
// session and left up; cycling it down and up again wedges the chip.)
//
// No lwIP: the magic packet is sent as raw Ethernet frames through
// cyw43_send_ethernet (this file overrides the SDK's panicking CYW43_LWIP=0
// callbacks with no-ops), so no DHCP or IP address is needed. Two frames per
// burst: EtherType 0x0842 (the dedicated WoL ethertype) and an IPv4/UDP
// broadcast to port 9 from 0.0.0.0, since some bridges only forward IP. A NIC
// armed for magic packets matches the pattern anywhere in the frame.
//

#include "wol.h"

#if OPINIONATED

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "bt.h"
#include "config.h"
#include "tusb.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

namespace {

constexpr uint32_t WOL_JOIN_TIMEOUT_MS = 20000;
constexpr uint32_t WOL_BACKOFF_MS = 3000;      // after a failed join
constexpr uint32_t WOL_FAST_RETRY_MS = 1000;   // after a dropped link / the first auth failures:
                                               // the chip needs a moment after a drop
constexpr uint8_t WOL_FAST_AUTH_RETRIES = 3;
constexpr uint32_t WOL_SEND_INTERVAL_MS = 1000;
constexpr uint32_t WOL_SESSION_MS = 5 * 60 * 1000; // give up if the PC never shows up
constexpr uint32_t WOL_TEST_SESSION_MS = 30000;
constexpr uint8_t WOL_TEST_BURSTS = 5;

enum wol_state_t : uint8_t {
    WOL_IDLE,     // not associated, waiting for a fresh controller connection
    WOL_JOINING,
    WOL_SENDING,
    WOL_BACKOFF,
};

enum wol_result_t : uint8_t {
    WOL_RESULT_NONE,
    WOL_RESULT_HOST_UP,  // data connection appeared -> stopped
    WOL_RESULT_TIMEOUT,  // session ran out without a data connection
    WOL_RESULT_TEST_DONE,
    WOL_RESULT_DISABLED, // disabled / config cleared mid-session
    WOL_RESULT_WEDGED,   // Wi-Fi chip stopped responding -> dongle reboots
};

wol_state_t state = WOL_IDLE;
uint32_t state_at_ms = 0;
uint32_t session_at_ms = 0;
uint32_t next_send_ms = 0;
bool test_mode = false;
bool test_requested = false;
uint8_t test_bursts_left = 0;
bool prev_bt = false;
bool wlan_on = false;         // cyw43_wifi_on done -- once per boot, never undone
bool wedged = false;          // WLAN firmware stopped answering: reboot when safe
bool wedged_in_test = false;  // ...found by a config-tool test: reboot right away
uint8_t own_mac[6]{};         // our STA MAC
uint8_t auth_failures = 0;    // CYW43_LINK_BADAUTH joins this session
uint32_t backoff_ms = WOL_BACKOFF_MS;
int8_t last_link = CYW43_LINK_DOWN;
wol_result_t last_result = WOL_RESULT_NONE;
uint16_t packets_sent = 0;
uint8_t join_attempts = 0;

uint32_t now_ms() {
    return to_ms_since_boot(get_absolute_time());
}

void enter(wol_state_t s) {
    state = s;
    state_at_ms = now_ms();
}

bool configured() {
    const WolConfig &w = get_wol_config();
    const uint8_t zero[6]{};
    return w.ssid_len > 0 && memcmp(w.mac, zero, sizeof(zero)) != 0;
}

bool data_connection() {
    return tud_mounted() && !tud_suspended();
}

// Every cyw43 ioctl can block up to CYW43_IOCTL_TIMEOUT_US (500 ms), and after
// a dropped link the chip was seen running 8 back-to-back ioctl timeouts (~4 s)
// inside one main-loop pass. With the regular 1 s watchdog that resets the
// dongle and drops the controller, so the watchdog is stretched for the length
// of a session. (Debug builds run without a watchdog.)
constexpr uint32_t WATCHDOG_NORMAL_MS = 1000; // main.cpp watchdog_enable()
constexpr uint32_t WATCHDOG_SESSION_MS = 8000;

void watchdog_set(uint32_t ms) {
#if !ENABLE_SERIAL
    watchdog_enable(ms, true);
#else
    (void) ms;
#endif
}

// Bring the WLAN core up on the first session and leave it up. Taking it back
// down (WLC_DOWN) and re-running cyw43_wifi_on is not a path the driver
// supports: every session after such a cycle dropped its link within seconds
// and then wedged the chip (all ioctls timing out, WLC_UP included), while
// the first session after boot was always clean.
void wlan_ensure_on() {
    if (wlan_on) return;
    const uint64_t t0 = time_us_64();
    watchdog_update();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    watchdog_update();
    cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, own_mac);
    watchdog_update();
    wlan_on = true;
    printf("[WOL] WLAN on (%u ms)\n", static_cast<unsigned>((time_us_64() - t0) / 1000));
}

// Disassociate. A station that is not associated neither scans nor tracks
// beacons, so the WLAN side of the radio sits idle and Bluetooth has the
// airtime to itself -- the state the dongle is in whenever the PC is up.
void wlan_leave() {
    if (cyw43_state.wifi_join_state) {
        watchdog_update();
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        watchdog_update();
        printf("[WOL] left Wi-Fi -- radio idle, Bluetooth only\n");
    }
    last_link = CYW43_LINK_DOWN;
}

// The CYW43's WLAN firmware occasionally stops answering altogether after a
// number of join/leave cycles (every ioctl times out from then on, and the
// Pico LED -- driven through the same ioctl path -- stalls the main loop for
// 500 ms per write). Nothing short of re-initialising the chip recovers it,
// and a fresh boot has always given a clean first session. So probe the chip
// with a harmless GET after every session and on failures, and reboot the
// dongle once it is safe to (see wol_task).
bool wlan_responsive() {
    if (!wlan_on) return true;
    uint8_t buf[4]{};
    watchdog_update();
    const int rc = cyw43_ioctl(&cyw43_state, CYW43_IOCTL_GET_ANTDIV, sizeof(buf), buf, CYW43_ITF_STA);
    watchdog_update();
    if (rc != 0) printf("[WOL] Wi-Fi chip not responding (rc %d)\n", rc);
    return rc == 0;
}

void finish(wol_result_t result) {
    wlan_leave();
    if (result == WOL_RESULT_WEDGED || !wlan_responsive()) {
        wedged = true;
        wedged_in_test = test_mode;
        result = WOL_RESULT_WEDGED;
    }
    watchdog_set(WATCHDOG_NORMAL_MS);
    test_mode = false;
    last_result = result;
    enter(WOL_IDLE);
    printf("[WOL] session end (result %u, %u packets)\n", result, packets_sent);
}

void start_join() {
    const WolConfig &w = get_wol_config();
    wlan_ensure_on();
    if (join_attempts < 255) join_attempts++;
    const uint32_t auth = w.password_len ? CYW43_AUTH_WPA2_MIXED_PSK : CYW43_AUTH_OPEN;
    watchdog_update();
    const int rc = cyw43_wifi_join(&cyw43_state, w.ssid_len, reinterpret_cast<const uint8_t *>(w.ssid),
                                   w.password_len, reinterpret_cast<const uint8_t *>(w.password),
                                   auth, nullptr, CYW43_CHANNEL_NONE);
    printf("[WOL] joining '%.*s' (attempt %u) -> %d\n", w.ssid_len, w.ssid, join_attempts, rc);
    if (rc != 0) backoff_ms = WOL_BACKOFF_MS;
    enter(rc == 0 ? WOL_JOINING : WOL_BACKOFF);
}

void start_session(bool test) {
    test_mode = test;
    test_bursts_left = WOL_TEST_BURSTS;
    packets_sent = 0;
    join_attempts = 0;
    auth_failures = 0;
    last_result = WOL_RESULT_NONE;
    session_at_ms = now_ms();
    watchdog_set(WATCHDOG_SESSION_MS);
    start_join();
}

uint16_t ip_checksum(const uint8_t *p, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i + 1 < len; i += 2) sum += (p[i] << 8) | p[i + 1];
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

void send_magic_packets() {
    const WolConfig &w = get_wol_config();
    uint8_t magic[102];
    memset(magic, 0xFF, 6);
    for (int i = 0; i < 16; i++) memcpy(magic + 6 + i * 6, w.mac, 6);

    uint8_t frame[14 + 20 + 8 + sizeof(magic)];
    memset(frame, 0xFF, 6);          // broadcast destination
    memcpy(frame + 6, own_mac, 6);

    // 1) EtherType 0x0842: raw Wake-on-LAN.
    frame[12] = 0x08; frame[13] = 0x42;
    memcpy(frame + 14, magic, sizeof(magic));
    const int rc1 = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, 14 + sizeof(magic), frame, false);

    // 2) IPv4 UDP 0.0.0.0:9 -> 255.255.255.255:9 (UDP checksum 0 = none).
    frame[12] = 0x08; frame[13] = 0x00;
    uint8_t *ip = frame + 14;
    const uint16_t ip_len = 20 + 8 + sizeof(magic);
    memset(ip, 0, 20);
    ip[0] = 0x45;                    // IPv4, 20-byte header
    ip[2] = ip_len >> 8; ip[3] = ip_len & 0xFF;
    ip[8] = 64;                      // TTL
    ip[9] = 17;                      // UDP
    memset(ip + 16, 0xFF, 4);        // dst 255.255.255.255 (src stays 0.0.0.0)
    const uint16_t csum = ip_checksum(ip, 20);
    ip[10] = csum >> 8; ip[11] = csum & 0xFF;
    uint8_t *udp = ip + 20;
    const uint16_t udp_len = 8 + sizeof(magic);
    udp[0] = 0; udp[1] = 9;          // src port 9
    udp[2] = 0; udp[3] = 9;          // dst port 9 (discard)
    udp[4] = udp_len >> 8; udp[5] = udp_len & 0xFF;
    udp[6] = 0; udp[7] = 0;
    memcpy(udp + 8, magic, sizeof(magic));
    const int rc2 = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, sizeof(frame), frame, false);

    if (packets_sent < 0xFFFF) packets_sent++;
    printf("[WOL] magic packet #%u -> %02X:%02X:%02X:%02X:%02X:%02X (rc %d/%d)\n", packets_sent,
           w.mac[0], w.mac[1], w.mac[2], w.mac[3], w.mac[4], w.mac[5], rc1, rc2);
}

} // namespace

// cyw43_ll_send_ethernet references lwIP's pbuf_copy_partial for its is_pbuf
// path. This build has no lwIP and only ever passes plain buffers
// (is_pbuf = false), so the pbuf path is dead -- satisfy the linker.
extern "C" uint16_t pbuf_copy_partial(const void *, void *, uint16_t, uint16_t) {
    return 0;
}

// Without lwIP the SDK's weak defaults for these callbacks panic ("cyw43 has no
// ethernet interface") -- the first frame the station receives (any broadcast
// on the network) would crash the dongle. We only send, so every incoming
// frame is dropped and link changes are read via cyw43_wifi_link_status().
void cyw43_cb_process_ethernet(void *, int, size_t, const uint8_t *) {}
void cyw43_cb_tcpip_set_link_up(cyw43_t *, int) {}
void cyw43_cb_tcpip_set_link_down(cyw43_t *, int) {}
int cyw43_tcpip_link_status(cyw43_t *self, int itf) {
    return cyw43_wifi_link_status(self, itf);
}

void wol_start_test() {
    test_requested = true;
}

void wol_task() {
    const uint32_t now = now_ms();
    const bool data = data_connection();
    const bool bt = bt_is_connected();
    const bool bt_edge = bt && !prev_bt;
    prev_bt = bt;
    const bool usable = get_wol_config().enabled && configured();

    // Wedged Wi-Fi chip: reboot once nobody is using the dongle -- no USB data
    // connection means the PC is off or asleep, so dropping the controller for a
    // moment costs nothing (it reconnects, and the fresh boot's first session is
    // clean). A wedge found by a config-tool test reboots right away.
    if (wedged && state == WOL_IDLE && (!data || wedged_in_test)) {
        printf("[WOL] rebooting to recover the Wi-Fi chip\n");
        watchdog_reboot(0, 0, 10);
        while (true) tight_loop_contents();
    }

    if (test_requested) {
        test_requested = false;
        if (configured()) {
            wlan_leave(); // restart cleanly if a session was running
            printf("[WOL] test requested\n");
            start_session(true);
        } else {
            last_result = WOL_RESULT_DISABLED;
        }
        return;
    }

    if (state == WOL_IDLE) {
        if (bt_edge && !data && usable) {
            // No grace period, for an instant feel: start joining right away. If
            // the PC is actually on, it enumerates the dongle (well under a second)
            // long before the Wi-Fi join completes, and the data check below
            // drops Wi-Fi again before a single packet goes out.
            printf("[WOL] controller connected without a USB data connection -- waking PC\n");
            start_session(false);
        }
        return;
    }

    if (!test_mode) {
        // The host is up: drop Wi-Fi at once so Bluetooth has the radio to itself.
        if (data) { finish(WOL_RESULT_HOST_UP); return; }
        if (!usable) { finish(WOL_RESULT_DISABLED); return; }
    }

    const uint32_t limit = test_mode ? WOL_TEST_SESSION_MS : WOL_SESSION_MS;
    if (now - session_at_ms >= limit) {
        finish(test_mode ? WOL_RESULT_TEST_DONE : WOL_RESULT_TIMEOUT);
        return;
    }

    switch (state) {
        case WOL_IDLE:
            return;

        case WOL_JOINING: {
            const int link = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
            last_link = static_cast<int8_t>(link);
            if (link == CYW43_LINK_JOIN) {
                printf("[WOL] Wi-Fi joined\n");
                next_send_ms = now;
                enter(WOL_SENDING);
            } else if (link < 0 || now - state_at_ms >= WOL_JOIN_TIMEOUT_MS) {
                // BADAUTH also covers a WPA handshake that timed out, which
                // happens with a correct password while BT is busy with the
                // controller -- retry fast a few times before backing off.
                if (link == CYW43_LINK_BADAUTH && auth_failures < 255) auth_failures++;
                backoff_ms = (link == CYW43_LINK_BADAUTH && auth_failures <= WOL_FAST_AUTH_RETRIES)
                                 ? WOL_FAST_RETRY_MS : WOL_BACKOFF_MS;
                printf("[WOL] join failed (link %d), retry in %u ms\n", link,
                       static_cast<unsigned>(backoff_ms));
                wlan_leave();
                enter(WOL_BACKOFF);
            }
            return;
        }

        case WOL_SENDING: {
            const int link = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
            last_link = static_cast<int8_t>(link);
            if (link != CYW43_LINK_JOIN) {
                printf("[WOL] Wi-Fi link lost (%d)\n", link);
                backoff_ms = WOL_FAST_RETRY_MS;
                wlan_leave();
                enter(WOL_BACKOFF);
                return;
            }
            if (static_cast<int32_t>(now - next_send_ms) < 0) return;
            send_magic_packets();
            next_send_ms = now + WOL_SEND_INTERVAL_MS;
            if (test_mode && --test_bursts_left == 0) finish(WOL_RESULT_TEST_DONE);
            return;
        }

        case WOL_BACKOFF:
            if (now - state_at_ms < backoff_ms) return;
            // A failed join or dropped link is how a wedge first shows up.
            if (!wlan_responsive()) { finish(WOL_RESULT_WEDGED); return; }
            start_join();
            return;
    }
}

// 0xFA layout (48 bytes):
//   [0] layout version (1)      [1] enabled
//   [2..7] target MAC           [8] ssid_len        [9..40] ssid
//   [41] password_len (the password itself is never readable)
//   [42] state (wol_state_t)    [43] last Wi-Fi link status (int8, CYW43_LINK_*)
//   [44] last result (wol_result_t)
//   [45..46] magic packets sent this session (LE)   [47] join attempts
uint16_t wol_get_status(uint8_t *buffer, uint16_t reqlen) {
    constexpr uint16_t LEN = 48;
    if (reqlen < LEN) return 0;
    const WolConfig &w = get_wol_config();
    memset(buffer, 0, LEN);
    buffer[0] = 1;
    buffer[1] = w.enabled;
    memcpy(buffer + 2, w.mac, 6);
    buffer[8] = w.ssid_len;
    memcpy(buffer + 9, w.ssid, w.ssid_len);
    buffer[41] = w.password_len;
    buffer[42] = state;
    buffer[43] = static_cast<uint8_t>(last_link);
    buffer[44] = last_result;
    buffer[45] = packets_sent & 0xFF;
    buffer[46] = packets_sent >> 8;
    buffer[47] = join_attempts;
    return LEN;
}

// [field][offset][len][data...]
//   field 0: enabled (data[0])
//   field 1: target MAC (6 bytes, offset ignored)
//   field 2: SSID chunk     } offset 0 starts a new value; the value's length
//   field 3: password chunk } becomes offset + len (so len 0 at offset 0 clears)
// Changes are RAM-only until the regular funcid 0x02 save.
void wol_set_field(const uint8_t *buffer, uint16_t len) {
    if (len < 3) return;
    const uint8_t field = buffer[0], offset = buffer[1];
    uint8_t n = buffer[2];
    const uint8_t *data = buffer + 3;
    if (n > len - 3) n = len - 3;
    WolConfig &w = get_wol_config();

    auto set_chunk = [&](char *dst, uint8_t &dst_len, uint8_t max) {
        if (offset > max) return;
        n = std::min<uint8_t>(n, max - offset);
        if (offset == 0) memset(dst, 0, max);
        memcpy(dst + offset, data, n);
        dst_len = offset + n;
    };

    switch (field) {
        case 0:
            if (n >= 1) w.enabled = data[0] ? 1 : 0;
            break;
        case 1:
            if (n >= 6) memcpy(w.mac, data, 6);
            break;
        case 2:
            set_chunk(w.ssid, w.ssid_len, WOL_SSID_MAX);
            break;
        case 3:
            set_chunk(w.password, w.password_len, WOL_PASSWORD_MAX);
            w.password[w.password_len] = 0;
            break;
        default:
            return;
    }
    printf("[WOL] config field %u updated\n", field);
}

#endif // OPINIONATED
