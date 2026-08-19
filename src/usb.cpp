//
// Created by awalol on 2026/3/4.
//

#include <algorithm>

#include "audio.h"
#include "bt.h"
#include "tusb.h"
#include "bsp/board_api.h"
#include "pico/time.h"
#include "config.h"
#include "usb.h"
#include "utils.h"
#include "wake.h"

uint8_t mute[2] = {}; // 0: SPEAKER(0x02) 1: MIC(0x05)
// 0: SPEAKER(0x02) 1: MIC(0x05). Mic seed kept within the advertised
// -23.25..+24 dB range so GET_CUR is never out of bounds before the host
// sets it. The mic gain to the controller is fixed regardless, so this only
// affects the value echoed on the USB control.
float volume[2] = {0.0f, 0.0f};

// Whether the host has set/read the speaker volume yet. Until then, GET_CUR
// seeds volume[0] from the config default. After that, GET_CUR must return
// exactly what the host last SET: the Linux kernel probes the control with a
// SET_CUR/GET_CUR round-trip (check_sticky_volume_control in
// sound/usb/mixer.c) and, when the readback doesn't match, flags the mixer as
// "sticky", drops the volume control, and pins it at max ("sticky mixer
// values ... disabling" in dmesg). The host then falls back to software
// volume and this firmware's dB->byte mapping is never exercised.
static bool spk_volume_synced = false;

void usb_audio_reset_volume_sync() {
    spk_volume_synced = false;
}

#define UAC1_ENTITY_SPK_FEATURE_UNIT    0x02
#define UAC1_ENTITY_MIC_FEATURE_UNIT    0x05

// Measured via a headphone-jack -> line-in RMS sweep (2026-07-14):
// the DS4 headphone amp is linear in dB at 1.0 dB per byte step (0.97 measured)
// from byte ~20 up to byte 80, and saturates hard at byte 80 -- bytes 80..255
// are identical to within 0.01 dB. So the byte value IS a dB attenuation
// relative to max: byte = 80 + dB. The usable amp range is -60..0 dB.
#define DS4_VOLUME_SAT 80.0f

// The feature unit advertises the real DS4 v2's speaker range (-73..-1 dB) for
// USB indistinguishability, which is wider than the amp's usable -60..0 dB. The
// host-set dB (stored raw in volume[0] so GET_CUR echoes it exactly) is
// linearly rescaled from the advertised span onto the usable span before the
// byte mapping, so the full slider travel still spans the amp's full range.
#define SPK_ADV_MIN_DB (-73.0f)
#define SPK_ADV_MAX_DB (-1.0f)
#define SPK_USABLE_MIN_DB (-60.0f)
#define SPK_USABLE_MAX_DB (0.0f)

// Push the current USB mute/volume state to the controller as a DS4 0x11
// volume report. volume[0] is the host-set dB in the advertised range.
static void ds4_push_volume() {
    if (get_config().lock_volume) return;
    uint8_t level = 0;
    if (!mute[0]) {
        float adv = volume[0];
        if (adv < SPK_ADV_MIN_DB) adv = SPK_ADV_MIN_DB;
        if (adv > SPK_ADV_MAX_DB) adv = SPK_ADV_MAX_DB;
        const float frac = (adv - SPK_ADV_MIN_DB) / (SPK_ADV_MAX_DB - SPK_ADV_MIN_DB);
        const float usable = SPK_USABLE_MIN_DB + frac * (SPK_USABLE_MAX_DB - SPK_USABLE_MIN_DB);
        float v = DS4_VOLUME_SAT + usable;
        if (v < 0.0f) v = 0.0f;
        if (v > DS4_VOLUME_SAT) v = DS4_VOLUME_SAT;
        level = static_cast<uint8_t>(v + 0.5f);
    }
    ds4_set_volume(level, level, level);
}

/*int main() {
    board_init();

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    board_init_after_tusb();

    while (1) {
        tud_task();
    }
}*/

//--------------------------------------------------------------------+
// Audio Callback Functions
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// UAC1 Helper Functions
//--------------------------------------------------------------------+

static bool audio10_set_req_entity(tusb_control_request_t const *p_request, uint8_t *pBuff) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t index = entityID == UAC1_ENTITY_SPK_FEATURE_UNIT ? 0 : 1;

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT || entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO10_FU_CTRL_MUTE:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_SET_CUR: {
                        // Only 1st form is supported
                        TU_VERIFY(p_request->wLength == 1);

                        mute[index] = pBuff[0];
                        ds4_push_volume();

                        TU_LOG2("    Set Mute: %d of entity: %u\r\n", mute[index], entityID);
                        return true;
                    }

                    default:
                        return false; // not supported
                }

            case AUDIO10_FU_CTRL_VOLUME:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_SET_CUR:
                        // Only 1st form is supported
                        TU_VERIFY(p_request->wLength == 2);

                        volume[index] = static_cast<float>(*reinterpret_cast<int16_t const *>(pBuff)) / 256;
                        if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                            spk_volume_synced = true;
                            ds4_push_volume();
                        }
                        // Mic volume: DS4 mic input is not implemented yet.

                        TU_LOG2("    Set Volume: %d dB of entity: %u\r\n", volume[index], entityID);
                        return true;

                    default:
                        return false; // not supported
                }

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }

    return false;
}

static bool audio10_get_req_entity(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    uint8_t index = entityID == UAC1_ENTITY_SPK_FEATURE_UNIT ? 0 : 1;

    // If request is for our speaker feature unit
    if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT || entityID == UAC1_ENTITY_MIC_FEATURE_UNIT) {
        switch (ctrlSel) {
            case AUDIO10_FU_CTRL_MUTE:
                // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
                // There does not exist a range parameter block for mute
                TU_LOG2("    Get Mute of entity: %u\r\n", entityID);
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[index], 1);

            case AUDIO10_FU_CTRL_VOLUME:
                switch (p_request->bRequest) {
                    case AUDIO10_CS_REQ_GET_CUR:
                        TU_LOG2("    Get Volume of entity: %u\r\n", entityID); {
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT && !spk_volume_synced) {
                                // Seed from config (100 = full), mapped into the advertised
                                // -73..-1 dB range so GET_CUR sits in-range from the start.
                                const float frac = std::min(static_cast<int>(get_config().speaker_volume), 100) / 100.0f;
                                volume[index] = SPK_ADV_MIN_DB + frac * (SPK_ADV_MAX_DB - SPK_ADV_MIN_DB);
                                spk_volume_synced = true;
                            }
                            int16_t vol = volume[index] * 256; // convert to 1/256 dB units
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &vol, sizeof(vol));
                        }

                    // Advertised ranges are byte-identical to a real DS4 v2
                    // (captured 2026-07-14): SPK min -73 / max -1 / res 1 dB,
                    // MIC min -23.25 / max +24 / res 0.75 dB. The advertised
                    // range is decoupled from the internal amp mapping -- the
                    // host-set dB is rescaled onto the measured -60..0 dB amp
                    // curve in ds4_push_volume, so the slider still tracks
                    // actual loudness (see SPK_ADV_* below).
                    case AUDIO10_CS_REQ_GET_MIN:
                        TU_LOG2("    Get Volume min of entity: %u\r\n", entityID); {
                            uint8_t min[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                min[0] = 0x00; min[1] = 0xb7; // -73 dB
                            }else {
                                min[0] = 0xc0; min[1] = 0xe8; // -23.25 dB
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                        }

                    case AUDIO10_CS_REQ_GET_MAX:
                        TU_LOG2("    Get Volume max of entity: %u\r\n", entityID); {
                            uint8_t max[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                max[0] = 0x00; max[1] = 0xff; // -1 dB
                            }else {
                                max[0] = 0x00; max[1] = 0x18; // +24 dB
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                        }

                    case AUDIO10_CS_REQ_GET_RES:
                        TU_LOG2("    Get Volume res of entity: %u\r\n", entityID); {
                            uint8_t res[2];
                            if (entityID == UAC1_ENTITY_SPK_FEATURE_UNIT) {
                                res[0] = 0x00; res[1] = 0x01; // 1 dB
                            }else {
                                res[0] = 0xc0; res[1] = 0x00; // 0.75 dB
                            }
                            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                        }
                    // Unknown/Unsupported control
                    default:
                        TU_BREAKPOINT();
                        return false;
                }
                break;

            // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }

    return false;
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;

    return audio10_get_req_entity(rhport, p_request);
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void) rhport;

    return audio10_set_req_entity(p_request, buf);
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {
    (void) instance;
    (void) len;
}

#ifndef ENABLE_WAKE_HID

void tud_suspend_cb(bool remote_wakeup_en) {
    printf("[USB PM] invoke tud_suspend_cb\n");
    if (!get_config().enable_wake) return;   // wake off: leave the controller's BT alone on a USB suspend (see wake.cpp)
    bt_power_off_controller();
}

#endif

//--------------------------------------------------------------------+
// Jack-follow audio exposure
//--------------------------------------------------------------------+
// A PS4 only offers a headset audio device while something is plugged into the
// DS4's 3.5 mm jack, and the host switches its default output the moment that
// device appears. The dongle mirrors that when audio_follow_jack is on (the
// default): the whole USB audio function is dropped from the configuration
// descriptor while the jack is empty, and a plug/unplug re-enumerates the
// device so the host sees the audio endpoint come and go like a real headset.
//
// The decision has to be latched per enumeration -- a descriptor request must
// never see a different answer than the one the host is already enumerating
// against -- so the live jack state only takes effect at (re)connect time.

namespace {

constexpr uint32_t JACK_DEBOUNCE_MS = 300; // ride out jack contact chatter before re-enumerating
constexpr uint32_t REENUM_GAP_MS = 150;    // bus-down window the host reads as an unplug (as FUNC_RECONNECT)

bool audio_exposed = true;    // latched for the current enumeration
uint32_t jack_change_ms = 0;  // first sighting of a not-yet-latched state, 0 = none pending
uint32_t reconnect_at_ms = 0; // when to bring the bus back up after a hide/show, 0 = idle

// Should the audio function be visible, given the jack and the config right now?
bool audio_exposure_target() {
    const Config_body &cfg = get_config();
    if (!cfg.audio_follow_jack) return true;
    if (cfg.speaker_select == 1) return true; // pinned to the built-in speaker: the jack is irrelevant
    return audio_headset_plugged();
}

} // namespace

bool usb_audio_exposed() {
    return audio_exposed;
}

void usb_audio_latch_exposure() {
    audio_exposed = audio_exposure_target();
    jack_change_ms = 0;
}

void usb_audio_exposure_task() {
    const uint32_t now = to_ms_since_boot(get_absolute_time());

    // Second half of a re-enumeration: bring the bus back up with the new latch.
    if (reconnect_at_ms) {
        if (static_cast<int32_t>(now - reconnect_at_ms) < 0) return;
        reconnect_at_ms = 0;
        usb_audio_latch_exposure();
        if (!tud_suspended()) tud_connect();
        printf("[AUDIO] USB audio device %s\n", audio_exposed ? "exposed" : "hidden");
        return;
    }

    if (!get_config().audio_follow_jack) return;
    // Only while we are actually on the bus and the host is awake: a hidden
    // dongle (no controller) has nothing to re-enumerate, and a reconnect
    // during suspend would wake a sleeping host.
    if (!tud_connected() || tud_suspended()) return;

    const bool target = audio_exposure_target();
    if (target == audio_exposed) {
        jack_change_ms = 0;
        return;
    }
    if (!jack_change_ms) {
        jack_change_ms = now ? now : 1;
        return;
    }
    if (now - jack_change_ms < JACK_DEBOUNCE_MS) return;
    jack_change_ms = 0;

    printf("[AUDIO] Headset jack %s -- re-enumerating to %s the USB audio device\n",
           target ? "plugged" : "unplugged", target ? "show" : "hide");
    wake_note_usb_reconnect(); // this disconnect is deliberate, not a host sleep
    usb_audio_reset_volume_sync();
    audio_usb_itf_reset();
    tud_disconnect();
    reconnect_at_ms = now + REENUM_GAP_MS;
    if (!reconnect_at_ms) reconnect_at_ms = 1;
}
