//
// DS4 audio path: USB speaker PCM (32 kHz stereo S16) -> SBC (dual channel,
// 16 blocks, 8 subbands, loudness, bitpool 25) -> HID output report 0x17
// (4 x 112-byte SBC frames per report, one report per 16 ms of audio).
//

#ifndef DS4_BRIDGE_AUDIO_H
#define DS4_BRIDGE_AUDIO_H

#include <cstdint>

void audio_init();
void audio_loop();
void core1_entry();
void set_headset(bool state);
bool audio_headset_plugged();
void set_mic_active(bool active);
// The USB bus is being torn down (deliberate re-enumeration): forget the
// host's alternate-setting state, so a mic session cannot keep streaming from
// the controller into an audio device that no longer exists.
void audio_usb_itf_reset();
bool audio_mic_active();
// Feed a BT input report 0x13 (state + mic SBC frame) into the mic pipeline.
void audio_mic_bt_data(const uint8_t *data, uint16_t len);

#endif //DS4_BRIDGE_AUDIO_H
