//
// Created by awalol on 2026/3/4.
//

#ifndef DS5_BRIDGE_USB_H
#define DS5_BRIDGE_USB_H

extern uint8_t mute[2]; // 0: SPEAKER(0x02) 1: MIC(0x05)
extern float volume[2]; // 0: SPEAKER(0x02) 1: MIC(0x05)

// Re-seed the speaker volume from config on the next GET_CUR (fresh enumeration).
void usb_audio_reset_volume_sync();

// --- Jack-follow audio exposure (config: audio_follow_jack) ---------------
// Whether the USB audio function is part of the CURRENT enumeration. Latched
// at connect time so every descriptor request within one enumeration agrees.
bool usb_audio_exposed();

// Sample jack + config state into that latch. Call right before tud_connect().
void usb_audio_latch_exposure();

// Main-loop task: re-enumerates the dongle when the headset jack state changes,
// so the audio device appears/disappears with the headset. No-op when
// audio_follow_jack is off.
void usb_audio_exposure_task();

#endif //DS5_BRIDGE_USB_H