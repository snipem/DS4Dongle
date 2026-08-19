//
// DS4 audio path.
//
// USB OUT (32 kHz stereo S16, matching the DS4's native BT audio rate, so no
// resampling is needed) -> 128-frame PCM blocks to core1 -> bluedroid SBC
// encoder (dual channel, 16 blocks, 8 subbands, loudness, bitpool 25 =
// 112-byte frames, header 9c 75 19) -> core0 packs 4 frames into HID output
// report 0x17 and queues it to the interrupt channel. One report carries
// 512 samples = 16 ms, and the USB isochronous stream is the pacing clock.
//
// Protocol reference: https://www.psdevwiki.com/ps4/DS4-BT and the working
// host-side PoC in ../../ds4_audio.py.
//

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "audio.h"
#include "bt.h"
#if ENABLE_DEBUG
#include "debug.h"
#endif
#include "tusb.h"
#include <cstdio>
#include <cstring>
#include "utils.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include "config.h"

extern bool spk_active; // host has the speaker OUT interface open (main.cpp)

extern "C" {
#include "classic/btstack_sbc_bluedroid.h"
}

#define SBC_FRAME_SAMPLES 128 // 16 blocks * 8 subbands, per channel
#define SBC_FRAME_LEN     112 // dual channel, bitpool 25
#define FRAMES_PER_REPORT 4
#define REPORT_SIZE       462 // 0x17 report incl. trailing CRC-32
// Audio header byte (report offset 5): target of the SBC payload.
#define AUDIO_TARGET_SPEAKER 0x02
#define AUDIO_TARGET_HEADSET 0x24

// Headset mic (BT input report 0x13, reverse-engineered 2026-07-14): one SBC
// frame per report at offset 82 (after the 0xA1 transport byte, report id,
// 2 header bytes, 71-byte state block and a 7-byte audio header), zero-padded
// to the fixed report size. SBC header 9c 31 1d = 16 kHz mono, 16 blocks,
// 8 subbands, loudness, bitpool 29 -> 66-byte frames, 128 samples = 8 ms.
#define MIC_SBC_FRAME_LEN 66
#define MIC_SBC_SCAN_FROM 75 // audio header start; frame is found by 0x9C syncword
#define MIC_PCM_SAMPLES   128 // mono 16 kHz samples per frame

static bool plug_headset = false;
static bool mic_active = false; // host has opened the mic IN interface (alt != 0)

static volatile bool core1_ready = false;
alignas(8) static uint32_t audio_core1_stack[2048];

struct pcm_block {
    int16_t data[SBC_FRAME_SAMPLES * 2]; // interleaved stereo
};

struct sbc_frame {
    uint8_t data[SBC_FRAME_LEN];
};

struct mic_sbc_frame {
    uint8_t data[MIC_SBC_FRAME_LEN];
};

struct mic_pcm_block {
    int16_t data[MIC_PCM_SAMPLES]; // mono 16 kHz
};

queue_t audio_fifo; // raw pcm blocks core0 -> core1
queue_t sbc_fifo;   // encoded sbc frames core1 -> core0
static queue_t mic_sbc_fifo; // mic sbc frames core0 (BT) -> core1 (decode)
static queue_t mic_pcm_fifo; // decoded mic pcm core1 -> core0 (USB IN ISR)

// Mic USB delivery state, owned by the EP IN reload ISR (tud_audio_tx_done_isr).
static mic_pcm_block mic_cur;
static uint16_t mic_cur_pos = 0; // samples consumed from mic_cur
static bool mic_cur_valid = false;
static volatile bool mic_streaming = false; // jitter buffer filled, consuming

void set_headset(bool state) {
    plug_headset = state;
}

bool audio_headset_plugged() {
    return plug_headset;
}

// Called from tud_audio_set_itf_cb when the host opens/closes the mic IN
// interface. Tells the controller to start/stop streaming headset-mic audio
// (input report 0x13).
static volatile bool mic_decoder_reset_pending = false;

void set_mic_active(bool active) {
    if (active && !mic_active) {
        // Fresh session: drop stale frames and have core1 reinit the decoder
        // (a garbage frame can leave the OI decoder without sync forever).
        while (queue_try_remove(&mic_sbc_fifo, NULL)) {}
        while (queue_try_remove(&mic_pcm_fifo, NULL)) {}
        mic_streaming = false;
        mic_cur_valid = false;
        mic_decoder_reset_pending = true;
    }
    mic_active = active;
    ds4_enable_mic(active && get_config().mic_select != 3);
}

void audio_usb_itf_reset() {
    spk_active = false;
    set_mic_active(false);
}

// BT input report 0x13 (state + mic audio): locate the SBC frame by its
// syncword and queue it for the core1 decoder. Called from the BT data path.
void __not_in_flash_func(audio_mic_bt_data)(const uint8_t *data, uint16_t len) {
    static uint32_t frames = 0, last_log_ms = 0;
    if (!mic_active || get_config().mic_select == 3) return;
    for (uint16_t i = MIC_SBC_SCAN_FROM; i + MIC_SBC_FRAME_LEN <= len - 4; i++) {
        // Match the full frame header (16 kHz mono 16/8 loudness, bitpool
        // 29), not just the syncword: a lone 0x9C also occurs inside SBC
        // payloads and a misaligned frame wedges the decoder.
        if (data[i] != 0x9C || data[i + 1] != 0x31 || data[i + 2] != 0x1D) continue;
        if (queue_is_full(&mic_sbc_fifo)) {
            queue_try_remove(&mic_sbc_fifo, NULL);
        }
        queue_try_add(&mic_sbc_fifo, data + i);
        frames++;
        i += MIC_SBC_FRAME_LEN - 1; // in case a report ever carries more than one frame
    }
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_log_ms >= 2000) {
        last_log_ms = now;
        printf("[MIC] sbc frames=%lu sbc_q=%u pcm_q=%u\n",
               frames, queue_get_level(&mic_sbc_fifo), queue_get_level(&mic_pcm_fifo));
    }
}

bool audio_mic_active() {
    return mic_active;
}

// Assemble and queue one 0x17 audio report once 4 SBC frames are available.
// bt_write() prepends 0xA2 and fills the trailing CRC-32.
static void __not_in_flash_func(audio_bt_task)() {
    static uint16_t frame_counter = 0;

    if (queue_get_level(&sbc_fifo) < FRAMES_PER_REPORT) {
        return;
    }

    const Config_body &cfg = get_config();
    if (cfg.speaker_select == 3) { // audio disabled
        while (queue_try_remove(&sbc_fifo, NULL)) {}
        return;
    }
    const uint8_t target =
            (cfg.speaker_select == 2 || (cfg.speaker_select == 0 && plug_headset))
                ? AUDIO_TARGET_HEADSET
                : AUDIO_TARGET_SPEAKER;

    uint8_t pkt[REPORT_SIZE]{};
    pkt[0] = 0x17;
    pkt[1] = 0x40;
    // Audio header; carry the mic-enable bits so the mic keeps streaming
    // while speaker audio is active (the controller sends one mic report per
    // received output report).
    pkt[2] = 0xA0 | (ds4_get_output_hdr2() & 0x07);
    pkt[3] = frame_counter & 0xFF;
    pkt[4] = (frame_counter >> 8) & 0xFF;
    pkt[5] = target;
    sbc_frame frame{};
    for (int i = 0; i < FRAMES_PER_REPORT; i++) {
        if (queue_try_remove(&sbc_fifo, &frame)) {
            memcpy(pkt + 6 + i * SBC_FRAME_LEN, frame.data, SBC_FRAME_LEN);
        } else {
            printf("[Audio] Warning: sbc_fifo remove failed\n");
        }
    }
    frame_counter += FRAMES_PER_REPORT;
    bt_write(pkt, sizeof(pkt));
}

// EP IN reload, called from the TinyUSB ISR after every ISO IN frame while
// the mic interface is open. The real DS4 v2 ships exactly 32 bytes (16
// samples) every single frame, silence included; Windows' rate-locked
// capture engine turns short or empty frames into audible chopping. So:
// always deposit exactly one frame, padding with silence on underrun.
// tud_audio_write is called only from here, so the FIFO holds at most one
// frame and the driver sends exactly what we deposit. Reloading inside the
// completion ISR (not the main loop) is also what keeps every SOF fed --
// main-loop feeding misses alternate frames and halves the delivered rate.
extern "C" bool __not_in_flash_func(tud_audio_tx_done_isr)(uint8_t rhport, uint16_t n_bytes_sent,
                                                           uint8_t func_id, uint8_t ep_in,
                                                           uint8_t cur_alt_setting) {
    (void) rhport;
    (void) n_bytes_sent;
    (void) func_id;
    (void) ep_in;
    (void) cur_alt_setting;

    int16_t frame[16]; // 1 ms at 16 kHz mono

    // Jitter buffer: BT delivers mic audio in report bursts, so start
    // consuming only once a reserve is queued, and go back to buffering
    // when it runs dry.
    if (!mic_streaming) {
        if (queue_get_level(&mic_pcm_fifo) >= 6) { // ~48 ms buffered
            mic_streaming = true;
        } else {
            memset(frame, 0, sizeof(frame));
            tud_audio_write(frame, sizeof(frame));
            return true;
        }
    }

    for (uint i = 0; i < TU_ARRAY_SIZE(frame); i++) {
        if (!mic_cur_valid) {
            if (queue_try_remove(&mic_pcm_fifo, &mic_cur)) {
                mic_cur_valid = true;
                mic_cur_pos = 0;
            } else {
                mic_streaming = false; // refill the jitter buffer
                for (; i < TU_ARRAY_SIZE(frame); i++) frame[i] = 0;
                break;
            }
        }
        frame[i] = mic_cur.data[mic_cur_pos++];
        if (mic_cur_pos == MIC_PCM_SAMPLES) mic_cur_valid = false;
    }

    tud_audio_write(frame, sizeof(frame));
    return true;
}

void __not_in_flash_func(audio_loop)() {
    // Ensure core1 has initialized the encoder before we process audio
    if (!core1_ready) return;

    audio_bt_task();

    // The controller sends one mic report (0x13) per output report it
    // receives. While the mic is open and no speaker audio is clocking the
    // link with 0x17 reports, prod it with a no-op 0x11 output every 8 ms
    // (one SBC mic frame period).
    if (mic_active && get_config().mic_select != 3 && !spk_active) {
        static uint64_t last_keepalive_us = 0;
        const uint64_t now_us = time_us_64();
        if (now_us - last_keepalive_us >= 8000) {
            last_keepalive_us = now_us;
            const uint8_t payload[31]{}; // flags 0x00: change nothing
            ds4_output(payload, sizeof(payload));
        }
    }

    // Read USB speaker PCM and chunk it into SBC-frame-sized blocks.
    if (!tud_audio_available()) return;

    static pcm_block block{};
    static uint block_pos = 0; // int16 samples filled so far

    int16_t raw[128];
    const uint32_t bytes_read = tud_audio_read(raw, sizeof(raw));
    const uint32_t samples = bytes_read / sizeof(int16_t);

    for (uint32_t i = 0; i < samples; i++) {
        block.data[block_pos++] = raw[i];
        if (block_pos == SBC_FRAME_SAMPLES * 2) {
            block_pos = 0;
            if (queue_is_full(&audio_fifo)) {
                queue_try_remove(&audio_fifo, NULL);
            }
            if (!queue_try_add(&audio_fifo, &block)) {
                printf("[Audio] Warning: audio_fifo add failed\n");
            }
        }
    }
}

void audio_init() {
    queue_init(&audio_fifo, sizeof(pcm_block), 4);
    queue_init(&sbc_fifo, sizeof(sbc_frame), 2 * FRAMES_PER_REPORT);
    queue_init(&mic_sbc_fifo, sizeof(mic_sbc_frame), 8);
    // Deep enough to hold the ISR jitter-buffer reserve (6 blocks) plus an
    // in-flight BT report burst.
    queue_init(&mic_pcm_fifo, sizeof(mic_pcm_block), 16);
#if ENABLE_DEBUG
    debug_fill_core1_stack_watermark(audio_core1_stack,
                                     sizeof(audio_core1_stack) / sizeof(audio_core1_stack[0]));
#endif
    multicore_launch_core1_with_stack(core1_entry, audio_core1_stack, sizeof(audio_core1_stack));
}

static btstack_sbc_encoder_bluedroid_t sbc_encoder_ctx;
static const btstack_sbc_encoder_t *sbc_encoder;

static btstack_sbc_decoder_bluedroid_t sbc_decoder_ctx;
static const btstack_sbc_decoder_t *sbc_decoder;

// Decoder callback (core1): one mic SBC frame yields 128 mono samples.
// bluedroid initializes the OI codec with pcmStride 2 (hardcoded in
// btstack_sbc_bluedroid.c), so mono output arrives with every sample
// duplicated into stereo-interleaved slots even though num_channels claims
// plain mono. Reading the buffer linearly halves the speed and drops each
// frame's second half; take every other sample.
static void mic_handle_pcm(int16_t *data, int num_samples, int num_channels,
                           int sample_rate, void *context) {
    (void) sample_rate;
    (void) context;
    if (num_channels != 1 || num_samples != MIC_PCM_SAMPLES) {
        return; // unexpected format; drop
    }
    mic_pcm_block block;
    for (int i = 0; i < MIC_PCM_SAMPLES; i++) {
        block.data[i] = data[i * 2];
    }
    if (queue_is_full(&mic_pcm_fifo)) {
        queue_try_remove(&mic_pcm_fifo, NULL);
    }
    queue_try_add(&mic_pcm_fifo, &block);
}

// Mic SBC frames from core0 -> decode -> mic_pcm_fifo for the USB IN stream.
static void __not_in_flash_func(mic_proc)() {
    if (mic_decoder_reset_pending) {
        mic_decoder_reset_pending = false;
        sbc_decoder->configure(&sbc_decoder_ctx, SBC_MODE_STANDARD, mic_handle_pcm, NULL);
    }
    mic_sbc_frame frame{};
    if (!queue_try_remove(&mic_sbc_fifo, &frame)) {
        return;
    }
    sbc_decoder->decode_signed_16(&sbc_decoder_ctx, 0, frame.data, sizeof(frame.data));
}

// PCM blocks from core0 -> SBC encode -> sbc_fifo for the 0x17 reports.
static void __not_in_flash_func(speaker_proc)() {
    pcm_block block{};
    if (!queue_try_remove(&audio_fifo, &block)) {
        return;
    }
    if (get_config().speaker_select == 3) {
        return;
    }
    sbc_frame frame{};
    const uint8_t status = sbc_encoder->encode_signed_16(&sbc_encoder_ctx, block.data, frame.data);
    if (status != 0) {
#if ENABLE_VERBOSE
        printf("[Audio] SBC encode failed: %u\n", status);
#endif
        return;
    }
    if (queue_is_full(&sbc_fifo)) {
        queue_try_remove(&sbc_fifo, NULL);
    }
    if (!queue_try_add(&sbc_fifo, &frame)) {
        printf("[Audio] Warning: sbc_fifo add failed\n");
    }
}

void __not_in_flash_func(core1_entry)() {
    // Register core1 as a flash-safe victim so core0's flash_safe_execute() really
    // parks this core while flash is accessed, instead of letting it fault on XIP.
    // Requires PICO_FLASH_ASSUME_CORE1_SAFE=0.
    flash_safe_execute_core_init();

    // The instance vtable is shared; this core has its context.
    sbc_encoder = btstack_sbc_encoder_bluedroid_init_instance(&sbc_encoder_ctx);
    const uint8_t status = sbc_encoder->configure(
        &sbc_encoder_ctx, SBC_MODE_STANDARD,
        16, 8, SBC_ALLOCATION_METHOD_LOUDNESS,
        32000, 25, SBC_CHANNEL_MODE_DUAL_CHANNEL);
    if (status != 0) {
        printf("[Audio] SBC encoder configure failed: %u\n", status);
        return;
    }

    // Mic decoder: format is taken from each frame's SBC header.
    sbc_decoder = btstack_sbc_decoder_bluedroid_init_instance(&sbc_decoder_ctx);
    sbc_decoder->configure(&sbc_decoder_ctx, SBC_MODE_STANDARD, mic_handle_pcm, NULL);

    // Signal core0 that the encoder is ready
    core1_ready = true;

    while (true) {
        speaker_proc();
        mic_proc();
    }
}
