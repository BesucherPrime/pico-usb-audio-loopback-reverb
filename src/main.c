/*
 * Copyright 2025, Hiroyuki OYAMA
 * Modified 2026 by Simon BEIMEL (with assistance from Gemini, Google AI)
 * * Improvements: Added I2S 32b output support from Elehobica, 2021
 * and optimized DMA buffer handling for RP2350.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bootsel_button.h"
#include "bsp/board_api.h"
#include "fx.h"
#include "hardware/clocks.h"
#include "led.h"
#include "pico/stdlib.h"
#include "ringbuffer.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/audio_i2s.h"

static ringbuffer_t rx_buffer = {0};
static ringbuffer_t tx_buffer = {0};

//static int32_t scratch_in[64 * sizeof(int32_t) * 2];
//static int32_t scratch_out[64 * sizeof(int32_t) * 2];
static int32_t scratch_in[256];
static int32_t scratch_out[256];
static uint64_t frac_acc = 0;

static int8_t silence_buf[AUDIO_FRAME_BYTES] = {0};
static uint16_t current_sampling_rate = 48000;
static const size_t FRAME_LENGTH = 48;

#define FX_TIME_LOG_COUNT 1000
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

static uint32_t fx_time_samples[FX_TIME_LOG_COUNT];
static int fx_time_index = 0;
static bool fx_log_ready = false;
static char scratch[1024];

static audio_buffer_pool_t *ap = NULL;

static void update_i2s_output(int32_t *source_buffer, size_t sample_count) {
    if (ap == NULL) return; // Nicht abbrechen bei NULL source_buffer!

    audio_buffer_t *abuf = take_audio_buffer(ap, false);
    if (abuf != NULL) {
        int32_t *dest = (int32_t *) abuf->buffer->bytes;
        
        if (source_buffer != NULL) {
            // Normales Audio-Signal kopieren
            memcpy(dest, source_buffer, sample_count * 2 * sizeof(int32_t));
        } else {
            // EXPLIZITE STILLE: Wenn kein Puffer kommt, Nullen schreiben
            memset(dest, 0, sample_count * 2 * sizeof(int32_t));
        }
        
        abuf->sample_count = sample_count;
        give_audio_buffer(ap, abuf);
    }
}

void init_i2s_output() {
    // 1. Audio-Format definieren (SDK 2.1.1 kompatibel)
    static audio_format_t fmt = {
        .sample_freq = 48000,
        .pcm_format = AUDIO_PCM_FORMAT_S32, 
        .channel_count = 2
    };

    // 2. Buffer-Format für den Pool (4 Bytes pro Sample pro Kanal = 8 Bytes Stride)
    static audio_buffer_format_t producer_format = {
        .format = &fmt,
        .sample_stride = 8 
    };

    // 3. Pool erstellen: 3 Buffer à 256 Samples bieten genug Puffer für USB-Jitter
    ap = audio_new_producer_pool(&producer_format, 12, 256); 

    // 4. Hardware-Konfiguration mit zwei DMA-Kanälen
    audio_i2s_config_t config = {
        .data_pin = 18,
        .clock_pin_base = 16,
        .dma_channel0 = 10,    
        .dma_channel1 = 11,    
        .pio_sm = 0
    };

    // 5. Setup mit zwei Formaten (In/Out) und Config
    audio_i2s_setup(&fmt, &fmt, &config);
    
    // 6. Pool mit der I2S-Hardware verbinden
    audio_i2s_connect(ap);

    // 7: Anti-Plopp: Den Pool mit Stille "vorwärmen", damit der DMA beim Start Nullen findet
    for (int i = 0; i < 64; i++) {
        update_i2s_output(NULL, FRAME_LENGTH);
    }
    // Kurze Pause, damit sich die Signalleitungen elektrisch stabilisieren
    sleep_ms(70);

    audio_i2s_set_enabled(true);
}



void audio_task(void) {
    uint32_t sampling_rate;
    uint8_t bit_rate;
    uint8_t channels;
    static uint32_t silence_counter = 0;

    usb_audio_get_config(&sampling_rate, &bit_rate, &channels);
    if (sampling_rate != current_sampling_rate) {
        current_sampling_rate = sampling_rate;
        return;
    }

    fx_set_enable(bb_get_bootsel_button());

    const size_t rx_samples = FRAME_LENGTH * 2;
    const size_t frame_bytes = FRAME_LENGTH * sizeof(int32_t) * 2;
    if (ringbuffer_size(&rx_buffer) >= rx_samples) {
        silence_counter = 0; // Wir haben wieder Daten
        ringbuffer_pop(&rx_buffer, scratch_in, rx_samples);

        fx_process(scratch_out, scratch_in, FRAME_LENGTH);

        ringbuffer_push(&tx_buffer, scratch_out, rx_samples);

        update_i2s_output(scratch_out, FRAME_LENGTH);
    } else {
        // WICHTIG: Wenn keine USB-Daten da sind, sende Stille an I2S
        // Das verhindert, dass der DMA im Kreis dreht oder Müll spielt.
        silence_counter++;
        if (silence_counter > 500) { // ca. 500ms Stille abwarten
            update_i2s_output(NULL, FRAME_LENGTH); 
            if (silence_counter > 1000) silence_counter = 501; 
        }
    }

}

void led_task(void) { led_update(); }

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                                   uint8_t ep_out, uint8_t cur_alt_setting) {
    if (ringbuffer_capacity(&rx_buffer) == 0)
        return true;
    uint16_t rx_size = tud_audio_read(scratch_in, n_bytes_received);
    if (rx_size != n_bytes_received)
        return true;
    ringbuffer_push(&rx_buffer, scratch_in, rx_size / sizeof(int32_t));
    return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in,
                                   uint8_t cur_alt_setting) {
    (void)rhport;
    (void)itf;
    (void)ep_in;
    (void)cur_alt_setting;

    uint32_t sampling_rate;
    uint8_t bit_rate;
    uint8_t channels;
    usb_audio_get_config(&sampling_rate, &bit_rate, &channels);

    const uint32_t USB_SOF_HZ = 1000;
    frac_acc += (uint64_t)current_sampling_rate;
    uint32_t frames = frac_acc / USB_SOF_HZ;
    frac_acc %= USB_SOF_HZ;
    const size_t samples_needed = frames * channels;
    if (samples_needed == 0)
        return true;

    size_t have = ringbuffer_size(&tx_buffer);
    size_t to_copy = (have < samples_needed) ? have : samples_needed;
    if (to_copy > 0) {
        ringbuffer_pop(&tx_buffer, scratch_out, to_copy);
    }
    if (to_copy < samples_needed) {
        if (to_copy >= channels && to_copy > 0) {
            size_t last_frame_start = to_copy - channels;
            for (size_t i = to_copy; i < samples_needed; i += channels) {
                for (uint8_t ch = 0; ch < channels; ch++) {
                    if (i + ch < samples_needed) {
                        scratch_out[i + ch] = scratch_out[last_frame_start + ch];
                    }
                }
            }
        } else {
            memset(scratch_out + to_copy, 0, (samples_needed - to_copy) * sizeof(int32_t));
        }
    }

    tud_audio_write(scratch_out, (uint16_t)(samples_needed * sizeof(int32_t)));

    return true;
}

int main(void) {
    set_sys_clock_khz(240000, true);

    board_init();
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    board_init_after_tusb();

    fx_init();
    init_i2s_output();

    while (1) {
        tud_task();
        audio_task();
        led_task();
    }
}
