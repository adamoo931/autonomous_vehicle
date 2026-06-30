#include "buzzer.h"
#include "config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "BUZZER";

#define BUZZER_DUTY_RES   LEDC_TIMER_10_BIT
#define BUZZER_DUTY_50    512   /* wypełnienie 50% przy rozdzielczości 10-bit */

static bool s_ready = false;
/* Zapobiega jednoczesnemu sterowaniu kanałem LEDC przez dwa zadania
 * (np. ton testowy wciśnięty w trakcie odtwarzania melodii). */
static volatile bool s_busy = false;
/* Sygnał przerwania pętli melodii, ustawiany przez buzzer_off(). */
static volatile bool s_stop_requested = false;

/* Standardowe częstotliwości nut (notacja Arduino "pitches.h"). */
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_G5 784

/* "Turkey in the Straw" - XIX-wieczna piosenka ludowa (domena publiczna),
 * powszechnie kojarzona z jinglem furgonetek z lodami. Uproszczona,
 * skrócona transkrypcja głównej frazy melodii. */
static const buzzer_note_t s_ice_cream_song[] = {
    { NOTE_G4, 150 }, { NOTE_G4, 150 }, { NOTE_A4, 150 }, { NOTE_G4, 150 }, { NOTE_C5, 300 }, { NOTE_B4, 300 },
    { NOTE_G4, 150 }, { NOTE_G4, 150 }, { NOTE_A4, 150 }, { NOTE_G4, 150 }, { NOTE_D5, 300 }, { NOTE_C5, 300 },
    { NOTE_G4, 150 }, { NOTE_G4, 150 }, { NOTE_G5, 150 }, { NOTE_E5, 150 }, { NOTE_C5, 150 }, { NOTE_B4, 150 }, { NOTE_A4, 300 },
    { NOTE_F4, 150 }, { NOTE_F4, 150 }, { NOTE_E4, 150 }, { NOTE_D4, 150 }, { NOTE_C4, 450 },
};
#define ICE_CREAM_SONG_LEN (sizeof(s_ice_cream_song) / sizeof(s_ice_cream_song[0]))

void buzzer_init(void) {
    ledc_timer_config_t tmr = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz         = 2000,           /* częstotliwość startowa */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch = {
        .channel    = BUZZER_LEDC_CHANNEL,
        .duty       = 0,                   /* start w ciszy */
        .gpio_num   = PIN_BUZZER,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = BUZZER_LEDC_TIMER,
    };
    ledc_channel_config(&ch);

    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    s_ready = true;
    ESP_LOGI(TAG, "Buzzer zainicjalizowany (GPIO%d)", PIN_BUZZER);
}

void buzzer_off(void) {
    if (!s_ready) return;
    s_stop_requested = true;   /* przerywa pętlę melodii, jeśli akurat gra */
    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

typedef struct {
    uint32_t freq;
    uint32_t dur_ms;
} tone_arg_t;

/* Zadanie jednorazowe: gra ton przez zadany czas i samo się usuwa.
 * Dzięki temu odtwarzanie nie blokuje wątku wywołującego (np. HTTP). */
static void tone_task(void *arg) {
    tone_arg_t a = *(tone_arg_t *)arg;
    free(arg);

    /* Ograniczenia bezpieczeństwa: zakres częstotliwości i czas trwania. */
    if (a.freq < 50)    a.freq = 50;
    if (a.freq > 10000) a.freq = 10000;
    if (a.dur_ms > 3000) a.dur_ms = 3000;

    ledc_set_freq(LEDC_MODE, BUZZER_LEDC_TIMER, a.freq);
    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_50);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(a.dur_ms));

    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    s_busy = false;
    vTaskDelete(NULL);
}

void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms) {
    if (!s_ready || s_busy) return;
    tone_arg_t *a = malloc(sizeof(tone_arg_t));
    if (!a) return;
    a->freq   = freq_hz;
    a->dur_ms = duration_ms;
    s_busy = true;
    if (xTaskCreate(tone_task, "buzz", 2048, a, 4, NULL) != pdPASS) {
        s_busy = false;
        free(a);
    }
}

void buzzer_beep(void) {
    buzzer_tone(2000, 200);
}

typedef struct {
    const buzzer_note_t *notes;
    size_t count;
    bool   loop;
    uint32_t loop_gap_ms;
} melody_arg_t;

/* Czeka 'ms' mlisekund, sprawdzając co 50 ms flagę przerwania - dzięki temu
 * buzzer_off() przerywa odtwarzanie najwyżej z tym opóźnieniem zamiast
 * czekać do końca całej przerwy między powtórzeniami. */
static bool wait_or_stop(uint32_t ms) {
    while (ms > 0) {
        if (s_stop_requested) return true;
        uint32_t step = ms > 50 ? 50 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        ms -= step;
    }
    return s_stop_requested;
}

/* Zadanie: gra sekwencję nut (raz albo w pętli z przerwą) i samo się usuwa.
 * Między nutami krótka cisza (staccato) - bez niej sąsiednie dźwięki o
 * różnej częstotliwości zlewają się w jeden ciągły, nieczytelny ton. */
static void melody_task(void *arg) {
    melody_arg_t a = *(melody_arg_t *)arg;
    free(arg);

    do {
        for (size_t i = 0; i < a.count; i++) {
            if (s_stop_requested) goto done;

            uint32_t freq = a.notes[i].freq_hz;
            uint32_t dur  = a.notes[i].dur_ms;
            if (freq < 50)  freq = 50;
            if (freq > 10000) freq = 10000;

            uint32_t on_ms = dur > 20 ? dur - 15 : dur;

            ledc_set_freq(LEDC_MODE, BUZZER_LEDC_TIMER, freq);
            ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_50);
            ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);
            if (wait_or_stop(on_ms)) break;

            ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);
            if (wait_or_stop(dur - on_ms)) break;
        }

        if (a.loop && !s_stop_requested) {
            if (wait_or_stop(a.loop_gap_ms)) break;
        }
    } while (a.loop && !s_stop_requested);

done:
    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    s_stop_requested = false;
    s_busy = false;
    vTaskDelete(NULL);
}

void buzzer_play_melody(const buzzer_note_t *notes, size_t count,
                         bool loop, uint32_t loop_gap_ms) {
    if (!s_ready || s_busy || !notes || count == 0) return;
    melody_arg_t *a = malloc(sizeof(melody_arg_t));
    if (!a) return;
    a->notes       = notes;
    a->count       = count;
    a->loop        = loop;
    a->loop_gap_ms = loop_gap_ms;
    s_stop_requested = false;   /* czyści ewentualną nieaktualną flagę z poprzedniego buzzer_off() */
    s_busy = true;
    if (xTaskCreate(melody_task, "buzz_song", 2048, a, 4, NULL) != pdPASS) {
        s_busy = false;
        free(a);
    }
}

void buzzer_play_ice_cream_song(void) {
    buzzer_play_melody(s_ice_cream_song, ICE_CREAM_SONG_LEN, true, 3000);
}
