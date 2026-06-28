#include "buzzer.h"
#include "config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "BUZZER";

#define BUZZER_DUTY_RES   LEDC_TIMER_10_BIT
#define BUZZER_DUTY_50    512   // 50% przy 10-bit

static bool s_ready = false;

void buzzer_init(void) {
    ledc_timer_config_t tmr = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz         = 2000,           // wartosc startowa
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);

    ledc_channel_config_t ch = {
        .channel    = BUZZER_LEDC_CHANNEL,
        .duty       = 0,                   // start: cisza
        .gpio_num   = PIN_BUZZER,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = BUZZER_LEDC_TIMER,
    };
    ledc_channel_config(&ch);

    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    s_ready = true;
    ESP_LOGI(TAG, "Buzzer initialized (GPIO%d)", PIN_BUZZER);
}

void buzzer_off(void) {
    if (!s_ready) return;
    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

typedef struct {
    uint32_t freq;
    uint32_t dur_ms;
} tone_arg_t;

// Zadanie jednorazowe: gra ton i sam sie usuwa (nie blokuje HTTP).
static void tone_task(void *arg) {
    tone_arg_t a = *(tone_arg_t *)arg;
    free(arg);

    if (a.freq < 50)    a.freq = 50;
    if (a.freq > 10000) a.freq = 10000;
    if (a.dur_ms > 3000) a.dur_ms = 3000;   // limit bezpieczenstwa

    ledc_set_freq(LEDC_MODE, BUZZER_LEDC_TIMER, a.freq);
    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_50);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(a.dur_ms));

    ledc_set_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, BUZZER_LEDC_CHANNEL);

    vTaskDelete(NULL);
}

void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms) {
    if (!s_ready) return;
    tone_arg_t *a = malloc(sizeof(tone_arg_t));
    if (!a) return;
    a->freq   = freq_hz;
    a->dur_ms = duration_ms;
    if (xTaskCreate(tone_task, "buzz", 2048, a, 4, NULL) != pdPASS) {
        free(a);
    }
}

void buzzer_beep(void) {
    buzzer_tone(2000, 200);
}
