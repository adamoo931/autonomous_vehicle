#include "hall_finish.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "HALLFIN";

/* Polaryzacja DO — patrz nagłówek. Startuje z config.h, zmieniana z dashboardu. */
static bool s_active_low     = (HALL_FINISH_DO_ACTIVE_LOW != 0);

/* Reakcja na Hall mety w jeździe ręcznej (patrz nagłówek). Domyślnie nie. */
static bool s_manual_enabled = false;

/* Zatrzask impulsu DO - czas (ms) ostatniej aktywności; 0 = nigdy / skasowany. */
static unsigned int s_last_active_ms = 0;

void hall_finish_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_HALL_FINISH_DO),
        .mode         = GPIO_MODE_INPUT,
        /* Zapasowy pull-up po stronie ESP32 — moduł LM393 ma zwykle wyjście
         * typu open-collector z własnym podciąganiem, ale gdyby go nie miał,
         * ten pull-up utrzyma zdefiniowany stan spoczynkowy (DO = HIGH). */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    ESP_LOGI(TAG,
             "Cyfrowy Hall mety: DO na GPIO%d (wejscie + pull-up), meta = DO %s. "
             "Prog ustaw potencjometrem modulu, obserwujac 'DO surowy' na dashboardzie.",
             PIN_HALL_FINISH_DO, s_active_low ? "LOW" : "HIGH");
}

int hall_finish_do_raw(void) {
    return gpio_get_level(PIN_HALL_FINISH_DO);
}

bool hall_finish_detected(void) {
    int raw = gpio_get_level(PIN_HALL_FINISH_DO);
    return s_active_low ? (raw == 0) : (raw != 0);
}

void hall_finish_poll(unsigned int now_ms) {
    if (hall_finish_detected()) s_last_active_ms = now_ms ? now_ms : 1u;
}

bool hall_finish_seen_recently(unsigned int now_ms, unsigned int within_ms) {
    return s_last_active_ms != 0 && (now_ms - s_last_active_ms) < within_ms;
}

void hall_finish_clear_latch(void) { s_last_active_ms = 0; }

void hall_finish_set_active_low(bool active_low) {
    s_active_low = active_low;
    ESP_LOGI(TAG, "Polaryzacja DO: meta = DO %s.", active_low ? "LOW" : "HIGH");
}

bool hall_finish_get_active_low(void) { return s_active_low; }

void hall_finish_set_manual_enabled(bool enabled) {
    s_manual_enabled = enabled;
    ESP_LOGI(TAG, "Reakcja na Hall mety w jezdzie recznej: %s",
             enabled ? "wlaczona" : "wylaczona");
}

bool hall_finish_get_manual_enabled(void) { return s_manual_enabled; }
