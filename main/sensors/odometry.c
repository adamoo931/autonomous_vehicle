#include "odometry.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "ODO";

/* Liczniki modyfikowane w przerwaniach — volatile, bo zmieniają się poza
 * normalnym przepływem programu. */
static volatile uint32_t s_pulses_left  = 0;
static volatile uint32_t s_pulses_right = 0;
static volatile bool     s_finish       = false;

/* Czujnik Halla (SS443R) ma wbudowaną histerezę magnetyczną, ale nic nie
 * filtruje zakłóceń elektrycznych na samej linii GPIO - a obok pracuje
 * mostek H z silnikami DC na PWM, klasyczne źródło EMI. Bez tego okna
 * odporności pojedynczy impuls zakłócający liczy się jako obrót koła,
 * nawet gdy pojazd stoi. 5 ms to margines z dużym zapasem (realne impulsy
 * z koła przy rozsądnych prędkościach pojazdu przychodzą co najmniej
 * dziesiątki ms), a krótkie zakłócenia EMI trwają mikrosekundy. */
#define HALL_DEBOUNCE_US 5000

static volatile int64_t s_last_left_us   = 0;
static volatile int64_t s_last_right_us  = 0;
static volatile int64_t s_last_finish_us = 0;

/* Procedury obsługi przerwań (umieszczone w IRAM dla minimalnego opóźnienia).
 * Wykonują tylko inkrementację licznika / ustawienie flagi. */
static void IRAM_ATTR isr_left(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - s_last_left_us >= HALL_DEBOUNCE_US) {
        s_pulses_left++;
        s_last_left_us = now;
    }
}
static void IRAM_ATTR isr_right(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - s_last_right_us >= HALL_DEBOUNCE_US) {
        s_pulses_right++;
        s_last_right_us = now;
    }
}
/* Uwaga: ten sam debounce co dla kół chroni metę tylko przed wielokrotnym
 * "doliczeniem" w obrębie krótkiego zrywu zakłóceń (typowe dla EMI - kilka
 * szybkich zboczy pod rząd), bo s_finish to flaga jednorazowa, więc nie ma
 * to tu większego znaczenia. NIE chroni przed pojedynczym, odosobnionym
 * impulsem zakłócającym, który fałszywie zgłosi metę przy pierwszym
 * wystąpieniu - tu wymagałoby to potwierdzenia, że sygnał utrzymuje się
 * jako niski przez chwilę (odczyt z poziomu zadania, nie ISR), a nie samego
 * zbocza. Zostawiam to do osobnej poprawki, jeśli okaże się potrzebna. */
static void IRAM_ATTR isr_finish(void *arg) {
    int64_t now = esp_timer_get_time();
    if (now - s_last_finish_us >= HALL_DEBOUNCE_US) {
        s_finish = true;
        s_last_finish_us = now;
    }
}

void odometry_init(void) {
    /* Lewe koło — pin z wewnętrznym podciąganiem, przerwanie na zboczu opadającym. */
    gpio_config_t io_left = {
        .pin_bit_mask = (1ULL << PIN_HALL_LEFT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_left);

    /* Prawe koło — GPIO tylko-wejściowe bez podciągania (wymaga zewn. pull-up). */
    gpio_config_t io_right = {
        .pin_bit_mask = (1ULL << PIN_HALL_RIGHT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_right);

    /* Czujnik mety — GPIO tylko-wejściowe. */
    gpio_config_t io_fin = {
        .pin_bit_mask = (1ULL << PIN_HALL_FINISH),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_fin);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_HALL_LEFT,   isr_left,   NULL);
    gpio_isr_handler_add(PIN_HALL_RIGHT,  isr_right,  NULL);
    gpio_isr_handler_add(PIN_HALL_FINISH, isr_finish, NULL);

    ESP_LOGI(TAG, "Odometria zainicjalizowana (L=GPIO%d, R=GPIO%d, META=GPIO%d)",
             PIN_HALL_LEFT, PIN_HALL_RIGHT, PIN_HALL_FINISH);
}

void odometry_reset(void) {
    s_pulses_left  = 0;
    s_pulses_right = 0;
    s_finish       = false;
}

odometry_data_t odometry_get(void) {
    odometry_data_t d;
    d.pulses_left    = s_pulses_left;
    d.pulses_right   = s_pulses_right;
    d.dist_left_mm   = d.pulses_left  * MM_PER_PULSE;
    d.dist_right_mm  = d.pulses_right * MM_PER_PULSE;
    d.dist_total_mm  = (d.dist_left_mm + d.dist_right_mm) / 2.0f;
    d.finish_detected = s_finish;
    return d;
}
