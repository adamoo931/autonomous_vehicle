#include "odometry.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ODO";

/* Liczniki modyfikowane w przerwaniach — volatile, bo zmieniają się poza
 * normalnym przepływem programu. */
static volatile uint32_t s_pulses_left  = 0;
static volatile uint32_t s_pulses_right = 0;
static volatile bool     s_finish       = false;

/* Procedury obsługi przerwań (umieszczone w IRAM dla minimalnego opóźnienia).
 * Wykonują tylko inkrementację licznika / ustawienie flagi. */
static void IRAM_ATTR isr_left(void *arg)  { s_pulses_left++;  }
static void IRAM_ATTR isr_right(void *arg) { s_pulses_right++; }
static void IRAM_ATTR isr_finish(void *arg){ s_finish = true;  }

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
