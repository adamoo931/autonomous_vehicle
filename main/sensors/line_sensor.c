#include "line_sensor.h"
#include "config.h"
#include "ads1115.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LINE";

/*
 * Przód-lewy (A1), tył-lewy (A2) i tył-prawy (A3) idą analogowo przez ADS1115:
 * linia wykryta = napięcie PONIŻEJ progu danego kanału (LINE_FL/BL/BR_THRESHOLD_V
 * w config.h). Przód-prawy jest cyfrowy na GPIO PIN_LINE_FR (stan wysoki = wykryto).
 * Przy odwrotnym okablowaniu/polaryzacji zmień porównania w line_sensor_read().
 */

void line_sensor_init(void) {
    /* Tylko przód-prawy pozostaje na GPIO — pin tylko-wejściowy bez
     * wewnętrznego podciągania, wymaga zewnętrznego rezystora pull-up do VCC. */
    gpio_config_t io_ro = {
        .pin_bit_mask = (1ULL << PIN_LINE_FR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_ro);

    ESP_LOGI(TAG, "Czujniki linii: analogowo przez ADS1115 - wykrycie ponizej progu [V]: przod-L(A1)=%.2f tyl-L(A2)=%.2f tyl-P(A3)=%.2f; przod-P cyfrowo (GPIO%d)",
             (double)LINE_FL_THRESHOLD_V, (double)LINE_BL_THRESHOLD_V,
             (double)LINE_BR_THRESHOLD_V, PIN_LINE_FR);
}

line_sensor_data_t line_sensor_read(void) {
    ads1115_data_t a = ads1115_get_last();

    line_sensor_data_t d = {
        .front_left   = (a.line_fl_v < LINE_FL_THRESHOLD_V),
        .front_right  = (gpio_get_level(PIN_LINE_FR) == 1),
        .back_left    = (a.line_bl_v < LINE_BL_THRESHOLD_V),
        .back_right   = (a.line_br_v < LINE_BR_THRESHOLD_V),
        .front_left_v = a.line_fl_v,
        .back_left_v  = a.line_bl_v,
        .back_right_v = a.line_br_v,
    };
    return d;
}

bool line_sensor_any_edge(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.front_right || d.back_left || d.back_right;
}
