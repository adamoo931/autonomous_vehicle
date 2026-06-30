#include "line_sensor.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LINE";

/*
 * Konwencja stanu logicznego czujnika CNY70:
 *   na planszy (jasna powierzchnia) -> odbicie -> stan niski,
 *   krawędź / brak podłoża          -> brak odbicia -> stan wysoki.
 * Przy odwrotnym okablowaniu zmień porównania w line_sensor_read().
 */

void line_sensor_init(void) {
    /* Piny z dostępnym wewnętrznym rezystorem podciągającym. */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LINE_FL) |
                        (1ULL << PIN_LINE_BL) |
                        (1ULL << PIN_LINE_BR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* PIN_LINE_FR to GPIO tylko-wejściowe bez wewnętrznego podciągania —
     * wymaga zewnętrznego rezystora pull-up do VCC. */
    gpio_config_t io_ro = {
        .pin_bit_mask = (1ULL << PIN_LINE_FR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_ro);

    ESP_LOGI(TAG, "Czujniki linii zainicjalizowane (GPIO%d/%d/%d/%d)",
             PIN_LINE_FL, PIN_LINE_FR, PIN_LINE_BL, PIN_LINE_BR);
}

line_sensor_data_t line_sensor_read(void) {
    line_sensor_data_t d = {
        .front_left  = (gpio_get_level(PIN_LINE_FL) == 1),
        .front_right = (gpio_get_level(PIN_LINE_FR) == 1),
        .back_left   = (gpio_get_level(PIN_LINE_BL) == 1),
        .back_right  = (gpio_get_level(PIN_LINE_BR) == 1),
    };
    return d;
}

bool line_sensor_any_edge(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.front_right || d.back_left || d.back_right;
}
