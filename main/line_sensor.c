// CNY70 – czujnik odbiciowy
// Logika: na planszy (jasna powierzchnia) = LOW, krawędź/brak = HIGH (z pull-up)
// Jeśli masz odwróconą logikę – zmień (== 0) na (== 1) w line_sensor_read().
#include "line_sensor.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LINE";

void line_sensor_init(void) {
    // GPIO14, GPIO15, GPIO23 – mogą mieć pull-up wewnętrzny
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

    // GPIO39 – input-only, brak wewnętrznego pull-up; wymaga zewnętrznego 10k do VCC
    gpio_config_t io_ro = {
        .pin_bit_mask = (1ULL << PIN_LINE_FR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_ro);

    ESP_LOGI(TAG, "Line sensors initialized (GPIO14/FR39/15/23)");
}

line_sensor_data_t line_sensor_read(void) {
    line_sensor_data_t d = {
        // HIGH = krawędź (brak odbicia), LOW = na planszy
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
