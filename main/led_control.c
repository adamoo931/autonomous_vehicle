#include "led_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "LED";

/* Buforowany stan diod - zwracany przez funkcje led_get_*(). */
static bool s_red = false, s_yellow = false, s_green = false;

void led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_RED_LED) |
                        (1ULL << PIN_YELLOW_LED) |
                        (1ULL << PIN_GREEN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    led_set_all(false, false, false);
    ESP_LOGI(TAG, "Diody LED zainicjalizowane");
}

/* Diody są podłączone anodą na stałe do szyny 3,3V, a katodą do GPIO
 * ("sink"/dolne sterowanie) - zapalają się, gdy GPIO ściąga katodę do
 * masy (LOW), a gasną przy GPIO=HIGH (brak różnicy potencjałów na
 * diodzie). Logika jest więc odwrócona względem typowego "GPIO=1 -> świeci". */
void led_set_red(bool on)    { s_red    = on; gpio_set_level(PIN_RED_LED,    on ? 0 : 1); }
void led_set_yellow(bool on) { s_yellow = on; gpio_set_level(PIN_YELLOW_LED, on ? 0 : 1); }
void led_set_green(bool on)  { s_green  = on; gpio_set_level(PIN_GREEN_LED,  on ? 0 : 1); }

void led_set_all(bool red, bool yellow, bool green) {
    led_set_red(red);
    led_set_yellow(yellow);
    led_set_green(green);
}

bool led_get_red(void)    { return s_red;    }
bool led_get_yellow(void) { return s_yellow; }
bool led_get_green(void)  { return s_green;  }
