#include "motor_driver.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>   /* abs() */

static const char *TAG = "MOTOR";

/* Buforowana prędkość każdego silnika (-100..100). */
static int s_left  = 0;
static int s_right = 0;

static inline int clamp(int v, int lo, int hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

/* Przeliczenie prędkości w procentach na wypełnienie PWM (10-bit) i zapis
 * do kanału LEDC. Bierze wartość bezwzględną - kierunek ustawiają piny GPIO. */
static void set_duty(ledc_channel_t ch, int pct) {
    uint32_t duty = (uint32_t)((abs(pct) * 1023UL) / 100UL);
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

void motor_init(void) {
    /* Piny kierunkowe obu mostków. */
    gpio_config_t dir = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) |
                        (1ULL << PIN_BIN1) | (1ULL << PIN_BIN2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir);

    /* Wspólny timer LEDC dla obu kanałów PWM. */
    ledc_timer_config_t tmr = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);

    /* Kanał lewy (A). */
    ledc_channel_config_t chA = {
        .channel    = LEDC_CHANNEL_LEFT,
        .duty       = 0,
        .gpio_num   = PIN_PWMA,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
    };
    ledc_channel_config(&chA);

    /* Kanał prawy (B). */
    ledc_channel_config_t chB = {
        .channel    = LEDC_CHANNEL_RIGHT,
        .duty       = 0,
        .gpio_num   = PIN_PWMB,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
    };
    ledc_channel_config(&chB);

    motor_stop();
    ESP_LOGI(TAG, "Sterownik silnikow zainicjalizowany");
}

void motor_set_left(int speed) {
    speed   = clamp(speed, -100, 100);
    s_left  = speed;

    /* Dodatnia prędkość = przód, ujemna = wstecz, zero = wybieg. */
    if (speed > 0) {
        gpio_set_level(PIN_AIN1, 1);
        gpio_set_level(PIN_AIN2, 0);
    } else if (speed < 0) {
        gpio_set_level(PIN_AIN1, 0);
        gpio_set_level(PIN_AIN2, 1);
    } else {
        gpio_set_level(PIN_AIN1, 0);
        gpio_set_level(PIN_AIN2, 0);
    }
    set_duty(LEDC_CHANNEL_LEFT, speed);
}

void motor_set_right(int speed) {
    speed   = clamp(speed, -100, 100);
    s_right = speed;

    /* Logika kierunku odwrócona względem motor_set_left() celowo - silnik
     * prawego koła fizycznie kręci się w przeciwną stronę niż lewy przy tej
     * samej elektrycznej konwencji IN1/IN2 (przewody silnika/BIN1-BIN2 są
     * podłączone do mostka TB6612 w odwrotnej polaryzacji względem kanału
     * A). Bez tego dodatnia wartość na prawym kanale kręciła kołem wstecz. */
    if (speed > 0) {
        gpio_set_level(PIN_BIN1, 0);
        gpio_set_level(PIN_BIN2, 1);
    } else if (speed < 0) {
        gpio_set_level(PIN_BIN1, 1);
        gpio_set_level(PIN_BIN2, 0);
    } else {
        gpio_set_level(PIN_BIN1, 0);
        gpio_set_level(PIN_BIN2, 0);
    }
    set_duty(LEDC_CHANNEL_RIGHT, speed);
}

/* Skręty realizowane różnicowo: koło wewnętrzne na połowie prędkości. */
void motor_forward(uint8_t speed)  { motor_set_left( speed); motor_set_right( speed); }
void motor_backward(uint8_t speed) { motor_set_left(-speed); motor_set_right(-speed); }
void motor_turn_left(uint8_t speed)  { motor_set_left(-(speed/2)); motor_set_right( speed); }
void motor_turn_right(uint8_t speed) { motor_set_left( speed);     motor_set_right(-(speed/2)); }
void motor_stop(void) { motor_set_left(0); motor_set_right(0); }

int motor_get_left_speed(void)  { return s_left;  }
int motor_get_right_speed(void) { return s_right; }
