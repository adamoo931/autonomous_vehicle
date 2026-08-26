#include "line_sensor.h"
#include "config.h"
#include "motor_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LINE";
static volatile bool s_edge_detected = false;
static volatile line_sensor_data_t s_last_trigger = {false, false, false, false};
static TaskHandle_t s_edge_notify_task = NULL;

static inline bool is_edge_level(int gpio_num) {
    return gpio_get_level(gpio_num) == 1;
}

static void IRAM_ATTR line_sensor_isr_handler(void *arg) {
    (void)arg;
    
    // Zapamietaj stan czujników w momencie przerwania
    s_last_trigger.front_left  = is_edge_level(PIN_LINE_FL);
    s_last_trigger.front_right = is_edge_level(PIN_LINE_FR);
    s_last_trigger.back_left   = is_edge_level(PIN_LINE_BL);
    s_last_trigger.back_right  = is_edge_level(PIN_LINE_BR);
    
    // Natychmiastowo zatrzymaj silniki (IRAM-safe)
    motor_stop();
    
    // Ustaw flagę dla pętli głównej
    s_edge_detected = true;
    
    if (s_edge_notify_task) {
        BaseType_t wake = pdFALSE;
        vTaskNotifyGiveFromISR(s_edge_notify_task, &wake);
        portYIELD_FROM_ISR(wake);
    }
}

/*
 * Konwencja stanu logicznego czujnika CNY70:
 *   na planszy (jasna powierzchnia) -> odbicie -> stan niski,
 *   krawędź / brak podłoża          -> brak odbicia -> stan wysoki.
 * Przy odwrotnym okablowaniu zmień porównania w line_sensor_read().
 */

void line_sensor_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LINE_FL) |
                        (1ULL << PIN_LINE_BL) |
                        (1ULL << PIN_LINE_BR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    }; 
    gpio_config(&io);

    gpio_config_t io_ro = {
        .pin_bit_mask = (1ULL << PIN_LINE_FR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_ro);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_LINE_FL, line_sensor_isr_handler, NULL);
    gpio_isr_handler_add(PIN_LINE_FR, line_sensor_isr_handler, NULL);
    gpio_isr_handler_add(PIN_LINE_BL, line_sensor_isr_handler, NULL);
    gpio_isr_handler_add(PIN_LINE_BR, line_sensor_isr_handler, NULL);

    ESP_LOGI(TAG, "Line sensors initialized with ISR (GPIO14/FR39/15/23)");
}

line_sensor_data_t line_sensor_read(void) {
    line_sensor_data_t d = {
        .front_left  = is_edge_level(PIN_LINE_FL),
        .front_right = is_edge_level(PIN_LINE_FR),
        .back_left   = is_edge_level(PIN_LINE_BL),
        .back_right  = is_edge_level(PIN_LINE_BR),
    };
    return d;
}

bool line_sensor_any_edge(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.front_right || d.back_left || d.back_right;
}

bool line_sensor_edge_detected(void) {
    return s_edge_detected;
}

void line_sensor_clear_edge_flag(void) {
    s_edge_detected = false;
}

void line_sensor_set_notify_task(TaskHandle_t task) {
    s_edge_notify_task = task;
}

// ============================================================
//  Zwraca ostatnio wykryty stan czujników (zapamiętany w ISR).
// ============================================================
line_sensor_data_t line_sensor_get_last_trigger(void) {
    return s_last_trigger;
}
