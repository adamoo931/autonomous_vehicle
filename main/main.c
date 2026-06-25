#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "config.h"
#include "motor_driver.h"
#include "led_control.h"
#include "line_sensor.h"
#include "odometry.h"
#include "pyrometer.h"
#include "imu.h"
#include "lidar.h"
#include "wifi_manager.h"
#include "http_server.h"

static const char *TAG = "MAIN";

// ─── Inicjalizacja magistrali I2C ──────────────────────────
static void i2c_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    ESP_LOGI(TAG, "I2C OK (SDA=GPIO%d, SCL=GPIO%d, %d Hz)",
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
}

// ─── Task: odczyt czujników co 100 ms ──────────────────────
static void sensor_task(void *arg) {
    imu_data_t      imu;
    pyrometer_data_t pyro;

    while (1) {
        imu_read(&imu);
        pyrometer_read(&pyro);
        // line_sensor i odometria są nieblokujące – czytane
        // bezpośrednio z http_server przy każdym żądaniu.

        // Sygnalizacja statusu diodami
        if (pyro.finish_detected) {
            led_set_green(true);
        }
        if (line_sensor_any_edge()) {
            led_set_yellow(true);
        } else {
            led_set_yellow(false);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ─── Startup sekwencja LED ──────────────────────────────────
static void led_startup_blink(void) {
    for (int i = 0; i < 3; i++) {
        led_set_all(true, true, true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set_all(false, false, false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

// ─── app_main ───────────────────────────────────────────────
void app_main(void) {
    ESP_LOGI(TAG, "=== Autonomous Vehicle – Start ===");

    // NVS (wymagane przez WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // ── Sprzęt ──
    led_init();
    led_startup_blink();

    i2c_init();
    motor_init();
    line_sensor_init();
    odometry_init();

    // ── I2C czujniki ──
    if (imu_init() != ESP_OK)
        ESP_LOGW(TAG, "IMU nie wykryty – kontynuuję bez IMU");

    if (pyrometer_init() != ESP_OK)
        ESP_LOGW(TAG, "Pirometr nie wykryty – kontynuuję bez pirometru");

    // ── LIDAR (można wyłączyć przez LIDAR_ENABLED 0 w config.h) ──
    lidar_init();

    // ── Sieć ──
    wifi_init_sta();
    http_server_start();

    // ── Sygnał gotowości ──
    led_set_green(true);
    ESP_LOGI(TAG, "System gotowy. Dashboard: http://<IP_ESP32>/");

    // ── Uruchom taski FreeRTOS ──
    xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, NULL, 5, NULL, 1);

    // Pętla główna – watchdog
    while (1) {
        ESP_LOGD(TAG, "Heap free: %lu B", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
