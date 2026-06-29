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
#include "ina219.h"
#include "sht40.h"
#include "buzzer.h"
#include "lidar.h"
#include "autonomy.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "web_monitor.h"

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

// ─── Skaner I2C – wypisuje wykryte adresy na monitor (web/UART) ─
static void i2c_scan(void) {
    ESP_LOGI(TAG, "Skanowanie magistrali I2C...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t h = i2c_cmd_link_create();
        i2c_master_start(h);
        i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(h);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(30));
        i2c_cmd_link_delete(h);
        if (ret == ESP_OK) {
            const char *name = "?";
            if (addr == 0x5A) name = "MLX90614 (pirometr)";
            else if (addr == 0x68 || addr == 0x69) name = "ICM-20948 (IMU)";
            else if (addr == 0x44) name = "SHT40";
            else if (addr >= 0x40 && addr <= 0x4F) name = "INA219";
            ESP_LOGI(TAG, "  -> wykryto 0x%02X  [%s]", addr, name);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  Nie wykryto ZADNEGO ukladu I2C! Sprawdz: zasilanie czujnikow, "
                      "rezystory podciagajace SDA/SCL, oraz piny GPIO%d/GPIO%d.",
                      PIN_I2C_SDA, PIN_I2C_SCL);
    } else {
        ESP_LOGI(TAG, "Skan I2C zakonczony – znaleziono %d ukl.", found);
    }
}

// ─── Task: odczyt czujników co 100 ms ──────────────────────
static void sensor_task(void *arg) {
    imu_data_t      imu;
    pyrometer_data_t pyro;
    ina219_data_t   ina;
    int slow = 0;

    while (1) {
        imu_read(&imu);
        pyrometer_read(&pyro);
        ina219_read(&ina);
        // SHT40 (temperatura/wilgotnosc) zmienia sie wolno – co ~1 s
        if (++slow >= 10) {
            slow = 0;
            sht40_read(NULL);
        }
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
    // Monitor webowy MUSI byc pierwszy – przechwytuje wszystkie logi
    // (zamiast martwej konsoli UART, ktora zajmuje LIDAR).
    web_monitor_init();

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
    buzzer_init();

    i2c_init();
    i2c_scan();             // pokaze realne adresy na monitorze webowym
    motor_init();
    line_sensor_init();
    odometry_init();

    // ── I2C czujniki ──
    if (imu_init() != ESP_OK)
        ESP_LOGW(TAG, "IMU nie wykryty – kontynuuję bez IMU");

    if (pyrometer_init() != ESP_OK)
        ESP_LOGW(TAG, "Pirometr nie wykryty – kontynuuję bez pirometru");

    if (ina219_init() != ESP_OK)
        ESP_LOGW(TAG, "INA219 nie wykryty – kontynuuję bez pomiaru pradu");

    if (sht40_init() != ESP_OK)
        ESP_LOGW(TAG, "SHT40 nie wykryty – kontynuuję bez temp/wilgotnosci");

    // ── LIDAR (można wyłączyć przez LIDAR_ENABLED 0 w config.h) ──
    lidar_init();

    // ── Sieć ──
    wifi_init_sta();
    http_server_start();

    // ── Autonomia (sterowana z dashboardu: „Symuluj autonomię") ──
    autonomy_init();

    // ── Sygnał gotowości ──
    led_set_green(true);
    buzzer_beep();          // krotki sygnal: system wstal
    ESP_LOGI(TAG, "System gotowy. Dashboard: http://<IP_ESP32>/");

    // ── Uruchom taski FreeRTOS ──
    xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, NULL, 5, NULL, 1);

    // Pętla główna – watchdog
    while (1) {
        ESP_LOGD(TAG, "Heap free: %lu B", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
