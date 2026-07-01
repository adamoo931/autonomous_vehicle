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

/* Inicjalizacja magistrali I2C w trybie master. Adresy czujników są zaszyte
 * na stałe w config.h (zweryfikowane na sprzęcie), więc po starcie nie
 * skanujemy już magistrali w poszukiwaniu urządzeń. */
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

/* Zadanie cyklicznego odczytu czujników I2C, okres 100 ms. */
static void sensor_task(void *arg) {
    imu_data_t      imu;
    pyrometer_data_t pyro;
    ina219_data_t   ina;
    int slow = 0;
    bool was_finish = false;   /* poprzedni stan czujnika Halla mety - do wykrycia zbocza */
    bool was_hot    = false;   /* poprzedni stan wykrycia obiektu cieplnego - do wykrycia zbocza */

    while (1) {
        imu_read(&imu);
        pyrometer_read(&pyro);
        ina219_read(&ina);
        /* SHT40 (temperatura/wilgotność) zmienia się wolno - odczyt co ~1 s. */
        if (++slow >= 10) {
            slow = 0;
            sht40_read(NULL);
        }
        /* Czujnik linii i odometria są nieblokujące - czytane bezpośrednio
         * w http_server przy każdym żądaniu HTTP. */

        /* Sygnalizacja stanu diodami LED: czerwona = autonomia nieaktywna,
         * zielona = autonomia aktywna (świeci cały czas, dopóki trwa).
         * Żółta na razie nieobsługiwana. */
        bool auto_on = autonomy_is_enabled();
        led_set_green(auto_on);
        led_set_red(!auto_on);

        /* Meta (czujnik Halla, GPIO34): zbocze "niewykryto -> wykryto"
         * odgrywa jingle raz, nie w pętli, i uruchamia tryb szukania obiektu
         * cieplnego pirometrem. Sam odczyt/aktualizacja pola na dashboardzie
         * dzieje się już w http_server przy każdym /api/sensors. */
        bool is_finish = odometry_get().finish_detected;
        if (is_finish && !was_finish) {
            buzzer_play_ice_cream_song_once();
            pyrometer_start_search();
        }
        was_finish = is_finish;

        /* Obiekt cieplny (różnica temperatury obiekt-otoczenie >= progu, w
         * trybie szukania): zbocze "niewykryto -> wykryto" odgrywa gamę raz
         * i zatrzymuje silniki (motor_stop() jest bezpieczne wywołać nawet
         * gdy już stały - po prostu nic wtedy nie zmienia). */
        bool is_hot = pyro.hot_detected;
        if (is_hot && !was_hot) {
            buzzer_play_scale_once();
            motor_stop();
        }
        was_hot = is_hot;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Trzykrotne mrugnięcie wszystkimi diodami - sygnał startu systemu. */
static void led_startup_blink(void) {
    for (int i = 0; i < 3; i++) {
        led_set_all(true, true, true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set_all(false, false, false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

void app_main(void) {
    /* Monitor webowy musi wystartować pierwszy - przechwytuje wszystkie logi.
     * Zastępuje konsolę UART, której piny są zajęte przez LIDAR. */
    web_monitor_init();

    ESP_LOGI(TAG, "=== Autonomous Vehicle - Start ===");

    /* NVS - wymagane przez sterownik WiFi. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Inicjalizacja sprzętu. */
    led_init();
    led_startup_blink();
    buzzer_init();

    i2c_init();
    motor_init();
    line_sensor_init();
    odometry_init();

    /* Czujniki na magistrali I2C. Brak czujnika nie przerywa rozruchu -
     * pojazd pracuje dalej w oparciu o pozostałe dane. */
    if (imu_init() != ESP_OK)
        ESP_LOGW(TAG, "IMU nie wykryty - kontynuuje bez IMU");

    if (pyrometer_init() != ESP_OK)
        ESP_LOGW(TAG, "Pirometr nie wykryty - kontynuuje bez pirometru");

    if (ina219_init() != ESP_OK)
        ESP_LOGW(TAG, "INA219 nie wykryty - kontynuuje bez pomiaru pradu");

    if (sht40_init() != ESP_OK)
        ESP_LOGW(TAG, "SHT40 nie wykryty - kontynuuje bez temp/wilgotnosci");

    /* LIDAR (można wyłączyć ustawiając LIDAR_ENABLED 0 w config.h). */
    lidar_init();

    /* Sieć WiFi oraz serwer HTTP z dashboardem. */
    wifi_init_sta();
    http_server_start();

    /* Moduł autonomii (uruchamiany z dashboardu przyciskiem autonomii). */
    autonomy_init();

    /* Sygnał gotowości systemu. Autonomia startuje wyłączona, więc dioda
     * czerwona (patrz sensor_task) - stan ten i tak zostanie ustawiony przy
     * pierwszym przebiegu sensor_task, ale robimy to też tutaj, żeby uniknąć
     * chwili bez żadnej zapalonej diody między startem a uruchomieniem zadania. */
    led_set_red(true);
    buzzer_beep();
    ESP_LOGI(TAG, "System gotowy. Dashboard: http://<IP_ESP32>/");

    /* Uruchomienie zadania odczytu czujników, przypięte do rdzenia 1. */
    xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, NULL, 5, NULL, 1);

    /* Pętla główna - lekki nadzór zużycia pamięci sterty. */
    while (1) {
        ESP_LOGD(TAG, "Heap free: %lu B", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
