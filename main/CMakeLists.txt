# Rejestracja komponentu "main". Czujniki znajdują się w podkatalogu
# sensors/ (dodanym też do INCLUDE_DIRS, aby ich nagłówki były widoczne
# w całym projekcie). Pozostałe moduły leżą bezpośrednio w main/.
idf_component_register(SRCS
    # Rdzeń aplikacji i sterowanie
    "main.c"
    "motor_driver.c"
    "led_control.c"
    "buzzer.c"
    "autonomy.c"
    # Łączność i serwer
    "wifi_manager.c"
    "http_server.c"
    "web_monitor.c"
    # Czujniki (podkatalog sensors/)
    "sensors/imu.c"
    "sensors/ina219.c"
    "sensors/sht40.c"
    "sensors/pyrometer.c"
    "sensors/line_sensor.c"
    "sensors/odometry.c"
    "sensors/lidar.c"
    INCLUDE_DIRS "." "sensors"
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        driver
        cjson
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
        esp_timer
)
