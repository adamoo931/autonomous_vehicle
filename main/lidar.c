#include "lidar.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#if LIDAR_ENABLED

static const char *TAG = "LIDAR";

// LD06 pakiet: START(0x54) + 46 bajtów danych
#define LD06_PACKET_LEN   47
#define LD06_START_BYTE   0x54
#define LD06_DATATYPE     0x2C   // 12 pomiarów

static lidar_packet_t s_last  = {0};
static TaskHandle_t   s_task  = NULL;

// Liczniki diagnostyczne (widoczne na monitorze webowym)
static volatile uint32_t s_rx_bytes    = 0;
static volatile uint32_t s_ok_packets  = 0;
static volatile uint32_t s_crc_errors  = 0;

// CRC8/Maxim (Dallas) – używane przez LD06
static uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void lidar_task(void *arg) {
    uint8_t buf[LD06_PACKET_LEN];
    uint8_t byte;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        // Co ~2 s raportuj statystyki (diagnostyka na monitorze webowym)
        if (xTaskGetTickCount() - last_report > pdMS_TO_TICKS(2000)) {
            last_report = xTaskGetTickCount();
            ESP_LOGI(TAG, "RX=%lu B/2s, pakiety_OK=%lu, CRC_err=%lu, min=%u mm",
                     (unsigned long)s_rx_bytes, (unsigned long)s_ok_packets,
                     (unsigned long)s_crc_errors, lidar_get_min_distance_mm());
            if (s_rx_bytes == 0) {
                ESP_LOGW(TAG, "Brak danych z LIDARa! Sprawdz: zasilanie 5V, "
                              "TX LIDARa -> GPIO%d, baud=%d.", PIN_LIDAR_RX, LIDAR_BAUD_RATE);
            }
            s_rx_bytes = 0;
        }

        // Szukaj bajtu startowego
        int r = uart_read_bytes(LIDAR_UART_PORT, &byte, 1, pdMS_TO_TICKS(50));
        if (r != 1) continue;
        s_rx_bytes++;
        if (byte != LD06_START_BYTE) continue;

        // Wczytaj resztę pakietu
        int got = uart_read_bytes(LIDAR_UART_PORT, buf + 1, LD06_PACKET_LEN - 1, pdMS_TO_TICKS(100));
        if (got != LD06_PACKET_LEN - 1) continue;
        s_rx_bytes += got;
        buf[0] = LD06_START_BYTE;

        if (buf[1] != LD06_DATATYPE) continue;

        // Sprawdź CRC (ostatni bajt)
        uint8_t expected_crc = crc8(buf, LD06_PACKET_LEN - 1);
        if (expected_crc != buf[LD06_PACKET_LEN - 1]) {
            s_crc_errors++;
            continue;
        }

        // Parsuj pakiet
        lidar_packet_t pkt = {0};
        // Pole "speed" LD06/LD19 jest w stopniach/s => RPM = (deg/s)/6
        pkt.speed_rpm = (uint16_t)(((buf[3] << 8) | buf[2]) / 6);
        uint16_t start_angle = (uint16_t)((buf[5] << 8) | buf[4]);
        uint16_t end_angle   = (uint16_t)((buf[43] << 8) | buf[42]);

        // 12 pomiarów (po 3 bajty: distance 2B + intensity 1B)
        for (int i = 0; i < 12; i++) {
            int off = 6 + i * 3;
            pkt.points[i].distance_mm     = (uint16_t)((buf[off+1] << 8) | buf[off]);
            pkt.points[i].intensity        = buf[off + 2];
            // Interpolacja kąta
            uint16_t range = (end_angle >= start_angle)
                ? (end_angle - start_angle)
                : (36000 - start_angle + end_angle);
            pkt.points[i].angle_hundredths = (start_angle + range * i / 11) % 36000;
        }
        pkt.count = 12;
        pkt.valid  = true;
        s_last = pkt;
        s_ok_packets++;
    }
}

void lidar_init(void) {
    uart_config_t cfg = {
        .baud_rate  = LIDAR_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(LIDAR_UART_PORT, 2048, 0, 0, NULL, 0);
    uart_param_config(LIDAR_UART_PORT, &cfg);
    uart_set_pin(LIDAR_UART_PORT, PIN_LIDAR_TX, PIN_LIDAR_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(lidar_task, "lidar", 4096, NULL, 5, &s_task);
    ESP_LOGI(TAG, "LIDAR (LD06/LD14P) init OK – UART%d, baud=%d, RX=GPIO%d",
             LIDAR_UART_PORT, LIDAR_BAUD_RATE, PIN_LIDAR_RX);
    ESP_LOGW(TAG, "GPIO1/3 zajete przez LIDAR – monitor USB nieaktywny "
                  "(uzyj monitora webowego w panelu).");
}

void lidar_stop(void) {
    if (s_task) { vTaskDelete(s_task); s_task = NULL; }
    uart_driver_delete(LIDAR_UART_PORT);
}

lidar_packet_t lidar_get_last_packet(void) { return s_last; }

uint16_t lidar_get_min_distance_mm(void) {
    if (!s_last.valid) return 0;
    uint16_t mn = 0xFFFF;
    for (int i = 0; i < s_last.count; i++) {
        if (s_last.points[i].distance_mm > 0 && s_last.points[i].distance_mm < mn)
            mn = s_last.points[i].distance_mm;
    }
    return (mn == 0xFFFF) ? 0 : mn;
}

#else  // LIDAR_ENABLED == 0

void lidar_init(void) { }
void lidar_stop(void) { }
lidar_packet_t lidar_get_last_packet(void) { lidar_packet_t p = {0}; return p; }
uint16_t lidar_get_min_distance_mm(void)   { return 0; }

#endif  // LIDAR_ENABLED
