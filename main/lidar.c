#include "lidar.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#if LIDAR_ENABLED

static const char *TAG = "LIDAR";

#define LD06_PACKET_LEN   47
#define LD06_START_BYTE   0x54
#define LD06_DATATYPE     0x2C

static lidar_packet_t s_last  = {0};
static TaskHandle_t   s_task  = NULL;

static volatile uint32_t s_rx_bytes    = 0;
static volatile uint32_t s_ok_packets  = 0;
static volatile uint32_t s_crc_errors  = 0;

// ── Bufor kołowy pełnego skanu ───────────────────────────────
static lidar_scan_point_t s_buf[LIDAR_SCAN_BUFFER];
static uint32_t           s_seq     = 0;     // łączna liczba dodanych punktów
static SemaphoreHandle_t  s_buf_mtx = NULL;  // ochrona bufora producent/konsument

// Dodaje punkt do bufora kołowego (wołane tylko z lidar_task,
// pod muteksem). Punkt otrzymuje numer sekwencyjny s_seq+1.
static inline void buf_push(uint16_t angle, uint16_t dist) {
    s_buf[s_seq % LIDAR_SCAN_BUFFER].angle_hundredths = angle;
    s_buf[s_seq % LIDAR_SCAN_BUFFER].distance_mm      = dist;
    s_seq++;
}

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
    uint8_t data[LD06_PACKET_LEN];
    uint8_t byte;
    int packet_idx = 0;
    TickType_t last_report = xTaskGetTickCount();

    // Komenda wybudzajaca LD14P
    uint8_t start_cmd[8] = {0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
    uart_write_bytes(LIDAR_UART_PORT, (const char *)start_cmd, 8);
    ESP_LOGI(TAG, "Wyslano komende startowa LD14P");
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        // Raport co 2 s (timeout pozwala na to nawet bez danych)
        if (xTaskGetTickCount() - last_report > pdMS_TO_TICKS(2000)) {
            last_report = xTaskGetTickCount();
            ESP_LOGI(TAG, "RX=%lu B/2s, pakiety_OK=%lu, CRC_err=%lu, min=%u mm",
                     (unsigned long)s_rx_bytes, (unsigned long)s_ok_packets,
                     (unsigned long)s_crc_errors, lidar_get_min_distance_mm());
            if (s_rx_bytes == 0) {
                ESP_LOGW(TAG, "Brak danych z LIDARa!");
                // Ponow komende startowa
                uart_write_bytes(LIDAR_UART_PORT, (const char *)start_cmd, 8);
                ESP_LOGI(TAG, "Ponawiam komende startowa...");
            }
            s_rx_bytes = 0;
        }

        // Timeout 100ms (nie portMAX_DELAY!) — musi przepuscic raport
        int rxBytes = uart_read_bytes(LIDAR_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (rxBytes <= 0) continue;
        s_rx_bytes++;

        // Maszyna stanow (jak oryginal)
        if (packet_idx == 0 && byte != 0x54) continue;
        if (packet_idx == 1 && byte != 0x2C) { packet_idx = 0; continue; }

        data[packet_idx++] = byte;

        if (packet_idx == LD06_PACKET_LEN) {
            packet_idx = 0;

            uint8_t expected_crc = crc8(data, LD06_PACKET_LEN - 1);
            bool crc_ok = (expected_crc == data[LD06_PACKET_LEN - 1]);
            if (!crc_ok) {
                s_crc_errors++;
            }

            lidar_packet_t pkt = {0};
            uint16_t start_angle = (uint16_t)((data[5] << 8) | data[4]);
            uint16_t end_angle   = (uint16_t)((data[43] << 8) | data[42]);
            pkt.speed_rpm = (uint16_t)(((data[3] << 8) | data[2]) / 6);

            for (int i = 0; i < 12; i++) {
                int off = 6 + i * 3;
                pkt.points[i].distance_mm = (uint16_t)((data[off+1] << 8) | data[off]);
                pkt.points[i].intensity   = data[off + 2];
                uint16_t range = (end_angle >= start_angle)
                    ? (end_angle - start_angle)
                    : (36000 - start_angle + end_angle);
                pkt.points[i].angle_hundredths = (start_angle + range * i / 11) % 36000;
            }
            pkt.count = 12;
            pkt.valid = true;
            s_last = pkt;
            s_ok_packets++;

            // Do mapy trafiają tylko pakiety z poprawnym CRC – brak śmieci.
            if (crc_ok && s_buf_mtx) {
                xSemaphoreTake(s_buf_mtx, portMAX_DELAY);
                for (int i = 0; i < 12; i++)
                    buf_push(pkt.points[i].angle_hundredths,
                             pkt.points[i].distance_mm);
                xSemaphoreGive(s_buf_mtx);
            }
        }
    }
}

void lidar_init(void) {
    // Mutex bufora skanu (producent = lidar_task, konsument = HTTP).
    if (!s_buf_mtx) s_buf_mtx = xSemaphoreCreateMutex();

    // ── KLUCZOWE: odepnij GPIO1/3 od UART0 (IO MUX) ──
    // W tym firmware UART0 jest aktywny (konsola ESP_LOG).
    // UART0 trzyma GPIO1/3 przez IO MUX (polaczenie sprzetowe).
    // gpio_reset_pin() przywraca pin do stanu GPIO i odlacza od UART0,
    // dzieki czemu UART2 moze je przejac przez GPIO matrix.
    gpio_reset_pin(GPIO_NUM_1);
    gpio_reset_pin(GPIO_NUM_3);
    ESP_LOGI(TAG, "GPIO1/3 odpiety od UART0 (IO MUX -> GPIO matrix)");

    // Kolejnosc identyczna z oryginalnym dzialajacym kodem
    uart_config_t uart_config = {
        .baud_rate  = LIDAR_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_PORT, PIN_LIDAR_TX, PIN_LIDAR_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_PORT, 2048, 0, 0, NULL, 0));

    xTaskCreate(lidar_task, "lidar", 4096, NULL, 5, &s_task);
    ESP_LOGI(TAG, "LIDAR LD14P init OK – UART%d, baud=%d, TX=GPIO%d, RX=GPIO%d",
             LIDAR_UART_PORT, LIDAR_BAUD_RATE, PIN_LIDAR_TX, PIN_LIDAR_RX);
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

uint16_t lidar_get_speed_rpm(void) { return s_last.speed_rpm; }

uint16_t lidar_copy_scan(uint32_t since, lidar_scan_point_t *out,
                         uint16_t max_pts, uint32_t *out_seq) {
    if (!out || max_pts == 0 || !s_buf_mtx) {
        if (out_seq) *out_seq = s_seq;
        return 0;
    }
    uint16_t n = 0;
    xSemaphoreTake(s_buf_mtx, portMAX_DELAY);
    uint32_t cur = s_seq;

    // Najstarszy punkt nadal obecny w buforze kołowym.
    uint32_t first_avail = (cur > LIDAR_SCAN_BUFFER)
                         ? (cur - LIDAR_SCAN_BUFFER + 1) : 1;
    uint32_t start = since + 1;
    if (start < first_avail) start = first_avail;          // klient się spóźnił
    if (cur >= start && (cur - start + 1) > max_pts)       // ogranicz do najnowszych
        start = cur - max_pts + 1;

    for (uint32_t s = start; s <= cur && n < max_pts; s++)
        out[n++] = s_buf[(s - 1) % LIDAR_SCAN_BUFFER];

    if (out_seq) *out_seq = cur;
    xSemaphoreGive(s_buf_mtx);
    return n;
}

#else

void lidar_init(void) { }
void lidar_stop(void) { }
lidar_packet_t lidar_get_last_packet(void) { lidar_packet_t p = {0}; return p; }
uint16_t lidar_get_min_distance_mm(void)   { return 0; }
uint16_t lidar_get_speed_rpm(void)         { return 0; }
uint16_t lidar_copy_scan(uint32_t since, lidar_scan_point_t *out,
                         uint16_t max_pts, uint32_t *out_seq) {
    (void)since; (void)out; (void)max_pts;
    if (out_seq) *out_seq = 0;
    return 0;
}

#endif