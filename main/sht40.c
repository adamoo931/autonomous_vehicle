#include "sht40.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHT40";

// ── Komendy SHT40 ──
#define SHT40_CMD_MEAS_HIGH   0xFD   // pomiar T+RH, wysoka precyzja (~8.2 ms)
#define SHT40_CMD_SOFT_RESET  0x94

static sht40_data_t s_last = {0};

// Adresy do przeszukania (standard to 0x44)
static const uint8_t s_candidates[] = { SHT40_ADDR, 0x44, 0x45, 0x46 };

// CRC8 wg datasheet Sensirion: wielomian 0x31, init 0xFF
static uint8_t sht_crc(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

static esp_err_t sht_send_cmd(uint8_t addr, uint8_t cmd) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, cmd, true);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    return ret;
}

static esp_err_t sht_read_raw(uint8_t addr, uint8_t *buf6) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(h, buf6, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(h, buf6 + 5, I2C_MASTER_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    return ret;
}

// Pelny pomiar: komenda -> czekaj -> odczyt 6 bajtow -> sprawdz CRC
static esp_err_t sht_measure(uint8_t addr, float *t, float *rh) {
    if (sht_send_cmd(addr, SHT40_CMD_MEAS_HIGH) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));   // wysoka precyzja: max 8.2 ms

    uint8_t b[6] = {0};
    if (sht_read_raw(addr, b) != ESP_OK) return ESP_FAIL;

    if (sht_crc(&b[0], 2) != b[2]) return ESP_ERR_INVALID_CRC;
    if (sht_crc(&b[3], 2) != b[5]) return ESP_ERR_INVALID_CRC;

    uint16_t t_raw  = (uint16_t)((b[0] << 8) | b[1]);
    uint16_t rh_raw = (uint16_t)((b[3] << 8) | b[4]);

    *t  = -45.0f + 175.0f * ((float)t_raw  / 65535.0f);
    float h = -6.0f + 125.0f * ((float)rh_raw / 65535.0f);
    if (h < 0.0f)   h = 0.0f;
    if (h > 100.0f) h = 100.0f;
    *rh = h;
    return ESP_OK;
}

esp_err_t sht40_init(void) {
    s_last.initialized = false;
    s_last.address     = 0;

    for (size_t i = 0; i < sizeof(s_candidates); i++) {
        uint8_t addr = s_candidates[i];
        sht_send_cmd(addr, SHT40_CMD_SOFT_RESET);
        vTaskDelay(pdMS_TO_TICKS(2));
        float t, rh;
        if (sht_measure(addr, &t, &rh) == ESP_OK) {
            s_last.address       = addr;
            s_last.temperature_c = t;
            s_last.humidity_pct  = rh;
            s_last.initialized   = true;
            ESP_LOGI(TAG, "SHT40 OK pod adresem 0x%02X (T=%.1f C, RH=%.1f%%)",
                     addr, (double)t, (double)rh);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "SHT40 nie wykryty (sprawdz adres 0x44 i magistrale I2C)");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t sht40_read(sht40_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;
    float t, rh;
    esp_err_t ret = sht_measure(s_last.address, &t, &rh);
    if (ret != ESP_OK) return ret;
    s_last.temperature_c = t;
    s_last.humidity_pct  = rh;
    if (out) *out = s_last;
    return ESP_OK;
}

sht40_data_t sht40_get_last(void) { return s_last; }
