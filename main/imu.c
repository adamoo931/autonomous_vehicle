#include "imu.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "IMU";

// ── Bank 0 rejestry ──
#define REG_BANK_SEL   0x7F
#define REG_WHO_AM_I   0x00
#define REG_PWR_MGMT1  0x06
#define REG_PWR_MGMT2  0x07
#define REG_ACCEL_XOUT 0x2D   // MSB; LSB = 0x2E ... itd.
#define REG_GYRO_XOUT  0x33
#define REG_TEMP_OUT   0x39

#define ICM_WHO_AM_I_EXPECTED 0xEA

static imu_data_t s_last = {0};

static esp_err_t icm_write(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM20948_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_write_byte(h, val, true);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return ret;
}

static esp_err_t icm_read(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM20948_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (ICM20948_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(h, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(h, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return ret;
}

static void select_bank(uint8_t bank) {
    icm_write(REG_BANK_SEL, (bank & 0x03) << 4);
}

static int16_t to_int16(uint8_t h, uint8_t l) {
    return (int16_t)((uint16_t)h << 8 | l);
}

esp_err_t imu_init(void) {
    select_bank(0);
    uint8_t who = 0;
    esp_err_t ret = icm_read(REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM-20948 brak odpowiedzi (err=%d)", ret);
        s_last.initialized = false;
        return ret;
    }
    if (who != ICM_WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (oczekiwano 0xEA)", who);
        s_last.initialized = false;
        return ESP_ERR_NOT_FOUND;
    }

    // Wybudź układ, wybierz auto-zegar
    icm_write(REG_PWR_MGMT1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Włącz accel + gyro
    icm_write(REG_PWR_MGMT2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    s_last.initialized = true;
    ESP_LOGI(TAG, "ICM-20948 OK (WHO_AM_I=0x%02X)", who);
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;

    select_bank(0);
    uint8_t raw[12];

    // Odczyt accel (6 bajtów) + gyro (6 bajtów) kolejno
    esp_err_t ra = icm_read(REG_ACCEL_XOUT, raw,     6);
    esp_err_t rg = icm_read(REG_GYRO_XOUT,  raw + 6, 6);
    if (ra != ESP_OK || rg != ESP_OK) return ESP_FAIL;

    // Temperatura
    uint8_t tmp[2];
    icm_read(REG_TEMP_OUT, tmp, 2);

    // Skala: accel ±2g (16384 LSB/g), gyro ±250°/s (131 LSB/°/s)
    s_last.accel_x = (float)to_int16(raw[0], raw[1]) / 16384.0f;
    s_last.accel_y = (float)to_int16(raw[2], raw[3]) / 16384.0f;
    s_last.accel_z = (float)to_int16(raw[4], raw[5]) / 16384.0f;
    s_last.gyro_x  = (float)to_int16(raw[6],  raw[7])  / 131.0f;
    s_last.gyro_y  = (float)to_int16(raw[8],  raw[9])  / 131.0f;
    s_last.gyro_z  = (float)to_int16(raw[10], raw[11]) / 131.0f;
    s_last.temp    = (float)to_int16(tmp[0], tmp[1]) / 333.87f + 21.0f;

    if (out) *out = s_last;
    return ESP_OK;
}

imu_data_t imu_get_last(void) { return s_last; }
