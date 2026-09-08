#include "ads1115.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ADS1115";

/* Mapa rejestrów ADS1115. */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/* Konfiguracja: single-shot start (OS=1), pojedynczy kanał względem GND
 * (MUX w bitach [14:12]), wzmocnienie PGA=+-4,096V (FSR poniżej), tryb
 * single-shot (MODE=1), 128 próbek/s (DR=100), komparator wyłączony
 * (COMP_QUE=11). Kolejne kanały różnią się tylko polem MUX:
 *   A0 -> 0xC383, A1 -> 0xD383, A2 -> 0xE383, A3 -> 0xF383.
 * Zmień PGA (patrz datasheet ADS1115), jeśli któreś napięcie przy realnym
 * zasilaniu czujnika wychodzi poza zakres +-4,096V. */
#define ADS1115_CONFIG_AIN0     0xC383
#define ADS1115_CONFIG_AIN1     0xD383
#define ADS1115_CONFIG_AIN2     0xE383
#define ADS1115_CONFIG_AIN3     0xF383
#define ADS1115_FSR_V           4.096f
#define ADS1115_CONV_DELAY_MS   10      /* > 1/128 s przy 128 probek/s, z zapasem */

static ads1115_data_t s_last = {0};

/* Zapis rejestru 16-bitowego w kolejności big-endian (jak INA219). */
static esp_err_t ads_write16(uint8_t reg, uint16_t val) {
    uint8_t data[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(h, data, sizeof(data), true);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    return ret;
}

/* Odczyt rejestru 16-bitowego w kolejności big-endian. */
static esp_err_t ads_read16(uint8_t reg, uint16_t *out) {
    uint8_t buf[2] = {0};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (ADS1115_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(h, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(h, &buf[1], I2C_MASTER_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    if (ret == ESP_OK) *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return ret;
}

esp_err_t ads1115_init(void) {
    s_last.initialized = false;
    s_last.address     = 0;

    /* Potwierdza obecność: wyzwala pomiar na A0 i sprawdza, czy układ w ogóle
     * odpowiada (zapis konfiguracji + odczyt rejestru konwersji). */
    if (ads_write16(ADS1115_REG_CONFIG, ADS1115_CONFIG_AIN0) != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 nie odpowiada pod 0x%02X", ADS1115_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    vTaskDelay(pdMS_TO_TICKS(ADS1115_CONV_DELAY_MS));
    uint16_t raw = 0;
    if (ads_read16(ADS1115_REG_CONVERSION, &raw) != ESP_OK) {
        ESP_LOGE(TAG, "ADS1115 nie odpowiada pod 0x%02X", ADS1115_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    s_last.address     = ADS1115_ADDR;
    s_last.initialized = true;
    ESP_LOGI(TAG, "ADS1115 OK pod adresem 0x%02X (A0=linia przod-P, A1..A3=linia przod-L/tyl-L/tyl-P, FSR=+-%.3fV)",
             ADS1115_ADDR, (double)ADS1115_FSR_V);
    return ESP_OK;
}

/* Wyzwala pojedynczy pomiar (single-shot) na zadanym kanale, czeka na jego
 * zakończenie i przelicza wynik na napięcie [V]. ADS1115 zasypia między
 * pomiarami w tym trybie, więc każdy odczyt wymaga świeżego startu. */
static esp_err_t ads_measure(uint16_t cfg, float *out_v) {
    if (ads_write16(ADS1115_REG_CONFIG, cfg) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(ADS1115_CONV_DELAY_MS));

    uint16_t raw = 0;
    if (ads_read16(ADS1115_REG_CONVERSION, &raw) != ESP_OK) return ESP_FAIL;

    /* Wynik to liczba ze znakiem (U2), waga LSB = FSR / 32768. */
    int16_t signed_raw = (int16_t)raw;
    *out_v = (float)signed_raw * (ADS1115_FSR_V / 32768.0f);
    return ESP_OK;
}

esp_err_t ads1115_read(ads1115_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;

    /* Wszystkie cztery kanały to analogowe czujniki odbiciowe linii
     * (best-effort: pojedynczy błąd zostawia poprzednią wartość kanału i nie
     * przerywa całego odczytu). A0 = przód-prawy (dawniej Hall mety). */
    float v;
    if (ads_measure(ADS1115_CONFIG_AIN0, &v) == ESP_OK) s_last.line_fr_v = v;
    if (ads_measure(ADS1115_CONFIG_AIN1, &v) == ESP_OK) s_last.line_fl_v = v;
    if (ads_measure(ADS1115_CONFIG_AIN2, &v) == ESP_OK) s_last.line_bl_v = v;
    if (ads_measure(ADS1115_CONFIG_AIN3, &v) == ESP_OK) s_last.line_br_v = v;

    if (out) *out = s_last;
    return ESP_OK;
}

ads1115_data_t ads1115_get_last(void) { return s_last; }
