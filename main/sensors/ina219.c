#include "ina219.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INA219";

/* Mapa rejestrów INA219. */
#define INA_REG_CONFIG      0x00
#define INA_REG_SHUNT_V     0x01
#define INA_REG_BUS_V       0x02
#define INA_REG_POWER       0x03
#define INA_REG_CURRENT     0x04
#define INA_REG_CALIB       0x05

/* Konfiguracja: zakres 32 V, PGA /8 (±320 mV), 12-bit, ciągły pomiar
 * bocznika i szyny. Jest to zarazem domyślna wartość rejestru po resecie,
 * dlatego służy też do weryfikacji obecności układu. */
#define INA_CONFIG_DEFAULT  0x399F
#define INA_CONFIG_RESET    0x8000

static ina219_data_t s_last = {0};

/* Zapis rejestru 16-bitowego w kolejności big-endian. */
static esp_err_t ina_write16(uint8_t addr, uint8_t reg, uint16_t val) {
    uint8_t data[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(h, data, sizeof(data), true);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    return ret;
}

/* Odczyt rejestru 16-bitowego w kolejności big-endian. */
static esp_err_t ina_read16(uint8_t addr, uint8_t reg, uint16_t *out) {
    uint8_t buf[2] = {0};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(h, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(h, &buf[1], I2C_MASTER_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    if (ret == ESP_OK) *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return ret;
}

/* Potwierdza, że pod adresem znajduje się INA219: zapisuje rejestr
 * konfiguracji i sprawdza, czy odczyt zwraca tę samą wartość. */
static bool ina_probe(uint8_t addr) {
    if (ina_write16(addr, INA_REG_CONFIG, INA_CONFIG_DEFAULT) != ESP_OK)
        return false;
    vTaskDelay(pdMS_TO_TICKS(2));
    uint16_t cfg = 0;
    if (ina_read16(addr, INA_REG_CONFIG, &cfg) != ESP_OK)
        return false;
    return (cfg == INA_CONFIG_DEFAULT);
}

esp_err_t ina219_init(void) {
    s_last.initialized = false;
    s_last.address     = 0;

    if (ina_probe(INA219_ADDR)) {
        s_last.address     = INA219_ADDR;
        s_last.initialized = true;
        ESP_LOGI(TAG, "INA219 OK pod adresem 0x%02X (bocznik=%.3f Ohm)",
                 INA219_ADDR, (double)INA219_SHUNT_OHMS);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "INA219 nie wykryty pod 0x%02X (sprawdz magistrale I2C)", INA219_ADDR);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ina219_read(ina219_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;

    uint16_t bus_raw = 0, shunt_raw = 0;
    esp_err_t rb = ina_read16(s_last.address, INA_REG_BUS_V,   &bus_raw);
    esp_err_t rs = ina_read16(s_last.address, INA_REG_SHUNT_V, &shunt_raw);
    if (rb != ESP_OK || rs != ESP_OK) return ESP_FAIL;

    /* Napięcie szyny: bity [15:3], waga LSB = 4 mV. */
    s_last.bus_voltage_v = (float)(bus_raw >> 3) * 0.004f;

    /* Napięcie bocznika: liczba ze znakiem, waga LSB = 10 µV = 0,01 mV. */
    int16_t shunt_signed    = (int16_t)shunt_raw;
    s_last.shunt_voltage_mv = (float)shunt_signed * 0.01f;

    /* Prąd z prawa Ohma: I = U_bocznika / R_bocznika (mV / Ω = mA). */
    s_last.current_ma = s_last.shunt_voltage_mv / INA219_SHUNT_OHMS;
    s_last.power_mw   = s_last.bus_voltage_v * s_last.current_ma;

    if (out) *out = s_last;
    return ESP_OK;
}

ina219_data_t ina219_get_last(void) { return s_last; }
