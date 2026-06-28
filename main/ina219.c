#include "ina219.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INA219";

// ── Rejestry INA219 ──
#define INA_REG_CONFIG      0x00
#define INA_REG_SHUNT_V     0x01
#define INA_REG_BUS_V       0x02
#define INA_REG_POWER       0x03
#define INA_REG_CURRENT     0x04
#define INA_REG_CALIB       0x05

// Konfiguracja: 32V, PGA /8 (+/-320mV), 12-bit, ciagly pomiar shunt+bus.
// To jest takze wartosc domyslna po resecie (0x399F).
#define INA_CONFIG_DEFAULT  0x399F
#define INA_CONFIG_RESET    0x8000

static ina219_data_t s_last = {0};

// Lista adresow do przeszukania. 0x45 = A0,A1 -> VBAT (wg schematu).
// 0x44 nalezy do SHT40, wiec jest na koncu i tak nigdy nie powinien
// zostac wybrany dla INA (weryfikacja przez odczyt rejestru CONFIG).
static const uint8_t s_candidates[] = { INA219_ADDR, 0x40, 0x41, 0x45, 0x44 };

// ── Zapis 16-bit (big-endian) do rejestru ──
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

// ── Odczyt 16-bit (big-endian) z rejestru ──
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

// Sprawdza czy pod danym adresem faktycznie siedzi INA219:
// zapisuje konfiguracje i czyta ja z powrotem.
static bool ina_probe(uint8_t addr) {
    if (ina_write16(addr, INA_REG_CONFIG, INA_CONFIG_DEFAULT) != ESP_OK)
        return false;
    vTaskDelay(pdMS_TO_TICKS(2));
    uint16_t cfg = 0;
    if (ina_read16(addr, INA_REG_CONFIG, &cfg) != ESP_OK)
        return false;
    // Po zapisie powinnismy odczytac dokladnie to co wpisalismy.
    return (cfg == INA_CONFIG_DEFAULT);
}

esp_err_t ina219_init(void) {
    s_last.initialized = false;
    s_last.address     = 0;

    for (size_t i = 0; i < sizeof(s_candidates); i++) {
        uint8_t addr = s_candidates[i];
        if (ina_probe(addr)) {
            s_last.address     = addr;
            s_last.initialized = true;
            ESP_LOGI(TAG, "INA219 OK pod adresem 0x%02X (bocznik=%.3f Ohm)",
                     addr, (double)INA219_SHUNT_OHMS);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "INA219 nie wykryty (sprawdz adres A0/A1 i magistrale I2C)");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ina219_read(ina219_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;

    uint16_t bus_raw = 0, shunt_raw = 0;
    esp_err_t rb = ina_read16(s_last.address, INA_REG_BUS_V,   &bus_raw);
    esp_err_t rs = ina_read16(s_last.address, INA_REG_SHUNT_V, &shunt_raw);
    if (rb != ESP_OK || rs != ESP_OK) return ESP_FAIL;

    // Bus voltage: bity [15:3], LSB = 4 mV
    s_last.bus_voltage_v = (float)(bus_raw >> 3) * 0.004f;

    // Shunt voltage: wartosc ze znakiem, LSB = 10 uV = 0.01 mV
    int16_t shunt_signed   = (int16_t)shunt_raw;
    s_last.shunt_voltage_mv = (float)shunt_signed * 0.01f;

    // Prad liczony programowo: I = U_shunt / R_shunt
    // (mV / Ohm = mA), dzieki czemu nie zalezymy od rejestru kalibracji.
    s_last.current_ma = s_last.shunt_voltage_mv / INA219_SHUNT_OHMS;
    s_last.power_mw   = s_last.bus_voltage_v * s_last.current_ma;

    if (out) *out = s_last;
    return ESP_OK;
}

ina219_data_t ina219_get_last(void) { return s_last; }
