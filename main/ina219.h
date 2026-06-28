#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

// ============================================================
//  INA219 – monitor pradu/napiecia (I2C)
//  Bocznik R13 = 0.05 Ohm (patrz config.h INA219_SHUNT_OHMS)
//  Adres ustalany automatycznie (patrz INA219_ADDR + lista
//  kandydatow w ina219.c).
// ============================================================

typedef struct {
    float    bus_voltage_v;   // napiecie szyny [V]
    float    shunt_voltage_mv;// spadek na boczniku [mV]
    float    current_ma;      // prad [mA]
    float    power_mw;        // moc [mW]
    uint8_t  address;         // wykryty adres I2C (0 = brak)
    bool     initialized;
} ina219_data_t;

esp_err_t     ina219_init(void);
esp_err_t     ina219_read(ina219_data_t *out);
ina219_data_t ina219_get_last(void);
