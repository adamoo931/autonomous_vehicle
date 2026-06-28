#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

// ============================================================
//  SHT40 – czujnik temperatury i wilgotnosci (I2C, adres 0x44)
// ============================================================

typedef struct {
    float    temperature_c;   // [°C]
    float    humidity_pct;    // [% RH]
    uint8_t  address;         // wykryty adres I2C (0 = brak)
    bool     initialized;
} sht40_data_t;

esp_err_t    sht40_init(void);
esp_err_t    sht40_read(sht40_data_t *out);
sht40_data_t sht40_get_last(void);
