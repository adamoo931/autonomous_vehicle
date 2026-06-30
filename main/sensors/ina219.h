#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * INA219 — monitor prądu, napięcia i mocy (I2C).
 *
 * Prąd liczony jest programowo z napięcia na boczniku pomiarowym
 * (INA219_SHUNT_OHMS w config.h), dzięki czemu wynik nie zależy od
 * wewnętrznego rejestru kalibracji układu. Adres I2C jest stały
 * (INA219_ADDR w config.h).
 */

typedef struct {
    float    bus_voltage_v;    /* napięcie szyny [V]                        */
    float    shunt_voltage_mv; /* spadek napięcia na boczniku [mV]          */
    float    current_ma;       /* prąd [mA]                                 */
    float    power_mw;         /* moc [mW]                                  */
    uint8_t  address;          /* adres I2C układu (0 = niezainicjalizowany)*/
    bool     initialized;      /* układ rozpoznany przy starcie             */
} ina219_data_t;

/* Konfiguruje układ i potwierdza jego obecność na magistrali I2C. */
esp_err_t     ina219_init(void);

/* Odczytuje napięcia, przelicza prąd i moc, zapisuje wynik do *out. */
esp_err_t     ina219_read(ina219_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
ina219_data_t ina219_get_last(void);
