#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * SHT40 — czujnik temperatury i wilgotności względnej (I2C).
 *
 * Pomiar wykonywany jest w trybie wysokiej precyzji, a poprawność transmisji
 * potwierdzana sumą kontrolną CRC8 zgodną z dokumentacją Sensirion. Adres I2C
 * jest stały (SHT40_ADDR w config.h).
 */

typedef struct {
    float    temperature_c;   /* temperatura [°C]                          */
    float    humidity_pct;    /* wilgotność względna [% RH]                */
    uint8_t  address;         /* adres I2C układu (0 = niezainicjalizowany)*/
    bool     initialized;     /* układ rozpoznany przy starcie             */
} sht40_data_t;

/* Resetuje czujnik i potwierdza jego obecność wykonując pomiar próbny. */
esp_err_t    sht40_init(void);

/* Wykonuje pomiar temperatury i wilgotności, zapisuje wynik do *out. */
esp_err_t    sht40_read(sht40_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
sht40_data_t sht40_get_last(void);
