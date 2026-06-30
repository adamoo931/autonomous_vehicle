#pragma once
#include <esp_err.h>
#include <stdbool.h>

/*
 * MLX90614 — bezkontaktowy pirometr (termometr na podczerwień, SMBus).
 *
 * Mierzy temperaturę otoczenia oraz temperaturę obiektu w polu widzenia.
 * Wykrycie obiektu cieplejszego niż PYROMETER_FINISH_THRESHOLD_C jest
 * traktowane jako osiągnięcie mety.
 */

typedef struct {
    float ambient_temp;     /* temperatura otoczenia [°C]                   */
    float object_temp;      /* temperatura obiektu  [°C]                    */
    bool  finish_detected;  /* obiekt przekroczył próg temperatury mety     */
    bool  initialized;      /* czujnik odpowiedział przy inicjalizacji      */
} pyrometer_data_t;

/* Inicjalizuje pirometr i sprawdza jego obecność na magistrali I2C. */
esp_err_t pyrometer_init(void);

/* Wykonuje pomiar (otoczenie + obiekt) i zapisuje wynik do *out. */
esp_err_t pyrometer_read(pyrometer_data_t *out);

/* Zwraca ostatni odczyt bez ponownego pomiaru (np. dla serwera HTTP). */
pyrometer_data_t pyrometer_get_last(void);
