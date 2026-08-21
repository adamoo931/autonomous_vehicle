#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * ADS1115 — 16-bitowy przetwornik ADC (I2C), tu używany do odczytu napięcia
 * z liniowego (ratiometrycznego) czujnika Halla SS495A podłączonego do
 * kanału A0 (pojedynczo względem GND) — zastępuje poprzedni cyfrowy czujnik
 * Halla mety.
 *
 * SS495A przy braku pola magnetycznego siedzi na napięciu spoczynkowym
 * (zmierzone na docelowym sprzęcie: ok. 2,488V) i odchyla się w górę lub w
 * dół w zależności od bieguna zbliżonego magnesu. Metę wykrywamy więc jako
 * wyjście napięcia poza pasmo HALL_FINISH_LOW_V..HALL_FINISH_HIGH_V (w
 * config.h) — granice trzeba dostroić eksperymentalnie na docelowym
 * sprzęcie, tak jak PYROMETER_FINISH_THRESHOLD_C dla pirometru.
 *
 * ADDR podpięty do GND -> adres I2C 0x48 (ADS1115_ADDR w config.h).
 */

typedef struct {
    float    voltage_v;       /* napięcie na kanale A0 [V] */
    bool     finish_detected; /* voltage_v <= HALL_FINISH_LOW_V lub >= HALL_FINISH_HIGH_V */
    uint8_t  address;         /* adres I2C układu (0 = niezainicjalizowany) */
    bool     initialized;     /* układ rozpoznany przy starcie */
} ads1115_data_t;

/* Konfiguruje układ i potwierdza jego obecność na magistrali I2C. */
esp_err_t ads1115_init(void);

/* Wykonuje pojedynczy pomiar (single-shot) na kanale A0, przelicza na
 * napięcie i porównuje z progiem detekcji mety. Zapisuje wynik do *out
 * (jeśli != NULL). */
esp_err_t ads1115_read(ads1115_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
ads1115_data_t ads1115_get_last(void);
