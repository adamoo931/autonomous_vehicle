#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * ADS1115 — 16-bitowy przetwornik ADC (I2C), używany do odczytu czterech
 * analogowych czujników odbiciowych CNY70 (po jednym w każdym rogu):
 *
 *   A0 — przód-prawy   (dawniej czujnik Halla mety; ten przeszedł na osobne
 *                        wyjście cyfrowe — patrz sensors/hall_finish.c)
 *   A1 — przód-lewy
 *   A2 — tył-lewy
 *   A3 — tył-prawy
 *
 * Każdy odczyt ads1115_read() próbkuje po kolei wszystkie cztery kanały
 * (single-shot względem GND). Linia wykryta = napięcie PONIŻEJ progu danego
 * kanału (LINE_*_THRESHOLD_V w config.h) — logika w sensors/line_sensor.c.
 *
 * ADDR podpięty do GND -> adres I2C 0x48 (ADS1115_ADDR w config.h).
 */

typedef struct {
    float    line_fr_v;    /* kanał A0 — czujnik odbiciowy przód-prawy [V] */
    float    line_fl_v;    /* kanał A1 — przód-lewy [V] */
    float    line_bl_v;    /* kanał A2 — tył-lewy [V]   */
    float    line_br_v;    /* kanał A3 — tył-prawy [V]  */
    uint8_t  address;      /* adres I2C układu (0 = niezainicjalizowany) */
    bool     initialized;  /* układ rozpoznany przy starcie */
} ads1115_data_t;

/* Konfiguruje układ i potwierdza jego obecność na magistrali I2C. */
esp_err_t ads1115_init(void);

/* Wykonuje pojedyncze pomiary (single-shot) kolejno na kanałach A0..A3 i
 * przelicza je na napięcia. Zapisuje wynik do *out (jeśli != NULL).
 * Pojedynczy błąd odczytu kanału pozostawia jego poprzednią wartość i nie
 * przerywa całego cyklu; funkcja zwraca błąd tylko gdy układ nie został
 * zainicjalizowany. */
esp_err_t ads1115_read(ads1115_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
ads1115_data_t ads1115_get_last(void);
