#pragma once
#include <stdbool.h>

/*
 * Czujniki linii CNY70 — cztery czujniki odbiciowe (po jednym w każdym rogu).
 *
 * Przód-lewy, tył-lewy i tył-prawy są odczytywane ANALOGOWO przez ADS1115
 * (kanały A1/A2/A3 — patrz sensors/ads1115.c). Linia wykryta = napięcie
 * PONIŻEJ progu danego kanału (LINE_FL/BL/BR_THRESHOLD_V w config.h).
 * Przód-prawy pozostał cyfrowy na GPIO (PIN_LINE_FR).
 *
 * Pole *_v podaje ostatnio zmierzone napięcie danego kanału ADC (0, gdy
 * ADS1115 nie został wykryty). Dla przodu-prawego napięcia nie ma.
 */

typedef struct {
    bool  front_left;      /* A1: napięcie < LINE_FL_THRESHOLD_V */
    bool  front_right;     /* GPIO PIN_LINE_FR == 1 */
    bool  back_left;       /* A2: napięcie < LINE_BL_THRESHOLD_V */
    bool  back_right;      /* A3: napięcie < LINE_BR_THRESHOLD_V */
    float front_left_v;    /* napięcie kanału A1 [V] */
    float back_left_v;     /* napięcie kanału A2 [V] */
    float back_right_v;    /* napięcie kanału A3 [V] */
} line_sensor_data_t;

/* Konfiguruje pin GPIO czujnika przód-prawy (pozostałe idą przez ADS1115). */
void line_sensor_init(void);

/* Zwraca bieżący stan wszystkich czterech czujników. Wartości analogowe
 * pochodzą z ostatniego cyklu odczytu ADS1115 (ads1115_get_last), więc
 * wywołanie jest nieblokujące. */
line_sensor_data_t line_sensor_read(void);

/* Zwraca true, jeśli którykolwiek czujnik sygnalizuje krawędź. */
bool line_sensor_any_edge(void);
