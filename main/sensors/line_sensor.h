#pragma once
#include <stdbool.h>

/*
 * Czujniki linii CNY70 — cztery czujniki odbiciowe (po jednym w każdym rogu),
 * WSZYSTKIE odczytywane ANALOGOWO przez ADS1115:
 *
 *   A0 — przód-prawy   A1 — przód-lewy   A2 — tył-lewy   A3 — tył-prawy
 *
 * Linia wykryta = napięcie PONIŻEJ progu danego kanału
 * (LINE_FR/FL/BL/BR_THRESHOLD_V w config.h). Pole *_v podaje ostatnio
 * zmierzone napięcie kanału (0, gdy ADS1115 nie został wykryty).
 */

typedef struct {
    bool  front_left;      /* A1: napięcie < LINE_FL_THRESHOLD_V */
    bool  front_right;     /* A0: napięcie < LINE_FR_THRESHOLD_V */
    bool  back_left;       /* A2: napięcie < LINE_BL_THRESHOLD_V */
    bool  back_right;      /* A3: napięcie < LINE_BR_THRESHOLD_V */
    float front_left_v;    /* napięcie kanału A1 [V] */
    float front_right_v;   /* napięcie kanału A0 [V] */
    float back_left_v;     /* napięcie kanału A2 [V] */
    float back_right_v;    /* napięcie kanału A3 [V] */
} line_sensor_data_t;

/* Zostawione dla zgodności wywołań (main.c) — wszystkie kanały idą teraz
 * przez ADS1115, więc funkcja tylko loguje konfigurację. */
void line_sensor_init(void);

/* Zwraca bieżący stan wszystkich czterech czujników. Wartości pochodzą z
 * ostatniego cyklu odczytu ADS1115 (ads1115_get_last), więc wywołanie jest
 * nieblokujące. */
line_sensor_data_t line_sensor_read(void);

/* Zwraca true, jeśli którykolwiek czujnik sygnalizuje krawędź. */
bool line_sensor_any_edge(void);
