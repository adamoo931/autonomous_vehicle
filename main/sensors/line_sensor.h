#pragma once
#include <stdbool.h>

/*
 * Czujniki linii CNY70 — cztery czujniki odbiciowe (po jednym w każdym rogu).
 *
 * Wartość true oznacza wykrycie krawędzi planszy (brak odbicia / ciemne
 * podłoże). Sterownik czyta stan czujników bezpośrednio, bez buforowania.
 */

typedef struct {
    bool front_left;
    bool front_right;
    bool back_left;
    bool back_right;
} line_sensor_data_t;

/* Konfiguruje piny GPIO czujników. */
void line_sensor_init(void);

/* Zwraca bieżący stan wszystkich czterech czujników. */
line_sensor_data_t line_sensor_read(void);

/* Zwraca true, jeśli którykolwiek czujnik sygnalizuje krawędź. */
bool line_sensor_any_edge(void);
