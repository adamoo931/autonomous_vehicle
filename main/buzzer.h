#pragma once
#include <stdint.h>

/* Buzzer pasywny sterowany tonem przez kanał LEDC (np. tranzystor BC847).
 * Pin oraz timer/kanał LEDC zdefiniowane w config.h. */

void buzzer_init(void);

/* Zagraj ton o zadanej częstotliwości przez zadany czas. Wywołanie jest
 * nieblokujące - ton odtwarza osobne zadanie, funkcja wraca natychmiast.
 * freq_hz w hercach, duration_ms w milisekundach. */
void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);

/* Krótki sygnał testowy (2 kHz, 200 ms). */
void buzzer_beep(void);

/* Natychmiastowe wyciszenie. */
void buzzer_off(void);
