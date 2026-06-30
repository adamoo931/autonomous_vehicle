#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Buzzer pasywny sterowany tonem przez kanał LEDC (np. tranzystor BC847).
 * Pin oraz timer/kanał LEDC zdefiniowane w config.h. */

void buzzer_init(void);

/* Zagraj ton o zadanej częstotliwości przez zadany czas. Wywołanie jest
 * nieblokujące - ton odtwarza osobne zadanie, funkcja wraca natychmiast.
 * freq_hz w hercach, duration_ms w milisekundach. */
void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);

/* Krótki sygnał testowy (2 kHz, 200 ms). */
void buzzer_beep(void);

/* Natychmiastowe wyciszenie. Przerywa też granie melodii w pętli
 * (buzzer_play_melody z loop=true) - zadanie kończy bieżącą nutę i wraca
 * dopiero przy następnym wywołaniu buzzer_play_*. */
void buzzer_off(void);

/* Pojedyncza nuta melodii: częstotliwość [Hz] i czas trwania [ms]. */
typedef struct {
    uint16_t freq_hz;
    uint16_t dur_ms;
} buzzer_note_t;

/* Odtwarza podaną sekwencję nut w tle (osobne zadanie) - wywołanie
 * nieblokujące. Tablica 'notes' musi żyć przez cały czas odtwarzania
 * (np. być statyczna/globalna). Gdy coś już gra, nowe wywołanie jest
 * ignorowane, żeby dwa zadania nie szarpały tym samym kanałem LEDC.
 * loop=true powtarza sekwencję w nieskończoność z przerwą loop_gap_ms
 * między powtórzeniami, do czasu wywołania buzzer_off(). */
void buzzer_play_melody(const buzzer_note_t *notes, size_t count,
                         bool loop, uint32_t loop_gap_ms);

/* Wbudowany jingle "furgonetki z lodami" (melodia "Turkey in the Straw",
 * domena publiczna). */
void buzzer_play_ice_cream_song(void);
