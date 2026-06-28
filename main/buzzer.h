#pragma once
#include <stdint.h>

// ============================================================
//  Buzzer pasywny (tranzystor BC847, sterowanie tonem LEDC)
//  Pin: PIN_BUZZER (config.h) – ZWERYFIKUJ z wlasnym schematem!
// ============================================================

void buzzer_init(void);

// Zagraj ton o danej czestotliwosci przez dany czas (NIEBLOKUJACE –
// odtwarzane w tle, funkcja wraca natychmiast). freq w Hz, czas w ms.
void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);

// Krotki sygnal testowy (2 kHz, 200 ms).
void buzzer_beep(void);

// Natychmiastowe wyciszenie.
void buzzer_off(void);
