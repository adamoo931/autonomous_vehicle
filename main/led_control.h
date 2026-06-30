#pragma once
#include <stdbool.h>

/* Sterowanie trzema diodami sygnalizacyjnymi (czerwona/żółta/zielona).
 * Piny zdefiniowane w config.h. Moduł pamięta ostatni stan każdej diody,
 * dzięki czemu można go odczytać (wykorzystywane przez dashboard). */

void led_init(void);

void led_set_red(bool on);
void led_set_yellow(bool on);
void led_set_green(bool on);
void led_set_all(bool red, bool yellow, bool green);

bool led_get_red(void);
bool led_get_yellow(void);
bool led_get_green(void);
