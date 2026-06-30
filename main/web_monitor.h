#pragma once
#include <stddef.h>

/* Monitor webowy: przechwytuje logi ESP_LOGx do bufora kołowego, który
 * można pobrać przez HTTP (/api/logs) i wyświetlić w przeglądarce. Zastępuje
 * monitor szeregowy, gdy port UART jest zajęty przez LIDAR. */

/* Instaluje hook logowania. Wywołać jak najwcześniej w app_main. */
void web_monitor_init(void);

/* Kopiuje zawartość bufora (od najstarszych) do 'out'. Zwraca liczbę
 * zapisanych bajtów (bez końcowego '\0'). */
size_t web_monitor_dump(char *out, size_t maxlen);

/* Czyści bufor logów. */
void web_monitor_clear(void);
