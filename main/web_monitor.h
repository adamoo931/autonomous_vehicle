#pragma once
#include <stddef.h>

// ============================================================
//  Web Monitor – przechwytuje logi ESP_LOGx do bufora kolowego,
//  ktory mozna pobrac przez HTTP i wyswietlic w przegladarce
//  (zamiennik monitora szeregowego, gdy UART zajmuje LIDAR).
// ============================================================

// Instaluje hook logowania. Wywolaj jak najwczesniej w app_main.
void web_monitor_init(void);

// Kopiuje zawartosc bufora (od najstarszych) do 'out'.
// Zwraca liczbe zapisanych bajtow (bez koncowego '\0').
size_t web_monitor_dump(char *out, size_t maxlen);

// Czysci bufor logow.
void web_monitor_clear(void);
