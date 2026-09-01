#pragma once
#include <stdbool.h>

/* =====================================================================
 *  Test wykrywania linii czujnikami odbiciowymi CNY70 - osobny tryb
 *  uruchamiany z dashboardu, obok autonomii.
 *
 *  Pojazd porusza się skokowo: 0,3 s jazdy z prędkością 25 % mocy, potem
 *  0,3 s postoju - i tak w kółko. Kierunek (do przodu / do tyłu) wybiera
 *  się na dashboardzie. Gdy którykolwiek z czterech czujników wykryje
 *  linię/krawędź, pojazd zatrzymuje się całkowicie, test się kończy, a do
 *  logu trafia informacja, który czujnik zadziałał. Stan
 *  "Zatrzymany (linia: ...)" pozostaje widoczny na dashboardzie do czasu
 *  ponownego uruchomienia testu.
 *
 *  Test i autonomia wykluczają się wzajemnie; każda ręczna komenda
 *  silników oraz STOP również przerywają test - patrz http_server.c.
 * ===================================================================== */

/* Tworzy zadanie testu (startuje wyłączone). */
void line_test_init(void);

/* Uruchamia test. forward=true -> jazda do przodu, false -> do tyłu. */
void line_test_start(bool forward);

/* Zatrzymuje test i silniki (ręczne przerwanie). */
void line_test_stop(void);

/* Czy test jest aktualnie aktywny. */
bool line_test_is_running(void);

/* Ostatnio wybrany kierunek jazdy (true = do przodu). */
bool line_test_get_forward(void);

/* Krótki opis stanu dla dashboardu ("Bezczynny", "Jazda do przodu",
 * "Jazda do tylu", "Zatrzymany (linia)"). */
const char *line_test_state_str(void);

/* Lista czujników, które zatrzymały ostatni test (np. "przod-L tyl-P"),
 * pusty napis, jeśli test jeszcze nie został zatrzymany linią. */
const char *line_test_hit_str(void);
