#pragma once
#include <esp_err.h>
#include <stdbool.h>

/*
 * MLX90614 — bezkontaktowy pirometr (termometr na podczerwień, SMBus).
 *
 * Mierzy temperaturę otoczenia oraz temperaturę obiektu w polu widzenia.
 * Wykrycie obiektu cieplejszego niż PYROMETER_FINISH_THRESHOLD_C jest
 * traktowane jako osiągnięcie mety.
 *
 * Osobno: tryb "szukania obiektu cieplnego" - włączany po wykryciu mety
 * przez czujnik Halla (main.c), a nie przez sam pirometr. W tym trybie
 * pyrometer_read() na bieżąco sprawdza różnicę temperatury obiekt-otoczenie
 * względem PYROMETER_HOT_DELTA_C i ustawia hot_detected, gdy próg zostanie
 * przekroczony. To osobny mechanizm od finish_detected (który patrzy na
 * temperaturę bezwzględną i zasila obecną logikę autonomii).
 */

typedef struct {
    float ambient_temp;     /* temperatura otoczenia [°C]                   */
    float object_temp;      /* temperatura obiektu  [°C]                    */
    bool  finish_detected;  /* obiekt przekroczył próg temperatury mety     */
    bool  initialized;      /* czujnik odpowiedział przy inicjalizacji      */
    bool  searching;        /* tryb szukania obiektu cieplnego aktywny      */
    bool  hot_detected;     /* obiekt cieplny wykryty (różnica >= progu)    */
} pyrometer_data_t;

/* Inicjalizuje pirometr i sprawdza jego obecność na magistrali I2C. */
esp_err_t pyrometer_init(void);

/* Wykonuje pomiar (otoczenie + obiekt) i zapisuje wynik do *out. */
esp_err_t pyrometer_read(pyrometer_data_t *out);

/* Zwraca ostatni odczyt bez ponownego pomiaru (np. dla serwera HTTP). */
pyrometer_data_t pyrometer_get_last(void);

/* Włącza tryb szukania obiektu cieplnego (wywołaj po wykryciu mety Hallem)
 * i czyści poprzednią flagę wykrycia. */
void pyrometer_start_search(void);

/* Wyłącza tryb szukania i czyści flagę wykrycia (np. reset przed nowym
 * przejazdem). */
void pyrometer_reset_search(void);
