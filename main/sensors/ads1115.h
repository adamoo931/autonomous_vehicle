#pragma once
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * ADS1115 — 16-bitowy przetwornik ADC (I2C), tu używany do odczytu napięcia
 * z liniowego (ratiometrycznego) czujnika Halla SS495A podłączonego do
 * kanału A0 (pojedynczo względem GND) — zastępuje poprzedni cyfrowy czujnik
 * Halla mety.
 *
 * SS495A przy braku pola magnetycznego siedzi na napięciu spoczynkowym
 * (HALL_FINISH_REST_V, zmierzone na docelowym sprzęcie: ok. 2,488V) i odchyla
 * się w górę lub w dół w zależności od bieguna zbliżonego magnesu. Metę
 * wykrywamy więc jako odchyłkę napięcia od wartości spoczynkowej o co najmniej
 * próg (w obie strony): |voltage_v - HALL_FINISH_REST_V| >= próg. Próg startuje
 * z HALL_FINISH_THRESHOLD_V (config.h), ale można go zmienić w trakcie pracy
 * przez ads1115_set_finish_threshold() (pole na dashboardzie).
 *
 * Kanały A1/A2/A3 obsługują trzy analogowe czujniki odbiciowe CNY70
 * (przód-lewy, tył-lewy, tył-prawy) — patrz sensors/line_sensor.c. Każdy
 * odczyt ads1115_read() próbkuje po kolei wszystkie cztery kanały.
 *
 * ADDR podpięty do GND -> adres I2C 0x48 (ADS1115_ADDR w config.h).
 */

typedef struct {
    float    voltage_v;       /* napięcie na kanale A0 (Hall mety SS495A) [V] */
    bool     finish_detected; /* |voltage_v - HALL_FINISH_REST_V| >= próg detekcji mety */
    float    line_fl_v;       /* kanał A1 — czujnik odbiciowy przód-lewy [V] */
    float    line_bl_v;       /* kanał A2 — czujnik odbiciowy tył-lewy [V] */
    float    line_br_v;       /* kanał A3 — czujnik odbiciowy tył-prawy [V] */
    uint8_t  address;         /* adres I2C układu (0 = niezainicjalizowany) */
    bool     initialized;     /* układ rozpoznany przy starcie */
} ads1115_data_t;

/* Konfiguruje układ i potwierdza jego obecność na magistrali I2C. */
esp_err_t ads1115_init(void);

/* Wykonuje pojedyncze pomiary (single-shot) kolejno na kanałach A0..A3,
 * przelicza na napięcia i porównuje A0 z progiem detekcji mety. Zapisuje
 * wynik do *out (jeśli != NULL). Zwraca błąd, gdy nie uda się odczytać
 * kanału A0 (Hall mety); pojedyncze błędy A1..A3 pozostawiają poprzednią
 * wartość danego kanału. */
esp_err_t ads1115_read(ads1115_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
ads1115_data_t ads1115_get_last(void);

/* Ustawia/odczytuje próg detekcji mety - maksymalną dopuszczalną odchyłkę
 * napięcia SS495A od wartości spoczynkowej (ads1115_get_finish_rest_v()),
 * przy której meldowana jest meta. Wartość w woltach, ograniczana do
 * sensownego zakresu. Nie jest zapisywana w NVS - po restarcie wraca
 * HALL_FINISH_THRESHOLD_V. */
void  ads1115_set_finish_threshold(float volts);
float ads1115_get_finish_threshold(void);

/* Ustawia/odczytuje napięcie spoczynkowe SS495A (bez magnesu w pobliżu) -
 * punkt odniesienia dla wykrycia mety. Rzeczywiste napięcie spoczynkowe
 * różni się między uruchomieniami ESP32 (obserwowane: ~2,44-2,49 V), więc
 * stała HALL_FINISH_REST_V w config.h to tylko wartość startowa po boocie -
 * kalibrowana w praktyce raz na przejazd przez autonomy.c (razem z bias
 * żyroskopu, na początku przejazdu, gdy pojazd stoi z dala od magnesu mety).
 * Przycinana do ±0,5 V wokół HALL_FINISH_REST_V. Nie zapisywana w NVS. */
void  ads1115_set_finish_rest_v(float volts);
float ads1115_get_finish_rest_v(void);
