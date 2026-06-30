#pragma once
#include <esp_err.h>
#include <stdbool.h>

/*
 * ICM-20948 — 9-osiowy układ inercyjny (akcelerometr + żyroskop + magnetometr).
 *
 * Sterownik wykorzystuje akcelerometr, żyroskop, wbudowany czujnik
 * temperatury oraz magnetometr AK09916 (osobny układ za wewnętrznym I2C
 * masterem ICM, dostępny przez przejście SLV0 - rejestry banku 3).
 * Adres I2C jest stały (ICM20948_ADDR w config.h).
 */

typedef struct {
    float accel_x, accel_y, accel_z;   /* przyspieszenie [g]                */
    float gyro_x,  gyro_y,  gyro_z;    /* prędkość kątowa [°/s]             */
    float mag_x,   mag_y,   mag_z;     /* pole magnetyczne [µT], osie zgodne z accel/gyro */
    float azimuth_deg;                 /* azymut (kierunek pojazdu) [0,360°), wg magnetometru */
    float temp;                        /* temperatura układu [°C]           */
    bool  initialized;                 /* akcelerometr/żyroskop rozpoznane przy starcie */
    bool  mag_initialized;             /* magnetometr AK09916 rozpoznany przy starcie   */
} imu_data_t;

/* Sprawdza obecność układu (WHO_AM_I), wybudza akcelerometr + żyroskop
 * i uruchamia magnetometr AK09916 (brak magnetometru nie jest błędem -
 * patrz imu_data_t.mag_initialized). */
esp_err_t imu_init(void);

/* Odczytuje wszystkie osie oraz temperaturę i zapisuje wynik do *out. */
esp_err_t imu_read(imu_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
imu_data_t imu_get_last(void);

/*
 * Kalibracja magnetometru (hard-iron + faza montażu) wykonywana w czasie
 * pracy, bez przebudowy firmware. Wynik zapisywany do NVS - przetrwa
 * restart. Wartości domyślne (wbudowane w imu.c) są nadpisywane wartościami
 * z NVS przy starcie, jeśli kalibracja była już kiedyś wykonana.
 *
 * Procedura: 1) imu_mag_calibration_start() i obrót pojazdu o pełny obrót
 * w czasie trwania kalibracji (zbiera min/max Mx/My do wyliczenia offsetu),
 * 2) ustawienie pojazdu na północ (wg kompasu referencyjnego) i wywołanie
 * imu_mag_set_north() (ustawia fazę tak, by bieżący kierunek czytał się
 * jako 0°).
 */

/* Uruchamia zbieranie min/max Mx/My przez duration_ms; po zakończeniu liczy
 * i zapisuje offset hard-iron. Wywołanie nieblokujące - próbki zbierane są
 * przy okazji normalnych odczytów imu_read() w tle. */
void imu_mag_calibration_start(uint32_t duration_ms);

/* Ustawia fazę tak, by bieżący kierunek pojazdu odczytywał się jako 0° (N).
 * Wykonaj trzymając pojazd skierowany na północ wg kompasu referencyjnego. */
void imu_mag_set_north(void);

typedef struct {
    bool     active;          /* trwa zbieranie min/max (krok 1)          */
    uint32_t remaining_ms;    /* pozostały czas zbierania (krok 1)        */
    float    offset_x, offset_y;  /* µT, aktualny offset hard-iron        */
    float    phase_deg;           /* aktualna faza/orientacja montażu     */
} imu_mag_cal_status_t;

imu_mag_cal_status_t imu_mag_get_cal_status(void);
