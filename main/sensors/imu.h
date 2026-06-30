#pragma once
#include <esp_err.h>
#include <stdbool.h>

/*
 * ICM-20948 — 9-osiowy układ inercyjny (akcelerometr + żyroskop + magnetometr).
 *
 * Sterownik wykorzystuje akcelerometr, żyroskop i wbudowany czujnik
 * temperatury. Magnetometr (osobny układ AK09916 za interfejsem I2C master
 * ICM) nie jest obsługiwany. Adres I2C jest stały (ICM20948_ADDR w config.h).
 */

typedef struct {
    float accel_x, accel_y, accel_z;   /* przyspieszenie [g]                */
    float gyro_x,  gyro_y,  gyro_z;    /* prędkość kątowa [°/s]             */
    float temp;                        /* temperatura układu [°C]           */
    bool  initialized;                 /* układ rozpoznany przy starcie     */
} imu_data_t;

/* Sprawdza obecność układu (WHO_AM_I) i wybudza akcelerometr + żyroskop. */
esp_err_t imu_init(void);

/* Odczytuje wszystkie osie oraz temperaturę i zapisuje wynik do *out. */
esp_err_t imu_read(imu_data_t *out);

/* Zwraca ostatni odczyt bez ponownej transakcji I2C. */
imu_data_t imu_get_last(void);
