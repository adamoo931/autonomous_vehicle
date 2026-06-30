#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * Odometria — pomiar przebytej drogi na podstawie impulsów z czujników
 * Halla obu kół oraz detekcja mety osobnym czujnikiem Halla.
 *
 * Impulsy zliczane są w procedurach obsługi przerwań GPIO; przeliczenie na
 * milimetry odbywa się przy odczycie (stała MM_PER_PULSE z config.h).
 */

typedef struct {
    uint32_t pulses_left;     /* zliczone impulsy lewego koła              */
    uint32_t pulses_right;    /* zliczone impulsy prawego koła             */
    float    dist_left_mm;    /* droga lewego koła [mm]                    */
    float    dist_right_mm;   /* droga prawego koła [mm]                   */
    float    dist_total_mm;   /* średnia droga obu kół [mm]                */
    bool     finish_detected; /* czujnik Halla mety zadziałał              */
} odometry_data_t;

/* Konfiguruje piny i instaluje procedury obsługi przerwań. */
void odometry_init(void);

/* Zeruje liczniki impulsów oraz flagę mety. */
void odometry_reset(void);

/* Zwraca bieżący stan odometrii (liczniki + przeliczone odległości). */
odometry_data_t odometry_get(void);
