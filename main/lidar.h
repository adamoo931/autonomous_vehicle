#pragma once
#include <stdint.h>
#include <stdbool.h>

// Implementacja dla LD06 / LD19 (pakiet 47 bajtów, baud 230400).
// Dla innych modeli (RPLIDAR, TFMini, etc.) dostosuj lidar.c.

#define LIDAR_MAX_POINTS 12  // punkty w jednym pakiecie LD06

typedef struct {
    uint16_t distance_mm;
    uint16_t angle_hundredths;  // kąt x100 [0..35999]
    uint8_t  intensity;
} lidar_point_t;

typedef struct {
    lidar_point_t points[LIDAR_MAX_POINTS];
    uint8_t  count;
    uint16_t speed_rpm;
    bool     valid;
} lidar_packet_t;

void lidar_init(void);
void lidar_stop(void);

/**
 * Zwraca ostatni poprawnie odebrany pakiet.
 * Wywoływana z dowolnego task-a.
 */
lidar_packet_t lidar_get_last_packet(void);

/**
 * Zwraca minimalną odległość z ostatniego pakietu [mm].
 * 0 = brak danych.
 */
uint16_t lidar_get_min_distance_mm(void);
