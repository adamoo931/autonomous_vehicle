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

// ============================================================
//  Bufor kołowy pełnego skanu – akumuluje punkty z wielu
//  pakietów, dzięki czemu klient (np. desktopowa apka mapująca)
//  może pobierać strumień punktów (kąt + odległość) i budować
//  z nich mapę przeszkód.
// ============================================================

// Pojemność bufora w punktach (~4 obroty LD06 przy 10 Hz).
#define LIDAR_SCAN_BUFFER 1800

typedef struct {
    uint16_t angle_hundredths;  // kąt x100 [0..35999]
    uint16_t distance_mm;       // odległość [mm], 0 = brak echa
} lidar_scan_point_t;

/**
 * Kopiuje do 'out' punkty o numerze sekwencyjnym większym niż 'since'
 * (najnowsze, maksymalnie 'max_pts'). Mechanizm sekwencyjny pozwala
 * klientowi pobierać tylko NOWE punkty bez duplikatów ani luk
 * (dopóki nie zostanie w tyle o więcej niż LIDAR_SCAN_BUFFER).
 *
 *   since    – ostatni numer sekwencyjny znany klientowi (0 = od początku)
 *   out      – bufor wyjściowy klienta
 *   max_pts  – rozmiar bufora 'out'
 *   out_seq  – [wyj.] bieżący licznik sekwencyjny (przekaż jako 'since'
 *              przy następnym wywołaniu)
 *
 * Zwraca liczbę skopiowanych punktów.
 */
uint16_t lidar_copy_scan(uint32_t since, lidar_scan_point_t *out,
                         uint16_t max_pts, uint32_t *out_seq);

/** Prędkość obrotowa głowicy z ostatniego pakietu [RPM]. */
uint16_t lidar_get_speed_rpm(void);

/**
 * Minimalna odległość [mm] w łuku [center-half .. center+half] stopni
 * (kąt w SUROWYM układzie lidaru, 0..359, z zawijaniem przez 0/360).
 * Pomija kierunki bez echa oraz odczyty starsze niż LIDAR_BIN_MAX_AGE_MS.
 * Zwraca 0, gdy w całym łuku nie ma świeżego echa (kierunek otwarty).
 *
 * Używane przez moduł autonomii (autonomy.c) do wykrywania przeszkód
 * w wybranych łukach wokół robota (przód, przekątne, boki).
 */
uint16_t lidar_min_in_arc(int center_deg, int half_deg);
