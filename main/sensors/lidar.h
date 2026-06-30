#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * Sterownik lidaru 2D LD06 / LD14P (UART, 230400 bps, pakiet 47 bajtów).
 *
 * Zadanie odbiorcze parsuje strumień pakietów, weryfikuje sumę kontrolną
 * i udostępnia dane na trzy sposoby:
 *   - ostatni pakiet (lidar_get_last_packet),
 *   - bufor kołowy całego skanu z numeracją sekwencyjną (lidar_copy_scan)
 *     do strumieniowania punktów na zewnątrz (np. aplikacja mapująca),
 *   - mapę biegunową 360° z najświeższą odległością na każdy stopień
 *     (lidar_min_in_arc) używaną przez moduł autonomii.
 * Inne modele lidarów wymagają dostosowania parsera w lidar.c.
 */

/* Liczba punktów pomiarowych w jednym pakiecie LD06. */
#define LIDAR_MAX_POINTS 12

typedef struct {
    uint16_t distance_mm;
    uint16_t angle_hundredths;  /* kąt w setnych stopnia [0..35999]         */
    uint8_t  intensity;
} lidar_point_t;

typedef struct {
    lidar_point_t points[LIDAR_MAX_POINTS];
    uint8_t  count;
    uint16_t speed_rpm;
    bool     valid;
} lidar_packet_t;

/* Inicjalizuje UART lidaru i uruchamia zadanie odbiorcze. */
void lidar_init(void);

/* Zatrzymuje zadanie odbiorcze i zwalnia sterownik UART. */
void lidar_stop(void);

/* Zwraca ostatni poprawnie odebrany pakiet (wywoływalne z dowolnego zadania). */
lidar_packet_t lidar_get_last_packet(void);

/* Najmniejsza odległość z ostatniego pakietu [mm]; 0 = brak danych. */
uint16_t lidar_get_min_distance_mm(void);

/* ============================================================
 *  Bufor kołowy pełnego skanu
 *  ----------------------------------------------------------
 *  Akumuluje punkty (kąt + odległość) z wielu pakietów. Każdy punkt ma
 *  numer sekwencyjny, dzięki czemu klient pobiera tylko nowe punkty —
 *  bez duplikatów i bez luk (dopóki nie zostanie w tyle o więcej niż
 *  pojemność bufora).
 * ============================================================ */

/* Pojemność bufora w punktach (kilka obrotów LD06 przy ~10 Hz). */
#define LIDAR_SCAN_BUFFER 1800

typedef struct {
    uint16_t angle_hundredths;  /* kąt w setnych stopnia [0..35999]         */
    uint16_t distance_mm;       /* odległość [mm], 0 = brak echa             */
} lidar_scan_point_t;

/*
 * Kopiuje do 'out' punkty o numerze sekwencyjnym większym niż 'since'
 * (najnowsze, maksymalnie 'max_pts').
 *   since   – ostatni numer sekwencyjny znany klientowi (0 = od początku)
 *   out     – bufor wyjściowy klienta
 *   max_pts – pojemność bufora 'out'
 *   out_seq – [wyj.] bieżący licznik sekwencyjny (przekaż jako 'since'
 *             przy kolejnym wywołaniu)
 * Zwraca liczbę skopiowanych punktów.
 */
uint16_t lidar_copy_scan(uint32_t since, lidar_scan_point_t *out,
                         uint16_t max_pts, uint32_t *out_seq);

/* Prędkość obrotowa głowicy z ostatniego pakietu [RPM]. */
uint16_t lidar_get_speed_rpm(void);

/*
 * Najmniejsza odległość [mm] w łuku [center-half .. center+half] stopni
 * (kąt w surowym układzie lidaru 0..359, z zawijaniem przez 0/360).
 * Pomija kierunki bez echa oraz odczyty starsze niż LIDAR_BIN_MAX_AGE_MS.
 * Zwraca 0, gdy w całym łuku brak świeżego echa (kierunek otwarty).
 * Używane przez moduł autonomii do oceny przeszkód w wybranych łukach.
 */
uint16_t lidar_min_in_arc(int center_deg, int half_deg);
