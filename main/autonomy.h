#pragma once
#include <stdbool.h>
#include <stdint.h>

// ============================================================
//  Autonomia – reaktywne omijanie przeszkód + dojazd do celu
//  termicznego (gorący czajnik wykrywany pirometrem MLX90614).
//
//  Algorytm jest UNIWERSALNY (nie zna kształtu planszy):
//    • lidar  -> przeszkody/ściany na wysokości robota,
//    • odometria -> wykrywanie utknięcia (brak postępu),
//    • pirometr -> cel (obiekt o X st. cieplejszy niż otoczenie) => META.
//
//  Wykrywanie krawędzi obszaru (czujniki linii CNY70) jest na razie
//  WYŁĄCZONE z tej wersji algorytmu na życzenie – do ew. dodania
//  później, gdy będzie potrzebne.
//
//  Uruchamiane przyciskiem „Symuluj autonomię” w dashboardzie
//  (POST /api/autonomy {"enable":true/false}). Każda ręczna
//  komenda silników natychmiast wyłącza autonomię (kill-switch).
//
//  LOG PRZEJAZDU
//  ----------------------------------------------------------
//  Co STATUS_LOG_MS (autonomy.c) zapisywany jest w pamięci RAM
//  kompaktowy rekord (stan, moc silników, odległości lidaru,
//  temperatury). Bufor zeruje się na starcie KAŻDEGO przejazdu
//  (autonomy_set_enabled(true)) i jest dostępny do pobrania jako
//  CSV pod adresem GET /api/autonomy/log.csv – dopóki ESP32 nie
//  zostanie zresetowany lub nie zacznie się nowy przejazd.
//  Pobierz log od razu po zakończeniu jazdy, zanim wystartujesz
//  kolejną – inaczej zostanie nadpisany.
// ============================================================

// Tworzy task autonomii (startuje w stanie bezczynnym).
void autonomy_init(void);

// Włącza / wyłącza tryb autonomiczny.
void autonomy_set_enabled(bool enable);

// Czy autonomia jest aktualnie aktywna.
bool autonomy_is_enabled(void);

// Krótki opis bieżącego stanu (do dashboardu), np. "Jazda", "Omijanie",
// "META osiągnięta", "Awaria (utknięcie)".
const char *autonomy_state_str(void);

// ============================================================
//  LOG PRZEJAZDU (do pobrania jako CSV przez http_server.c)
// ============================================================

// Jeden próbkowany rekord (co STATUS_LOG_MS) z przejazdu.
// Odległości w mm; D_OPEN_MM (z autonomy.c) = "brak echa / otwarte".
// Temperatury *10 (np. 235 = 23.5°C) – bez floatów, by zmniejszyć
// rozmiar rekordu (więcej próbek mieści się w buforze RAM).
typedef struct {
    uint32_t t_ms;          // czas od startu przejazdu [ms]
    int16_t  front_mm;
    int16_t  diag_l_mm;
    int16_t  diag_r_mm;
    int16_t  side_l_mm;
    int16_t  side_r_mm;
    int8_t   motor_l;       // -100..100 [%]
    int8_t   motor_r;
    int16_t  obj_temp_x10;
    int16_t  amb_temp_x10;
    int16_t  best_open_deg; // kąt najbardziej otwartego kierunku DOOKOŁA [st], + = lewo
    uint8_t  state;         // wartość enum stanu – użyj autonomy_log_state_name()
} autonomy_log_rec_t;

// Ile rekordów ma bieżący/ostatni przejazd (zeruje się przy starcie nowego).
uint32_t autonomy_log_count(void);

// Odczyt rekordu o danym indeksie (0..autonomy_log_count()-1).
// Zwraca false, jeśli idx jest poza zakresem.
bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out);

// Nazwa stanu dla wartości zapisanej w rekordzie (do CSV).
const char *autonomy_log_state_name(uint8_t state);
