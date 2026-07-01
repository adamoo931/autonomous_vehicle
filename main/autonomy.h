#pragma once
#include <stdbool.h>
#include <stdint.h>

/* =====================================================================
 *  Autonomia - etap 1: jazda na wprost od startu, zatrzymanie na
 *  przeszkodzie wykrytej LIDAR-em. Bez omijania/skrętu - to świadomie
 *  minimalna wersja do zweryfikowania samej integracji LIDAR+silniki,
 *  zanim dojdzie etap 2 (skan otoczenia i wybór wystarczająco szerokiej
 *  szczeliny, z uwzględnieniem zadanego azymutu start->meta) oraz etap 3
 *  (szukanie źródła ciepła pirometrem po wykryciu mety magnetycznej -
 *  patrz pyrometer_start_search()/main.c, zaimplementowane już osobno).
 *
 *  Tryb uruchamiany jest z dashboardu (POST /api/autonomy
 *  {"enable":true/false}). Każda ręczna komenda silników natychmiast
 *  wyłącza autonomię (kill-switch). Napotkanie przeszkody też wyłącza
 *  autonomię (s_enabled=false) - stan "Zatrzymany" zostaje widoczny na
 *  dashboardzie do czasu ponownego włączenia.
 *
 *  LOG PRZEJAZDU
 *  Co STATUS_LOG_MS (patrz autonomy.c) do pamięci RAM zapisywany jest
 *  kompaktowy rekord (stan, moc silników, odległości LIDAR, temperatury).
 *  Bufor zerowany jest na starcie każdego przejazdu i dostępny jako CSV
 *  pod GET /api/autonomy/log.csv - do czasu resetu ESP32 lub rozpoczęcia
 *  nowego przejazdu. Log należy pobrać zaraz po zakończeniu jazdy, zanim
 *  ruszy kolejna, inaczej zostanie nadpisany.
 * ===================================================================== */

/* Tworzy zadanie autonomii (startuje w stanie bezczynnym). */
void autonomy_init(void);

/* Włącza / wyłącza tryb autonomiczny. */
void autonomy_set_enabled(bool enable);

/* Czy autonomia jest aktualnie aktywna. */
bool autonomy_is_enabled(void);

/* Krótki opis bieżącego stanu (dla dashboardu), np. "Jazda",
 * "Zatrzymany (przeszkoda)". */
const char *autonomy_state_str(void);

/* Zgrubny azymut start->meta [stopnie, 0..360), wpisywany z dashboardu
 * przed przejazdem. Na razie tylko przechowywany - wykorzysta go przyszła
 * logika nawigacji (etap 2) przy wyborze kierunku spośród kilku otwartych
 * szczelin. */
void  autonomy_set_target_azimuth(float deg);
float autonomy_get_target_azimuth(void);

/* =====================================================================
 *  LOG PRZEJAZDU (do pobrania jako CSV przez http_server.c)
 * ===================================================================== */

/* Pojedynczy próbkowany rekord przejazdu (co STATUS_LOG_MS). Odległości
 * w mm, gdzie D_OPEN_MM (autonomy.c) oznacza "brak echa / otwarte".
 * Temperatury są mnożone *10 (np. 235 = 23.5 st. C), aby uniknąć typów
 * zmiennoprzecinkowych i zmniejszyć rozmiar rekordu - dzięki czemu w
 * buforze RAM mieści się więcej próbek. */
typedef struct {
    uint32_t t_ms;          /* czas od startu przejazdu [ms] */
    int16_t  front_mm;
    int16_t  diag_l_mm;
    int16_t  diag_r_mm;
    int16_t  side_l_mm;
    int16_t  side_r_mm;
    int8_t   motor_l;       /* -100..100 [%] */
    int8_t   motor_r;
    int16_t  obj_temp_x10;
    int16_t  amb_temp_x10;
    int16_t  best_open_deg; /* kąt najbardziej otwartego kierunku dookoła [st], + = lewo */
    uint8_t  state;         /* wartość enum stanu - patrz autonomy_log_state_name() */
} autonomy_log_rec_t;

/* Liczba rekordów bieżącego/ostatniego przejazdu (zerowana przy starcie nowego). */
uint32_t autonomy_log_count(void);

/* Odczyt rekordu o danym indeksie (0..autonomy_log_count()-1).
 * Zwraca false, jeśli idx jest poza zakresem. */
bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out);

/* Nazwa stanu odpowiadająca wartości zapisanej w rekordzie (do CSV). */
const char *autonomy_log_state_name(uint8_t state);
