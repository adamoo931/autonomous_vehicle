#pragma once
#include <stdbool.h>
#include <stdint.h>

/* =====================================================================
 *  Autonomia - etap 1 (wersja minimalna): jazda na wprost od startu.
 *  Po wykryciu linii toru (którykolwiek z trzech czujników odbiciowych
 *  CNY70 podpiętych pod ADC ADS1115; czwarty, cyfrowy na GPIO, pomijany -
 *  fałszywe odczyty) pojazd staje i przez ~1,5 s czeka na sygnał czujnika
 *  Halla mety (SS495A -> ADS1115). Jeśli Hall zamelduje metę - stop
 *  ostateczny, koniec przejazdu. Jeśli nie - linia była tylko przecięciem
 *  taśmy toru (meta to magnes pod tą samą taśmą): pojazd cofa się ~2 s,
 *  stoi jeszcze ~0,5 s i jedzie dalej na wprost. Wykrywanie linii nie jest
 *  po tym wyciszane - ponowne najechanie na tę samą taśmę powtarza cały
 *  test od nowa (celowo, do weryfikacji odczytów czujników odbiciowych).
 *  Bez omijania przeszkód, korekty
 *  kursu i danych z LIDAR-u - to świadomie minimalna wersja skryptu
 *  jazdy; logika czujników pozostaje nietknięta, moduł tylko z niej
 *  korzysta. Przyszłe etapy: skan otoczenia i wybór szczeliny z
 *  uwzględnieniem azymutu start->meta oraz szukanie źródła ciepła
 *  pirometrem (patrz pyrometer_start_search()/main.c).
 *
 *  Tryb uruchamiany jest z dashboardu (POST /api/autonomy
 *  {"enable":true/false}). Każda ręczna komenda silników natychmiast
 *  wyłącza autonomię (kill-switch). Potwierdzenie mety też wyłącza
 *  autonomię (s_enabled=false) - stan "Zatrzymany (meta)" zostaje
 *  widoczny na dashboardzie do czasu ponownego włączenia.
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
 * "Linia - sprawdzam metę", "Cofanie (nie meta)", "Postój po cofnięciu",
 * "Zatrzymany (meta)". */
const char *autonomy_state_str(void);

/* Zgrubny azymut start->meta [stopnie, 0..360), wpisywany z dashboardu
 * przed przejazdem. Na razie tylko przechowywany - wykorzysta go przyszła
 * logika nawigacji (etap 2) przy wyborze kierunku spośród kilku otwartych
 * szczelin. */
void  autonomy_set_target_azimuth(float deg);
float autonomy_get_target_azimuth(void);

/* Moc silników [% mocy, 0..100] przy jeździe na wprost i przy cofaniu
 * (ST_CRUISE/ST_LINE_BACKUP w autonomy.c) - wpisywana z dashboardu.
 * Wartość spoza zakresu jest przycinana. Domyślnie 35% (patrz autonomy.c). */
void autonomy_set_speed_pct(int pct);
int  autonomy_get_speed_pct(void);

/* Czas trwania [s] i energia zużyta [mWh] w bieżącym/ostatnim przejeździe,
 * liczone od startu (autonomy_set_enabled(true)) do zatrzymania autonomii
 * (ręcznego lub wewnętrznego - meta/przeszkoda bez szczeliny). Energia jest
 * całkowana z odczytów INA219 w pętli sterowania (patrz autonomy.c). */
float autonomy_get_run_time_s(void);
float autonomy_get_run_energy_mwh(void);

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
