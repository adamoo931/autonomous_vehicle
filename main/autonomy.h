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

/* Czy autonomia potwierdziła metę i zakończyła przejazd (stan
 * "Zatrzymany (meta)"). Zostaje true po wyłączeniu autonomii, aż do startu
 * kolejnego przejazdu. sensor_task w main.c reaguje sygnałem mety (buzzer,
 * pirometr, zielona dioda) na TĘ decyzję - potwierdzoną oknem kontaktu z
 * taśmą i debounce w autonomy.c - a nie na surowy, podatny na zakłócenia od
 * silników odczyt czujnika Halla. */
bool autonomy_finish_reached(void);

/* Krótki opis bieżącego stanu (dla dashboardu), np. "Jazda",
 * "Linia - sprawdzam metę", "Cofanie (nie meta)", "Postój po cofnięciu",
 * "Zatrzymany (meta)". */
const char *autonomy_state_str(void);

/* --- Krok 2: podgląd na żywo estymatora kursu z żyroskopu. Kurs = całka
 * (gyro_z_filt - bias) z rzeczywistym Δt, liczona w pętli autonomii
 * NIEZALEŻNIE od tego, czy autonomia jest włączona (obserwowalność bez
 * ruszania silników). Bias mierzony przez ~1 s bezruchu na starcie każdego
 * przejazdu (stan "Kalibracja żyroskopu"); kurs wtedy zerowany. Nadal NIE
 * używane w sterowaniu - to Krok 3. */
float autonomy_get_heading_deg(void);       /* scałkowany kurs względny [°], zawinięty (-180,180] */
float autonomy_get_gyro_z_dps(void);        /* surowa prędkość kątowa yaw (gyro_z) [°/s] */
float autonomy_get_gyro_z_filt_dps(void);   /* gyro_z po filtrze EMA [°/s] */
float autonomy_get_gyro_bias_dps(void);     /* zmierzony bias gyro_z [°/s] (0 przed 1. kalibracją) */

/* --- Krok 3/4: zadany kurs (cel regulatora utrzymania kursu w ST_CRUISE),
 * względem kierunku startowego. + = w lewo. Ustawiany z dashboardu
 * (POST /api/autonomy/heading). Przycinany do ±90°. Do testów jazdy prosto
 * ustaw 0. Domyślnie ~18° (namiar start->meta). */
float autonomy_get_heading_target_deg(void);
void  autonomy_set_heading_target_deg(float deg);

/* Kopiuje 8 minimalnych odległości sektorowych LIDAR [mm] do out[8]
 * (0 = brak echa / kierunek otwarty). Kolejność: przód, przód-L, lewo,
 * tył-L, tył, tył-P, prawo, przód-P (patrz enum SEC_* w autonomy.c). */
void autonomy_get_lidar_sectors_mm(int16_t out[8]);

/* --- Krok 6: kalibracja/parametry kontroli korytarza (LIDAR), nastawiane
 * z dashboardu (POST /api/autonomy/lidar). front_deg = offset przodu głowicy
 * LIDAR [°], dobierany narzędziem tools/lidar_map.py; front_stop_mm = próg
 * zatrzymania przed przeszkodą [mm], przycinany do [150,1500]. corridor_mm =
 * ostatnio policzona min. odległość w korytarzu na wprost (telemetria). */
int  autonomy_get_lid_front_deg(void);
void autonomy_set_lid_front_deg(int deg);
int  autonomy_get_front_stop_mm(void);
void autonomy_set_front_stop_mm(int mm);
int  autonomy_get_corridor_mm(void);

/* --- Krok 7: parametry omijania przeszkody (follow-the-gap), nastawiane z
 * dashboardu (POST /api/autonomy/lidar, pola scan_deg / pass_ms).
 * scan_deg = polowa zakresu skanu szczelin [°], przycinane do [30,120];
 * pass_ms = czas jazdy "przez szczeline" [ms], przycinane do [500,6000]. */
int  autonomy_get_scan_max_deg(void);
void autonomy_set_scan_max_deg(int deg);
int  autonomy_get_avoid_pass_ms(void);
void autonomy_set_avoid_pass_ms(int ms);

/* --- Krok 9: wykrycie dalekiej (górnej) krawędzi toru = przejście faza 1->2.
 * forward_ms = skumulowany czas jazdy do przodu w bieżącym przejeździe [ms]
 * (podgląd, do kalibracji); traverse_ms = próg, po którym kontakt z taśmą
 * przy zbieżnym kursie jest traktowany jako górna krawędź. Nastawiane z
 * dashboardu (POST /api/autonomy/lidar, pole traverse_ms), przycinane do
 * [5000,180000]. Nie zapisywane w NVS. */
int  autonomy_get_forward_ms(void);
int  autonomy_get_traverse_ms(void);
void autonomy_set_traverse_ms(int ms);

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

/* Pojedynczy próbkowany rekord przejazdu (co STATUS_LOG_MS). Wartości
 * skalowane *10 / w mV, aby uniknąć typów zmiennoprzecinkowych i zmniejszyć
 * rozmiar rekordu. Odległości LIDAR w mm, 0 = brak echa / kierunek otwarty.
 *
 * Krok 1: doszły surowe prędkości kątowe (3 osie żyroskopu - do ustalenia,
 * która oś to yaw), scałkowany kurs względny (surowy), 8 sektorów LIDAR i
 * napięcia 3 czujników linii - komplet sygnałów decyzyjnych dla kolejnych
 * kroków. Stare pola LIDAR (front, diag L/R, side L/R, best_open_deg)
 * zastąpiono tablicą lidar_mm[8]. Krok 5e: doszło napięcie i wykrycie Halla
 * mety - wcześniej nie było ich w logu, co utrudniało weryfikację
 * podejrzewanych fałszywych zadziałań. */
typedef struct {
    uint32_t t_ms;          /* czas od startu przejazdu [ms] */
    uint8_t  state;         /* wartość enum stanu - patrz autonomy_log_state_name() */
    int8_t   motor_l;       /* -100..100 [%] */
    int8_t   motor_r;
    int16_t  gyro_x_x10;    /* prędkość kątowa X [°/s] * 10 (surowa z IMU) */
    int16_t  gyro_y_x10;    /* prędkość kątowa Y [°/s] * 10 */
    int16_t  gyro_z_x10;    /* prędkość kątowa Z (yaw) [°/s] * 10 - surowa */
    int16_t  gyro_zf_x10;   /* gyro_z po filtrze EMA [°/s] * 10 (Krok 2) */
    int16_t  gyro_bias_x10; /* aktualny bias gyro_z [°/s] * 10 - zmienny w czasie (Krok 4b: powolna adaptacja) */
    int16_t  heading_x10;   /* scałkowany kurs względny [°] * 10, zawinięty (-180,180]; po odjęciu biasu (Krok 2) */
    int16_t  lidar_mm[8];   /* min. odległość w 8 sektorach [mm]; 0 = otwarte. Kolejność jak w SEC_* */
    int16_t  corridor_mm;   /* Krok 6: min. odległość w korytarzu na wprost (3 promienie) [mm] */
    int16_t  line_fr_mv;    /* czujnik linii przód-prawy (ADS1115 A0) [mV] */
    int16_t  line_fl_mv;    /* czujnik linii przód-lewy [mV] */
    int16_t  line_bl_mv;    /* tył-lewy [mV] */
    int16_t  line_br_mv;    /* tył-prawy [mV] */
    int8_t   hall_do;       /* surowy stan pinu DO cyfrowego Halla mety (0/1) */
    uint8_t  hall_hit;      /* meta wg polaryzacji DO (hall_finish_detected) w chwili próbki (0/1) */
    uint8_t  hall_latch;    /* Krok 9c: zatrzask - impuls DO zlapany w ostatnich HALL_LATCH_MS (0/1) */
    int16_t  obj_temp_x10;  /* pirometr: temperatura obiektu [°C] * 10 */
    int16_t  amb_temp_x10;  /* pirometr: temperatura otoczenia [°C] * 10 */
} autonomy_log_rec_t;

/* Liczba rekordów bieżącego/ostatniego przejazdu (zerowana przy starcie nowego). */
uint32_t autonomy_log_count(void);

/* Odczyt rekordu o danym indeksie (0..autonomy_log_count()-1).
 * Zwraca false, jeśli idx jest poza zakresem. */
bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out);

/* Nazwa stanu odpowiadająca wartości zapisanej w rekordzie (do CSV). */
const char *autonomy_log_state_name(uint8_t state);
