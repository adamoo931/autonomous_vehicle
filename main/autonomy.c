#include "autonomy.h"
#include "config.h"
#include "motor_driver.h"
#include "pyrometer.h"
#include "ina219.h"
#include "line_sensor.h"
#include "ads1115.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "AUTO";

/* =====================================================================
 *  Autonomia - wersja minimalna (etap 1, napisana od zera)
 *
 *  Zadanie skryptu jazdy:
 *    1. po wlaczeniu jedzie na wprost ze stala predkoscia,
 *    2. po wykryciu linii toru (ktorykolwiek z TRZECH czujnikow
 *       odbiciowych CNY70 podpietych pod ADC ADS1115: przod-L, tyl-L,
 *       tyl-P; czwarty, cyfrowy na GPIO, swiadomie pomijany - falszywe
 *       odczyty) pojazd STAJE i przez ~LINE_WAIT_MS czeka na sygnal z
 *       czujnika Halla,
 *    3. jesli w tym czasie Hall zamelduje mete (SS495A -> ADS1115,
 *       finish_detected) -> to meta: stop ostateczny, koniec przejazdu,
 *    4. jesli Hall nie zadziala -> to nie meta, tylko zwykle przeciecie
 *       tasmy wyznaczajacej tor: pojazd cofa sie przez ~LINE_BACKUP_MS,
 *       stoi jeszcze ~LINE_POSTBACKUP_PAUSE_MS, po czym znow rusza na
 *       wprost. Wykrywanie linii NIE jest wyciszane po cofnieciu - jesli
 *       pojazd ponownie najedzie na te sama tasme, test (stop/oczekiwanie
 *       na Hall/cofniecie) powtarza sie od nowa. To celowe: ma sluzyc do
 *       weryfikacji poprawnosci odczytow czujnikow odbiciowych (pojazd ma
 *       sie "zapetlac" na tej samej linii, dopoki operator nie przerwie
 *       autonomii recznie).
 *
 *  Swiadomie nie ma tu omijania przeszkod, korekty kursu ani danych z
 *  LIDAR-u - logika czujnikow pozostaje nietknieta, ten modul tylko z
 *  niej korzysta. Kazda reczna komenda silnikow wylacza autonomie
 *  (kill-switch w http_server.c). Potwierdzenie mety wylacza autonomie -
 *  stan "Zatrzymany (meta)" zostaje widoczny na dashboardzie do czasu
 *  ponownego wlaczenia.
 *
 *  LOG PRZEJAZDU: co STATUS_LOG_MS do RAM zapisywany jest kompaktowy
 *  rekord (stan, moc silnikow, temperatury z pirometru). Bufor zerowany
 *  na starcie kazdego przejazdu, do pobrania jako CSV pod
 *  GET /api/autonomy/log.csv. Pola LIDAR w rekordzie nie sa uzywane w
 *  tej wersji (zapisywane jako 0).
 * ===================================================================== */

/* Moc silnikow [% mocy] przy jezdzie na wprost i przy cofaniu (ta wersja:
 * bez ruchu obrotowego) - stala moc przez caly czas ruchu, bez impulsu
 * rozruchowego. Ustawiana z dashboardu (s_speed_pct, patrz
 * autonomy_set_speed_pct()); SPEED_PCT_DEFAULT to wartosc startowa po
 * uruchomieniu ESP32. Gdy w przyszlosci dojdzie logika obrotu/skretu w
 * miejscu, powinna uzywac wyzszej mocy (~50%) - jedno kolo stojace daje
 * wiecej tarcia niz jazda na wprost oboma kolami. */
#define SPEED_PCT_DEFAULT   35
#define SPEED_PCT_MIN         0
#define SPEED_PCT_MAX       100

/* Test mety po wykryciu linii. Meta to magnes pod ta sama tasma, ktora
 * wyznacza granice toru, wiec sama linia nie odroznia mety od zwyklego
 * przeciecia tasmy - robi to dopiero czujnik Halla. */
#define LINE_WAIT_MS            1500  /* postoj na linii - oczekiwanie na sygnal Hall */
#define LINE_BACKUP_MS          2000  /* czas cofania, gdy linia to nie meta */
#define LINE_POSTBACKUP_PAUSE_MS 500  /* postoj po cofnieciu, zanim pojazd znow ruszy */

/* Okresy petli sterowania i logowania. */
#define LOOP_MS          50      /* 20 Hz - petla sterowania */
#define STATUS_LOG_MS   300      /* ~3,3 Hz - log konsoli + rekord CSV */

/* Bufor logu przejazdu (RAM, do pobrania jako CSV). 2000 rekordow po
 * ~24 B ~= 47 KB; przy 300 ms daje ok. 10 minut nagrywania. */
#define AUTO_LOG_MAX   2000

/* Stany maszyny sterujacej. */
typedef enum {
    ST_IDLE,          /* wylaczony / bezczynny */
    ST_CRUISE,        /* jazda na wprost */
    ST_LINE_WAIT,     /* linia wykryta - postoj i oczekiwanie na potwierdzenie mety Hallem */
    ST_LINE_BACKUP,   /* to nie meta - cofanie */
    ST_LINE_PAUSE,    /* krotki postoj po cofnieciu, przed ponowna proba jazdy */
    ST_STOP_FINISH,   /* meta potwierdzona Hallem - stop ostateczny */
} st_t;

static volatile bool s_enabled = false;
static volatile st_t s_state   = ST_IDLE;
static TaskHandle_t  s_task     = NULL;

static uint32_t s_state_t = 0;   /* ms wejscia w biezacy stan */
static bool     s_stopped = false;
static uint32_t s_log_t   = 0;   /* ms ostatniego szczegolowego logu */

/* Zgrubny azymut start->meta z dashboardu. W tej wersji tylko
 * przechowywany (dashboard ma pole) - jazda go nie wykorzystuje. */
static float s_target_azimuth_deg = 0.0f;

/* Moc silnikow [%] przy jezdzie na wprost/do tylu - ustawiana z
 * dashboardu (patrz autonomy_set_speed_pct()). */
static volatile int s_speed_pct = SPEED_PCT_DEFAULT;

/* Log przejazdu w RAM (eksportowany jako CSV). */
static autonomy_log_rec_t s_log[AUTO_LOG_MAX];
static uint32_t s_log_n = 0;
static uint32_t s_run_t0 = 0;
static bool     s_log_full_warned = false;

/* Czas trwania i zuzyta energia biezacego/ostatniego przejazdu. */
static uint32_t s_run_elapsed_ms = 0;
static float    s_run_energy_mwh = 0.0f;

static const char *state_name(st_t s) {
    switch (s) {
        case ST_IDLE:        return "Bezczynny";
        case ST_CRUISE:      return "Jazda";
        case ST_LINE_WAIT:   return "Linia - sprawdzam mete";
        case ST_LINE_BACKUP: return "Cofanie (nie meta)";
        case ST_LINE_PAUSE:  return "Postoj po cofnieciu";
        case ST_STOP_FINISH: return "Zatrzymany (meta)";
        default:             return "?";
    }
}

static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static inline void enter(st_t s) { s_state = s; s_state_t = now_ms(); }

/* Zamraza czas trwania biezacego przejazdu (dashboard ma pokazywac czas
 * do tego momentu, nie licznik biegnacy dalej). Wolane tylko przy
 * s_enabled==true, wiec nadpisuje raz na przejazd. */
static inline void stop_run(void) { s_run_elapsed_ms = now_ms() - s_run_t0; }

/* Dopisuje jeden rekord do logu przejazdu (RAM). Wolane co STATUS_LOG_MS. */
static void record_sample(uint32_t now, pyrometer_data_t pd) {
    if (s_log_n >= AUTO_LOG_MAX) {
        if (!s_log_full_warned) {
            s_log_full_warned = true;
            ESP_LOGW(TAG, "Bufor logu przejazdu pelny (%d rekordow) - dalsze probki odrzucane.",
                     AUTO_LOG_MAX);
        }
        return;
    }
    autonomy_log_rec_t *r = &s_log[s_log_n++];
    r->t_ms          = now - s_run_t0;
    r->front_mm      = 0;
    r->diag_l_mm     = 0;
    r->diag_r_mm     = 0;
    r->side_l_mm     = 0;
    r->side_r_mm     = 0;
    r->motor_l       = (int8_t)motor_get_left_speed();
    r->motor_r       = (int8_t)motor_get_right_speed();
    r->obj_temp_x10  = (int16_t)(pd.object_temp  * 10.0f);
    r->amb_temp_x10  = (int16_t)(pd.ambient_temp * 10.0f);
    r->best_open_deg = 0;
    r->state         = (uint8_t)s_state;
}

/* Wykrycie linii toru na potrzeby autonomii - bazuje WYLACZNIE na trzech
 * czujnikach odbiciowych CNY70 podpietych analogowo przez ADS1115
 * (przod-lewy A1, tyl-lewy A2, tyl-prawy A3). Czwarty czujnik (przod-prawy,
 * cyfrowy na GPIO PIN_LINE_FR) jest tu swiadomie pomijany - potrafi dawac
 * falszywe odczyty. Nie uzywamy line_sensor_any_edge(), bo ono uwzglednia
 * tez ten czujnik GPIO. */
static inline bool track_line_detected(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.back_left || d.back_right;
}

/* Konczy przejazd: zatrzymuje silniki, zamraza czas, wylacza autonomie i
 * ustawia stan koncowy (widoczny na dashboardzie do nastepnego startu). */
static void finish_run(st_t stop_state, const char *reason) {
    ESP_LOGI(TAG, "%s - zatrzymuje pojazd i koncze przejazd.", reason);
    motor_stop();
    enter(stop_state);
    stop_run();
    s_enabled = false;
}

/* Glowna petla sterowania autonomii. */
static void autonomy_task(void *arg) {
    (void)arg;

    while (1) {
        /* Tryb wylaczony: pilnuj, by silniki staly. Stan koncowy
         * ST_STOP_FINISH zostaje widoczny na dashboardzie do ponownego
         * wlaczenia (zeby bylo wiadomo, ze pojazd dojechal do mety);
         * stany przejsciowe kasujemy do "Bezczynny". */
        if (!s_enabled) {
            if (s_state != ST_STOP_FINISH) s_state = ST_IDLE;
            if (!s_stopped) { motor_stop(); s_stopped = true; }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_stopped = false;
        uint32_t now = now_ms();

        pyrometer_data_t pd = pyrometer_get_last();
        ina219_data_t    pw = ina219_get_last();

        /* Calkowanie energii: moc [mW] * czas [h] = energia [mWh].
         * Zakladany krok czasowy to LOOP_MS. */
        s_run_energy_mwh += pw.power_mw * (LOOP_MS / 3600000.0f);

        /* Szczegolowy log statusu (konsola) oraz rekord do CSV. */
        if (now - s_log_t >= STATUS_LOG_MS) {
            s_log_t = now;
            ESP_LOGI(TAG, "[%s] silniki L=%d%% R=%d%% | termo: obiekt=%.1fC otoczenie=%.1fC",
                     state_name(s_state), motor_get_left_speed(), motor_get_right_speed(),
                     pd.object_temp, pd.ambient_temp);
            record_sample(now, pd);
        }

        /* Meta (Hall) ma bezwzgledny priorytet - potwierdza mete
         * niezaleznie od tego, czy pojazd jedzie, czeka na linii, czy sie
         * cofa. Magnes lezy pod ta sama tasma co granica toru, wiec
         * dopiero Hall odroznia mete od zwyklego przeciecia tasmy. */
        if (s_state != ST_STOP_FINISH && ads1115_get_last().finish_detected) {
            finish_run(ST_STOP_FINISH, "Meta potwierdzona czujnikiem Halla");
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        switch (s_state) {

        case ST_IDLE:
            enter(ST_CRUISE);
            break;

        case ST_CRUISE:
            /* Wykrywanie linii jest zawsze "uzbrojone" - takze zaraz po
             * cofnieciu i ponownym najechaniu na te sama tasme. Robione
             * celowo: to ma sluzyc do powtarzalnego sprawdzania odczytow
             * czujnikow odbiciowych (stop -> test Halla -> cofniecie ->
             * ponowny stop na tej samej linii), nie do omijania jej. */
            if (track_line_detected()) {
                ESP_LOGI(TAG, "Linia wykryta - stop, czekam %d ms na potwierdzenie mety (Hall).",
                         LINE_WAIT_MS);
                motor_stop();
                enter(ST_LINE_WAIT);
                break;
            }
            /* Jazda na wprost: stala moc (z dashboardu), bez impulsu rozruchowego. */
            motor_set_left(s_speed_pct);
            motor_set_right(s_speed_pct);
            break;

        case ST_LINE_WAIT:
            /* Stoj i odliczaj. Meta (Hall) obsluzona wyzej, priorytetowo -
             * tutaj rozstrzygamy tylko przypadek "Hall nie zadzialal". */
            motor_stop();
            if (now - s_state_t >= LINE_WAIT_MS) {
                ESP_LOGI(TAG, "Brak sygnalu Hall przez %d ms - to nie meta. Cofam %d ms.",
                         LINE_WAIT_MS, LINE_BACKUP_MS);
                enter(ST_LINE_BACKUP);
            }
            break;

        case ST_LINE_BACKUP:
            if (now - s_state_t < LINE_BACKUP_MS) {
                motor_set_left(-s_speed_pct);
                motor_set_right(-s_speed_pct);
            } else {
                motor_stop();
                enter(ST_LINE_PAUSE);
            }
            break;

        case ST_LINE_PAUSE:
            /* Krotki postoj po cofnieciu, zanim pojazd znow ruszy na wprost. */
            motor_stop();
            if (now - s_state_t >= LINE_POSTBACKUP_PAUSE_MS) {
                ESP_LOGI(TAG, "Postoj po cofnieciu zakonczony - jade dalej na wprost.");
                enter(ST_CRUISE);
            }
            break;

        case ST_STOP_FINISH:
        default:
            motor_stop();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void autonomy_init(void) {
    if (s_task) return;
    xTaskCreatePinnedToCore(autonomy_task, "autonomy", 4096, NULL, 6, &s_task, 1);
    ESP_LOGI(TAG, "Modul autonomii gotowy (wylaczony). Predkosc jazdy=%d%%.", s_speed_pct);
}

void autonomy_set_enabled(bool enable) {
    if (enable) {
        enter(ST_CRUISE);
        s_stopped = false;
        /* Nowy przejazd => nowy log (poprzedni, jesli nie pobrany, zostaje nadpisany). */
        s_log_n           = 0;
        s_log_full_warned = false;
        s_run_t0          = now_ms();
        s_run_elapsed_ms  = 0;
        s_run_energy_mwh  = 0.0f;
        s_log_t           = 0;          /* wymus natychmiastowy pierwszy rekord */
        s_enabled = true;
        ESP_LOGI(TAG, "Autonomia WLACZONA - jazda na wprost; linia -> test mety Hallem, brak mety -> cofnij i jedz dalej (log wyzerowany).");
    } else {
        /* Zamrazamy czas przejazdu tylko przy realnym przejsciu wl.->wyl.
         * (handler bywa wolany tez, gdy autonomia jest juz wylaczona -
         * np. przy kazdym recznym ruchu silnika, patrz http_server.c). */
        if (s_enabled) stop_run();
        s_enabled = false;
        motor_stop();
        if (s_state != ST_STOP_FINISH) enter(ST_IDLE);
        ESP_LOGI(TAG, "Autonomia WYLACZONA.");
    }
}

bool autonomy_is_enabled(void) { return s_enabled; }

const char *autonomy_state_str(void) { return state_name(s_state); }

void autonomy_set_target_azimuth(float deg) {
    /* Normalizacja do [0,360). Przechowywane, ale jazda go nie uzywa. */
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    s_target_azimuth_deg = deg;
    ESP_LOGI(TAG, "Zadany azymut start->meta ustawiony na %.1f st. (nieuzywany w wersji minimalnej).",
             s_target_azimuth_deg);
}

float autonomy_get_target_azimuth(void) { return s_target_azimuth_deg; }

void autonomy_set_speed_pct(int pct) {
    if (pct < SPEED_PCT_MIN) pct = SPEED_PCT_MIN;
    if (pct > SPEED_PCT_MAX) pct = SPEED_PCT_MAX;
    s_speed_pct = pct;
    ESP_LOGI(TAG, "Predkosc jazdy autonomicznej (na wprost/do tylu) ustawiona na %d%%.", pct);
}

int autonomy_get_speed_pct(void) { return s_speed_pct; }

float autonomy_get_run_time_s(void) {
    uint32_t ms = s_enabled ? (now_ms() - s_run_t0) : s_run_elapsed_ms;
    return (float)ms / 1000.0f;
}

float autonomy_get_run_energy_mwh(void) { return s_run_energy_mwh; }

/* API logu przejazdu. */
uint32_t autonomy_log_count(void) { return s_log_n; }

bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out) {
    if (!out || idx >= s_log_n) return false;
    *out = s_log[idx];
    return true;
}

const char *autonomy_log_state_name(uint8_t state) { return state_name((st_t)state); }
