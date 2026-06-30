#include "autonomy.h"
#include "config.h"
#include "motor_driver.h"
#include "lidar.h"
#include "pyrometer.h"
#include "odometry.h"
#include "led_control.h"
#include "buzzer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "AUTO";

/* Kalibracja orientacji LIDAR względem przodu robota.
 *   LID_FRONT_DEG - surowy kąt LIDAR (0..359) wskazujący fizyczny przód.
 *   LID_MIRROR    - 0 albo 1; jeśli pojazd skręca w złą stronę lub "widzi"
 *                   przeszkodę po przeciwnej stronie niż faktyczna, zmień
 *                   na przeciwną wartość.
 * Najwygodniej dobrać obie wartości wizualnie w narzędziu desktopowym
 * (tools/lidar_map.py) - odpowiadają tam "offsetowi kąta" i "odwróceniu
 * kierunku" - a następnie przepisać tutaj. */
#define LID_FRONT_DEG    0
#define LID_MIRROR       0

/* Prędkości [% mocy]. Wartości dobrane pod miękkie podłoże (dywan/wykładzina),
 * gdzie poniżej ~35% lekki pojazd nie utrzymuje ruchu. Na twardej podłodze
 * można je obniżyć. */
#define SP_CRUISE        35      /* jazda do przodu */
#define SP_TURN          35      /* obrót */
#define SP_BACK          35      /* cofanie */
#define SP_STEER_MAX      6      /* maks. korekta toru przy centrowaniu */

/* Impuls rozruchowy: na początku każdej fazy ruchu silniki dostają krótki,
 * mocny impuls przełamujący tarcie statyczne (lekki pojazd przy 25-35%
 * często nie rusza z postoju), po czym schodzą do prędkości przelotowej.
 * Na bardzo śliskiej podłodze można skrócić KICK_MS lub zmniejszyć moc. */
#define KICK_POWER       90      /* moc impulsu rozruchowego [%] */
#define KICK_MS         220      /* czas trwania impulsu [ms] */

/* Progi nawigacji "jedź w najbardziej otwartą przestrzeń" [mm].
 * FRONT_GO - minimalna wolna przestrzeń z przodu pozwalająca jechać prosto.
 * Poniżej tej wartości pojazd obraca się ku najbardziej otwartemu kierunkowi
 * dookoła (także do tyłu) zamiast napierać na bliską ścianę. */
#define FRONT_GO        600      /* [mm] próg jazdy do przodu */
#define FRONT_GO_HYST   120      /* [mm] histereza wyjścia z obrotu (anty-drganie) */
#define D_OPEN_MM      4000      /* brak echa interpretowany jako ta odległość (otwarte) */
#define STEER_DIV       100      /* dzielnik łagodnej korekty toru (mniejszy = mocniej) */

/* Łuki pomiarowe (kąty względem przodu robota):
 *   0 st. = przód, +90 st. = lewo, -90 st. = prawo, 180 st. = tył. */
#define FRONT_HALF       28      /* przód: +/-28 st. */
#define DIAG_CENTER      45
#define DIAG_HALF        25
#define SIDE_CENTER      75
#define SIDE_HALF        30

/* Czasy manewrów [ms]. */
#define T_TURN_MIN      250      /* min. czas obrotu omijającego (anty-oscylacja) */
#define T_AVOID_MAX    6000      /* brak otwarcia w tym czasie => ucieczka */
#define T_BACK          700      /* czas cofania */
#define T_BLIND        700       /* ślepy obrót po cofnięciu (~90 st.) */
#define T_BLIND_LONG   1300      /* dłuższy obrót (ślepy zaułek) */

/* Wykrywanie utknięcia na podstawie odometrii. */
#define STUCK_MS       1600      /* brak przyrostu impulsów => utknięcie */
#define STUCK_MIN_PULSES  1      /* min. przyrost uznawany za ruch */
#define ESCAPE_LIMIT      6      /* tyle nieudanych ucieczek => awaria (stop) */

/* Cel termiczny: META to obiekt o zadaną RÓŻNICĘ stopni cieplejszy niż
 * otoczenie, a nie próg absolutny - inaczej ciepłe pomieszczenie dałoby
 * falstart. Gorący obiekt (np. czajnik z wodą) daje różnicę znacznie większą
 * niż cokolwiek w temperaturze pokojowej. */
#define AUTO_TARGET_DELTA_C  50.0f
#define TARGET_CONFIRM         4    /* tyle kolejnych odczytów > progu = META */

/* Okresy pętli sterowania i logowania. */
#define LOOP_MS          50      /* 20 Hz - pętla sterowania */
#define STATUS_LOG_MS    300     /* ~3,3 Hz - log konsoli + rekord CSV */

/* Bufor logu przejazdu (RAM, do pobrania jako CSV).
 * 2000 rekordów * ~24 B ~= 47 KB. Przy interwale 300 ms daje to ok. 10 minut
 * nagrywania jednego przejazdu. Po zapełnieniu dalsze próbki są odrzucane
 * (przejazd i tak powinien już się zakończyć: META/awaria/ręczny stop). */
#define AUTO_LOG_MAX   2000

/* Stany maszyny sterującej. */
typedef enum {
    ST_IDLE,     /* wyłączony / bezczynny */
    ST_CRUISE,   /* jazda do przodu (z centrowaniem w korytarzu) */
    ST_AVOID,    /* obrót w miejscu w celu ominięcia przeszkody */
    ST_BACK,     /* cofanie (po nieudanym ominięciu / utknięciu) */
    ST_BLIND,    /* ślepy obrót po cofnięciu */
    ST_REACHED,  /* cel osiągnięty - stop */
    ST_FAULT     /* awaria (zbyt wiele ucieczek) - stop */
} st_t;

static volatile bool s_enabled = false;
static volatile st_t s_state   = ST_IDLE;
static TaskHandle_t  s_task    = NULL;

static uint32_t s_state_t      = 0;   /* ms wejścia w bieżący stan */
static uint32_t s_progress_t   = 0;   /* ms ostatniego postępu (odometria) */
static uint32_t s_last_pulses  = 0;
static int      s_dir          = +1;  /* kierunek obrotu ku otwartej przestrzeni (+1=lewo) */
static int      s_blind_dir    = -1;  /* kierunek ślepego obrotu (ucieczka) */
static uint32_t s_blind_ms     = T_BLIND;
static int      s_target_hits  = 0;
static int      s_escape_count = 0;
static bool     s_stopped      = false;
static uint32_t s_log_t        = 0;   /* ms ostatniego szczegółowego logu */

/* Log przejazdu w RAM (eksportowany jako CSV). */
static autonomy_log_rec_t s_log[AUTO_LOG_MAX];
static uint32_t           s_log_n        = 0;     /* liczba zapisanych rekordów */
static uint32_t           s_run_t0       = 0;     /* ms startu bieżącego przejazdu */
static bool      s_log_full_warned = false;

/* Nazwa stanu dla danej wartości enuma. Współdzielona przez podgląd na żywo
 * (autonomy_state_str) oraz log CSV (autonomy_log_state_name). */
static const char *state_name(st_t s) {
    switch (s) {
        case ST_IDLE:    return "Bezczynny";
        case ST_CRUISE:  return "Jazda";
        case ST_AVOID:   return "Omijanie przeszkody";
        case ST_BACK:    return "Cofanie";
        case ST_BLIND:   return "Obrot";
        case ST_REACHED: return "META osiagnieta";
        case ST_FAULT:   return "Awaria (utkniecie)";
        default:         return "?";
    }
}

static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static inline void enter(st_t s) { s_state = s; s_state_t = now_ms(); }

/* Minimalna odległość w łuku względem przodu robota (z uwzględnieniem kalibracji). */
static inline uint16_t arc(int rel_center, int half) {
    int s = LID_MIRROR ? -1 : 1;
    int c = (LID_FRONT_DEG + s * rel_center) % 360;
    if (c < 0) c += 360;
    return lidar_min_in_arc(c, half);
}
/* Brak echa (0) traktujemy jako kierunek otwarty (bardzo daleko). */
static inline int open_mm(uint16_t v) { return v == 0 ? D_OPEN_MM : (int)v; }

/* Obrót w miejscu: dir>0 = w lewo, dir<0 = w prawo (kręci jeden silnik).
 * pow = moc [%] (wyższa na czas impulsu rozruchowego, potem SP_TURN). */
static inline void pivot_pow(int dir, int pow) {
    if (dir > 0) { motor_set_left(pow);  motor_set_right(0); }
    else         { motor_set_left(0);    motor_set_right(pow); }
}

static void reset_progress(void) {
    odometry_data_t od = odometry_get();
    s_last_pulses = od.pulses_left + od.pulses_right;
    s_progress_t  = now_ms();
}

/* Dopisuje jeden rekord do logu przejazdu (RAM). Wołane co STATUS_LOG_MS;
 * pomijane w stanie awarii, bo pojazd stoi i nic się nie zmienia - nie ma
 * sensu zapychać bufora powtarzającymi się próbkami. */
static void record_sample(uint32_t now, uint16_t front_raw,
                           int diag_l, int diag_r, int side_l, int side_r,
                           int best_open_deg, pyrometer_data_t pd) {
    if (s_state == ST_FAULT) return;
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
    r->front_mm      = (int16_t)open_mm(front_raw);
    r->diag_l_mm     = (int16_t)diag_l;
    r->diag_r_mm     = (int16_t)diag_r;
    r->side_l_mm     = (int16_t)side_l;
    r->side_r_mm     = (int16_t)side_r;
    r->motor_l       = (int8_t)motor_get_left_speed();
    r->motor_r       = (int8_t)motor_get_right_speed();
    r->obj_temp_x10  = (int16_t)(pd.object_temp  * 10.0f);
    r->amb_temp_x10  = (int16_t)(pd.ambient_temp * 10.0f);
    r->best_open_deg = (int16_t)best_open_deg;
    r->state         = (uint8_t)s_state;
}

/* Skanuje 12 kierunków dookoła robota (co 30 st.) i zwraca kierunek obrotu
 * (+1 = w lewo, -1 = w prawo) ku najbardziej otwartemu z nich. Uwzględnienie
 * tyłu (120..180..-120 st.) pozwala zawrócić, gdy cała wolna przestrzeń jest
 * za pojazdem. Do *best_angle_out (jeśli != NULL) trafia kąt najbardziej
 * otwartego sektora (do logu). */
static int seek_open_dir(int *best_angle_out) {
    static const int ang[12]  = {0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30};
    static const int half[12] = {FRONT_HALF, 16, 16, 16, 16, 16, 18, 16, 16, 16, 16, 16};
    int best_i = 0, best_cl = -1;
    for (int i = 0; i < 12; i++) {
        int cl = open_mm(arc(ang[i], half[i]));
        if (cl > best_cl) { best_cl = cl; best_i = i; }
    }
    if (best_angle_out) *best_angle_out = ang[best_i];
    if (ang[best_i] == 0) return +1;                 /* już z przodu (kierunek nieistotny) */
    if (ang[best_i] == 180) {                          /* dokładnie z tyłu - wybierz wg stron */
        int l = open_mm(arc(90, 25)), r = open_mm(arc(-90, 25));
        return (l >= r) ? +1 : -1;
    }
    return (ang[best_i] > 0) ? +1 : -1;                /* dodatni kąt => w lewo */
}

/* Rozpoczyna sekwencję ucieczki: cofnięcie, a następnie ślepy obrót ku
 * otwartej stronie. long_turn=true dla ślepego zaułka (dłuższy obrót). */
static void begin_escape(bool long_turn) {
    s_blind_dir = seek_open_dir(NULL);
    s_blind_ms  = long_turn ? T_BLIND_LONG : T_BLIND;
    s_escape_count++;
    if (s_escape_count > ESCAPE_LIMIT) {
        motor_stop();
        led_set_red(true);
        buzzer_tone(400, 600);
        ESP_LOGE(TAG, "Zbyt wiele prob ucieczki - AWARIA, zatrzymuje pojazd.");
        enter(ST_FAULT);
    } else {
        motor_stop();
        ESP_LOGW(TAG, "Ucieczka #%d (cofnij + obrot w %s, long=%d)",
                 s_escape_count, s_blind_dir > 0 ? "LEWO" : "PRAWO", long_turn);
        enter(ST_BACK);
    }
}

/* Główna pętla sterowania autonomii. */
static void autonomy_task(void *arg) {
    (void)arg;
    reset_progress();

    while (1) {
        /* Tryb wyłączony: pilnuj, by silniki stały. */
        if (!s_enabled) {
            if (s_state != ST_REACHED && s_state != ST_FAULT) s_state = ST_IDLE;
            if (!s_stopped) { motor_stop(); s_stopped = true; }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_stopped = false;
        uint32_t now = now_ms();

        /* Odczyt czujników. */
        pyrometer_data_t pd = pyrometer_get_last();
        odometry_data_t od = odometry_get();
        uint32_t pulses = od.pulses_left + od.pulses_right;

        /* Łuki LIDAR liczone raz na cykl, używane w decyzjach i logu.
         * front_raw pozostaje surowy (0 = brak echa) - potrzebny do progu
         * FRONT_GO; pozostałe przepuszczone przez open_mm() (0 -> D_OPEN_MM). */
        uint16_t front_raw = arc(0, FRONT_HALF);
        int diag_l = open_mm(arc(+DIAG_CENTER, DIAG_HALF));
        int diag_r = open_mm(arc(-DIAG_CENTER, DIAG_HALF));
        int side_l = open_mm(arc(+SIDE_CENTER, SIDE_HALF));
        int side_r = open_mm(arc(-SIDE_CENTER, SIDE_HALF));

        /* Szczegółowy log statusu (konsola) oraz rekord do CSV. */
        if (now - s_log_t >= STATUS_LOG_MS) {
            s_log_t = now;
            int best_open = 0;
            seek_open_dir(&best_open);   /* kąt najbardziej otwartego kierunku dookoła */
            ESP_LOGI(TAG,
                "[%s] silniki L=%d%% R=%d%% | lidar mm: przod=%d diag_L=%d diag_R=%d bok_L=%d bok_R=%d | "
                "najwiecej_miejsca=%d st | termo: obiekt=%.1fC otoczenie=%.1fC delta=%.1fC (potw=%d/%d) | "
                "impulsy=%lu bez_ruchu=%lums",
                autonomy_state_str(), motor_get_left_speed(), motor_get_right_speed(),
                open_mm(front_raw), diag_l, diag_r, side_l, side_r, best_open,
                pd.object_temp, pd.ambient_temp, pd.object_temp - pd.ambient_temp,
                s_target_hits, TARGET_CONFIRM,
                (unsigned long)pulses, (unsigned long)(now - s_progress_t));
            record_sample(now, front_raw, diag_l, diag_r, side_l, side_r, best_open, pd);
        }

        /* 1) Cel termiczny - najwyższy priorytet. Pojazd jest przy celu, gdy
         *    MLX widzi obiekt o AUTO_TARGET_DELTA_C stopni cieplejszy niż
         *    otoczenie przez kilka kolejnych odczytów (odporność na ciepłe
         *    pomieszczenie i chwilowy szum). */
        float delta = pd.object_temp - pd.ambient_temp;
        if (delta >= AUTO_TARGET_DELTA_C) s_target_hits++;
        else                              s_target_hits = 0;

        if (s_target_hits >= TARGET_CONFIRM && s_state != ST_REACHED) {
            motor_stop();
            led_set_green(true);
            buzzer_tone(2000, 150);
            vTaskDelay(pdMS_TO_TICKS(180));
            buzzer_tone(2600, 200);
            ESP_LOGI(TAG, "META! Obiekt=%.1f C, otoczenie=%.1f C (delta=%.1f C) - koniec przejazdu.",
                     pd.object_temp, pd.ambient_temp, delta);
            enter(ST_REACHED);
            s_enabled = false;          /* zatrzymaj autonomię bez kasowania stanu */
            continue;
        }

        /* Stany końcowe - trzymaj stop. */
        if (s_state == ST_REACHED || s_state == ST_FAULT) {
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        /* 2) Maszyna stanów. */
        switch (s_state) {

        case ST_IDLE:
            reset_progress();
            enter(ST_CRUISE);
            break;

        case ST_CRUISE: {
            /* a) Wykrywanie utknięcia (brak przyrostu impulsów). */
            if (pulses - s_last_pulses >= STUCK_MIN_PULSES) {
                s_last_pulses = pulses;
                s_progress_t  = now;
            } else if (now - s_progress_t > STUCK_MS) {
                ESP_LOGW(TAG, "Brak postepu (utkniecie) - ucieczka.");
                begin_escape(false);
                break;
            }

            /* b) Za mało miejsca z przodu? Obróć się ku najbardziej otwartemu
             *    kierunkowi dookoła (także do tyłu - jeśli wolna przestrzeń
             *    jest za pojazdem, zawróci). */
            if (front_raw != 0 && front_raw < FRONT_GO) {
                int best = 0;
                s_dir = seek_open_dir(&best);
                led_set_yellow(true);
                ESP_LOGI(TAG, "Ciasno z przodu (%u mm) - obracam ku otwartej przestrzeni "
                              "(%s, najlepszy sektor %d st.)",
                         front_raw, s_dir > 0 ? "LEWO" : "PRAWO", best);
                enter(ST_AVOID);
                break;
            }

            /* c) Jazda do przodu: impuls rozruchowy, a po nim łagodny skręt ku
             *    bardziej otwartej stronie (celowanie w przestrzeń, nie w róg). */
            if (now - s_state_t < KICK_MS) {
                motor_set_left(KICK_POWER);
                motor_set_right(KICK_POWER);
            } else {
                int lo = open_mm(arc(+45, 22)), ll = open_mm(arc(+90, 25));
                int ro = open_mm(arc(-45, 22)), rr = open_mm(arc(-90, 25));
                int left_open  = lo > ll ? lo : ll;
                int right_open = ro > rr ? ro : rr;
                int bias = (left_open - right_open) / STEER_DIV;   /* >0 => skręć w lewo */
                if (bias >  SP_STEER_MAX) bias =  SP_STEER_MAX;
                if (bias < -SP_STEER_MAX) bias = -SP_STEER_MAX;
                /* Korektę realizujemy dodając moc kołu zewnętrznemu - żadne koło
                 * nie spada poniżej SP_CRUISE (inaczej słabsze koło staje). */
                int l = SP_CRUISE, r = SP_CRUISE;
                if (bias > 0) r += bias;      /* skręt w lewo: przyspiesz prawe (zewnętrzne) */
                else          l += -bias;     /* skręt w prawo: przyspiesz lewe */
                motor_set_left(l);
                motor_set_right(r);
            }
            led_set_yellow(false);
            break;
        }

        case ST_AVOID: {
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_TURN;
            pivot_pow(s_dir, pow);
            /* Obrót w miejscu też kręci kołami - odśwież postęp, aby po powrocie
             * do jazdy licznik utknięcia startował od zera. */
            s_last_pulses = pulses;
            s_progress_t  = now;

            /* Wyjście dopiero, gdy z przodu jest naprawdę otwarcie (histereza
             * zapobiega przeskakiwaniu tam i z powrotem na granicy progu). */
            bool open_ahead = (front_raw == 0) || (front_raw >= FRONT_GO + FRONT_GO_HYST);
            if (open_ahead && (now - s_state_t) >= T_TURN_MIN) {
                s_escape_count = 0;                 /* znaleziono otwartą przestrzeń */
                ESP_LOGI(TAG, "Przod otwarty (%d mm) - jade w te strone.", open_mm(front_raw));
                reset_progress();
                enter(ST_CRUISE);
            } else if (now - s_state_t > T_AVOID_MAX) {
                ESP_LOGW(TAG, "Po pelnym obrocie brak otwarcia - ucieczka (cofniecie).");
                begin_escape(true);
            }
            break;
        }

        case ST_BACK: {
            /* Cofanie po nieudanej próbie ominięcia / utknięciu. */
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_BACK;
            motor_backward(pow);
            if (now - s_state_t > T_BACK) {
                motor_stop();
                ESP_LOGI(TAG, "Cofanie zakonczone - obrot w %s (%lu ms).",
                         s_blind_dir > 0 ? "LEWO" : "PRAWO", (unsigned long)s_blind_ms);
                enter(ST_BLIND);
            }
            break;
        }

        case ST_BLIND: {
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_TURN;
            pivot_pow(s_blind_dir, pow);
            if (now - s_state_t > s_blind_ms) {
                ESP_LOGI(TAG, "Obrot po ucieczce zakonczony - wracam do jazdy.");
                reset_progress();
                enter(ST_CRUISE);
            }
            break;
        }

        default:
            enter(ST_IDLE);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void autonomy_init(void) {
    if (s_task) return;
    xTaskCreatePinnedToCore(autonomy_task, "autonomy", 4096, NULL, 6, &s_task, 1);
    ESP_LOGI(TAG, "Modul autonomii gotowy (wylaczony). Predkosc jazdy=%d%%.", SP_CRUISE);
}

void autonomy_set_enabled(bool enable) {
    if (enable) {
        s_target_hits  = 0;
        s_escape_count = 0;
        led_set_red(false);
        reset_progress();
        enter(ST_CRUISE);
        s_stopped = false;
        /* Nowy przejazd => nowy log (poprzedni, jeśli nie pobrany, zostaje nadpisany). */
        s_log_n           = 0;
        s_log_full_warned = false;
        s_run_t0          = now_ms();
        s_log_t           = 0;          /* wymuś natychmiastowy pierwszy rekord */
        s_enabled = true;
        ESP_LOGI(TAG, "Autonomia WLACZONA. (log przejazdu wyzerowany)");
    } else {
        s_enabled = false;
        motor_stop();
        if (s_state != ST_REACHED && s_state != ST_FAULT) enter(ST_IDLE);
        ESP_LOGI(TAG, "Autonomia WYLACZONA.");
    }
}

bool autonomy_is_enabled(void) { return s_enabled; }

const char *autonomy_state_str(void) { return state_name(s_state); }

/* API logu przejazdu. */
uint32_t autonomy_log_count(void) { return s_log_n; }

bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out) {
    if (!out || idx >= s_log_n) return false;
    *out = s_log[idx];
    return true;
}

const char *autonomy_log_state_name(uint8_t state) { return state_name((st_t)state); }
