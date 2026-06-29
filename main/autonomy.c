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

// ============================================================
//  KALIBRACJA – orientacja lidaru względem PRZODU robota
//  ----------------------------------------------------------
//  LID_FRONT_DEG : surowy kąt lidaru (0..359), który wskazuje
//                  fizyczny PRZÓD pojazdu.
//  LID_MIRROR    : 0 albo 1. Jeśli pojazd skręca w złą stronę
//                  / „widzi” przeszkodę po przeciwnej stronie
//                  niż faktyczna – zmień na przeciwną wartość.
//
//  Najłatwiej dobrać te dwie wartości w apce desktopowej
//  (tools/lidar_map.py) – to te same pojęcia, co „offset kąta”
//  i „odwróć kierunek”. Najpierw ustaw je tam wizualnie, potem
//  przepisz tutaj.
// ============================================================
#define LID_FRONT_DEG    0
#define LID_MIRROR       0

// ── Prędkości ───────────────────────────────────────────────
// Podniesione do 35% – na dywaniku/wykładzinie 25% nie wystarczało,
// by utrzymać ruch. Na twardej podłodze możesz zejść niżej.
#define SP_CRUISE        35      // jazda do przodu [% mocy]
#define SP_TURN          35      // obrót
#define SP_BACK          35      // cofanie
#define SP_STEER_MAX      6      // maks. korekta toru przy centrowaniu

// ── Rozruch: przełamanie tarcia statycznego ─────────────────
// Na starcie KAŻDEJ fazy ruchu silniki dostają krótki, mocny impuls,
// żeby ruszyć z miejsca (lekki pojazd przy 25-35% często nie rusza
// z postoju). Po impulsie schodzą do prędkości przelotowej.
// Na bardzo śliskiej podłodze możesz skrócić KICK_MS lub zmniejszyć moc.
#define KICK_POWER       90      // moc impulsu rozruchowego [%]
#define KICK_MS         220      // czas trwania impulsu [ms]

// ── Progi odległości lidaru [mm] ────────────────────────────
#define D_BLOCK         350      // przeszkoda z przodu bliżej => omijaj
#define D_CLEAR         480      // przód uznaj za wolny dopiero powyżej (histereza)
#define D_OPEN_MM      4000      // „brak echa” interpretuj jako tak daleko

// ── Łuki pomiarowe (kąty WZGLĘDEM przodu robota) ────────────
//    0° = przód, +90° = lewo, -90° = prawo, 180° = tył
#define FRONT_HALF       28      // przód: ±28°
#define DIAG_CENTER      45
#define DIAG_HALF        25
#define SIDE_CENTER      75
#define SIDE_HALF        30

// ── Czasy manewrów [ms] ─────────────────────────────────────
#define T_TURN_MIN      250      // min. czas obrotu omijającego (anty-oscylacja)
#define T_AVOID_MAX    4500      // brak wyjazdu w tym czasie => ślepy zaułek
#define T_BACK          700      // czas cofania
#define T_BLIND        700       // ślepy obrót po cofnięciu (~90°)
#define T_BLIND_LONG   1300      // dłuższy obrót (ślepy zaułek)

// ── Wykrywanie utknięcia (odometria) ────────────────────────
#define STUCK_MS       1600      // brak przyrostu impulsów => utknięcie
#define STUCK_MIN_PULSES  1      // min. przyrost uznany za ruch
#define ESCAPE_LIMIT      6      // tyle nieudanych ucieczek => AWARIA (stop)

// ── Cel termiczny ────────────────────────────────────────────
// META = obiekt o tyle stopni CIEPLEJSZY niż otoczenie (różnica),
// a nie próg absolutny – inaczej ciepłe pomieszczenie (np. 30°C
// otoczenia) dałoby falstart. Gorący czajnik z wodą daje dużo
// większą różnicę niż jakikolwiek przedmiot w temperaturze pokojowej.
#define AUTO_TARGET_DELTA_C  50.0f
#define TARGET_CONFIRM         4    // tyle kolejnych odczytów > progu = META

// ── Okres pętli sterowania / logowania ──────────────────────
#define LOOP_MS          50      // 20 Hz – pętla sterowania
#define STATUS_LOG_MS    300     // ~3,3 Hz – log konsoli + rekord do pliku CSV

// ── Bufor logu przejazdu (RAM, do pobrania jako CSV) ────────
// 2000 rekordów * ~24 B ≈ 47 KB. Przy interwale 300 ms to ok. 10 minut
// nagrywania jednego przejazdu – więcej niż dość na pojedynczy test.
// Po zapełnieniu bufora dalsze próbki są odrzucane (przejazd i tak
// dawno powinien się skończyć – META/AWARIA/ręczny STOP).
#define AUTO_LOG_MAX   2000

// ============================================================
typedef enum {
    ST_IDLE,     // wyłączony / bezczynny
    ST_CRUISE,   // jazda do przodu (z centrowaniem w korytarzu)
    ST_AVOID,    // obrót w miejscu, żeby ominąć przeszkodę
    ST_BACK,     // cofanie (po nieudanym ominięciu / utknięciu)
    ST_BLIND,    // ślepy obrót po cofnięciu
    ST_REACHED,  // cel osiągnięty – stop
    ST_FAULT     // awaria (zbyt wiele ucieczek) – stop
} st_t;

static volatile bool s_enabled = false;
static volatile st_t s_state   = ST_IDLE;
static TaskHandle_t  s_task    = NULL;

static uint32_t s_state_t      = 0;   // ms wejścia w bieżący stan
static uint32_t s_progress_t   = 0;   // ms ostatniego postępu (odometria)
static uint32_t s_last_pulses  = 0;
static int      s_dir          = +1;  // kierunek obrotu omijającego (+1=lewo)
static int      s_blind_dir    = -1;  // kierunek ślepego obrotu
static uint32_t s_blind_ms     = T_BLIND;
static int      s_alt          = +1;  // alternujący kierunek dla ucieczek
static int      s_target_hits  = 0;
static int      s_escape_count = 0;
static bool     s_stopped      = false;
static uint32_t s_log_t        = 0;   // ms ostatniego szczegółowego logu

// ── log przejazdu (RAM, do CSV) ──────────────────────────────
static autonomy_log_rec_t s_log[AUTO_LOG_MAX];
static uint32_t           s_log_n        = 0;     // liczba zapisanych rekordów
static uint32_t           s_run_t0       = 0;     // ms startu bieżącego przejazdu
static bool      s_log_full_warned = false;

// Nazwa stanu dla dowolnej wartości enuma (współdzielona przez podgląd
// na żywo – autonomy_state_str() – i log CSV – autonomy_log_state_name()).
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

// ── pomocnicze ──────────────────────────────────────────────
static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
static inline void enter(st_t s) { s_state = s; s_state_t = now_ms(); }

// Minimalna odległość w łuku WZGLĘDEM przodu robota (z kalibracją).
static inline uint16_t arc(int rel_center, int half) {
    int s = LID_MIRROR ? -1 : 1;
    int c = (LID_FRONT_DEG + s * rel_center) % 360;
    if (c < 0) c += 360;
    return lidar_min_in_arc(c, half);
}
// 0 (brak echa) => potraktuj jako „otwarte” (bardzo daleko).
static inline int open_mm(uint16_t v) { return v == 0 ? D_OPEN_MM : (int)v; }

// Obrót: dir>0 = w LEWO, dir<0 = w PRAWO. Kręci się tylko jeden silnik.
// pow = moc [%] (wyższa na czas impulsu rozruchowego, potem SP_TURN).
static inline void pivot_pow(int dir, int pow) {
    if (dir > 0) { motor_set_left(pow);  motor_set_right(0); }
    else         { motor_set_left(0);    motor_set_right(pow); }
}

static void reset_progress(void) {
    odometry_data_t od = odometry_get();
    s_last_pulses = od.pulses_left + od.pulses_right;
    s_progress_t  = now_ms();
}

// Dopisuje jeden rekord do logu przejazdu (RAM). Wołane co STATUS_LOG_MS,
// pomijane w stanie AWARII (bo pojazd stoi i nic się nie zmienia – nie
// ma sensu zapychać bufora powtarzającymi się próbkami).
static void record_sample(uint32_t now, uint16_t front_raw,
                           int diag_l, int diag_r, int side_l, int side_r,
                           pyrometer_data_t pd) {
    if (s_state == ST_FAULT) return;
    if (s_log_n >= AUTO_LOG_MAX) {
        if (!s_log_full_warned) {
            s_log_full_warned = true;
            ESP_LOGW(TAG, "Bufor logu przejazdu pelny (%d rekordow) – dalsze próbki odrzucane.",
                     AUTO_LOG_MAX);
        }
        return;
    }
    autonomy_log_rec_t *r = &s_log[s_log_n++];
    r->t_ms         = now - s_run_t0;
    r->front_mm     = (int16_t)open_mm(front_raw);
    r->diag_l_mm    = (int16_t)diag_l;
    r->diag_r_mm    = (int16_t)diag_r;
    r->side_l_mm    = (int16_t)side_l;
    r->side_r_mm    = (int16_t)side_r;
    r->motor_l      = (int8_t)motor_get_left_speed();
    r->motor_r      = (int8_t)motor_get_right_speed();
    r->obj_temp_x10 = (int16_t)(pd.object_temp  * 10.0f);
    r->amb_temp_x10 = (int16_t)(pd.ambient_temp * 10.0f);
    r->state        = (uint8_t)s_state;
}

// Rozpocznij sekwencję ucieczki: cofnij, potem ślepy obrót.
// long_turn=true dla ślepego zaułka / krawędzi z obu stron.
static void begin_escape(int dir, bool long_turn) {
    s_blind_dir = dir;
    s_blind_ms  = long_turn ? T_BLIND_LONG : T_BLIND;
    s_escape_count++;
    if (s_escape_count > ESCAPE_LIMIT) {
        motor_stop();
        led_set_red(true);
        buzzer_tone(400, 600);
        ESP_LOGE(TAG, "Zbyt wiele prob ucieczki – AWARIA, zatrzymuje pojazd.");
        enter(ST_FAULT);
    } else {
        motor_stop();
        ESP_LOGW(TAG, "Ucieczka #%d (dir=%d, long=%d)", s_escape_count, dir, long_turn);
        enter(ST_BACK);
    }
}

// ── główna pętla sterowania ─────────────────────────────────
static void autonomy_task(void *arg) {
    (void)arg;
    reset_progress();

    while (1) {
        // --- tryb wyłączony: pilnuj, by silniki stały ---
        if (!s_enabled) {
            if (s_state != ST_REACHED && s_state != ST_FAULT) s_state = ST_IDLE;
            if (!s_stopped) { motor_stop(); s_stopped = true; }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_stopped = false;
        uint32_t now = now_ms();

        // --- odczyt czujników ---
        pyrometer_data_t pd = pyrometer_get_last();
        odometry_data_t od = odometry_get();
        uint32_t pulses = od.pulses_left + od.pulses_right;

        // Łuki lidaru – liczone raz na cykl, używane w kilku miejscach
        // i w logu statusu (front_raw zostaje "surowy" – 0 = brak echa,
        // potrzebne do progów D_BLOCK/D_CLEAR; reszta po open_mm()).
        uint16_t front_raw = arc(0, FRONT_HALF);
        int diag_l = open_mm(arc(+DIAG_CENTER, DIAG_HALF));
        int diag_r = open_mm(arc(-DIAG_CENTER, DIAG_HALF));
        int side_l = open_mm(arc(+SIDE_CENTER, SIDE_HALF));
        int side_r = open_mm(arc(-SIDE_CENTER, SIDE_HALF));

        // --- szczegółowy log statusu (konsola) + rekord do pliku CSV ---
        if (now - s_log_t >= STATUS_LOG_MS) {
            s_log_t = now;
            ESP_LOGI(TAG,
                "[%s] silniki L=%d%% R=%d%% | lidar mm: przod=%d diag_L=%d diag_R=%d bok_L=%d bok_R=%d | "
                "termo: obiekt=%.1fC otoczenie=%.1fC delta=%.1fC (potw=%d/%d) | impulsy=%lu bez_ruchu=%lums",
                autonomy_state_str(), motor_get_left_speed(), motor_get_right_speed(),
                open_mm(front_raw), diag_l, diag_r, side_l, side_r,
                pd.object_temp, pd.ambient_temp, pd.object_temp - pd.ambient_temp,
                s_target_hits, TARGET_CONFIRM,
                (unsigned long)pulses, (unsigned long)(now - s_progress_t));
            record_sample(now, front_raw, diag_l, diag_r, side_l, side_r, pd);
        }

        // =========================================================
        // 1) CEL TERMICZNY (najwyższy priorytet)
        //    Pojazd jest „przy czajniku”, gdy MLX widzi obiekt o
        //    AUTO_TARGET_DELTA_C stopni cieplejszy niż otoczenie –
        //    przez kilka kolejnych odczytów (odporne na ciepłe
        //    pomieszczenie / chwilowy szum odczytu).
        // =========================================================
        float delta = pd.object_temp - pd.ambient_temp;
        if (delta >= AUTO_TARGET_DELTA_C) s_target_hits++;
        else                              s_target_hits = 0;

        if (s_target_hits >= TARGET_CONFIRM && s_state != ST_REACHED) {
            motor_stop();
            led_set_green(true);
            buzzer_tone(2000, 150);
            vTaskDelay(pdMS_TO_TICKS(180));
            buzzer_tone(2600, 200);
            ESP_LOGI(TAG, "META! Obiekt=%.1f C, otoczenie=%.1f C (delta=%.1f C) – koniec przejazdu.",
                     pd.object_temp, pd.ambient_temp, delta);
            enter(ST_REACHED);
            s_enabled = false;          // zatrzymaj autonomię (bez kasowania stanu)
            continue;
        }

        // Stany końcowe – trzymaj stop.
        if (s_state == ST_REACHED || s_state == ST_FAULT) {
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        // =========================================================
        // 2) MASZYNA STANÓW
        // =========================================================
        switch (s_state) {

        case ST_IDLE:
            reset_progress();
            enter(ST_CRUISE);
            break;

        case ST_CRUISE: {
            // a) wykrywanie utknięcia (brak przyrostu impulsów)
            if (pulses - s_last_pulses >= STUCK_MIN_PULSES) {
                s_last_pulses = pulses;
                s_progress_t  = now;
            } else if (now - s_progress_t > STUCK_MS) {
                ESP_LOGW(TAG, "Brak postepu (utkniecie) – ucieczka.");
                begin_escape(s_alt, false);
                s_alt = -s_alt;
                break;
            }

            // b) przeszkoda z przodu? -> obróć w stronę bardziej otwartą
            if (front_raw != 0 && front_raw < D_BLOCK) {
                s_dir = (side_l >= side_r) ? +1 : -1;   // +1=lewo, -1=prawo
                led_set_yellow(true);
                ESP_LOGI(TAG, "Przeszkoda z przodu (%u mm) – omijam w %s (bok_L=%d bok_R=%d mm)",
                         front_raw, s_dir > 0 ? "LEWO" : "PRAWO", side_l, side_r);
                enter(ST_AVOID);
                break;
            }

            // c) jazda do przodu: impuls rozruchowy + delikatne centrowanie
            if (now - s_state_t < KICK_MS) {
                // przełamanie tarcia – oba koła pełną mocą prosto
                motor_set_left(KICK_POWER);
                motor_set_right(KICK_POWER);
            } else {
                int k = (diag_r - diag_l) / 60;       // prawo bardziej otwarte => skręć w prawo
                if (k >  SP_STEER_MAX) k =  SP_STEER_MAX;
                if (k < -SP_STEER_MAX) k = -SP_STEER_MAX;
                // korektę realizujemy DODAJĄC kołu zewnętrznemu – żadne koło
                // nie spada poniżej SP_CRUISE (inaczej słabsze koło staje)
                int l = SP_CRUISE, r = SP_CRUISE;
                if (k > 0) l += k;                    // skręt w prawo: dołóż lewemu
                else       r += -k;                   // skręt w lewo: dołóż prawemu
                motor_set_left(l);
                motor_set_right(r);
            }
            led_set_yellow(false);
            break;
        }

        case ST_AVOID: {
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_TURN;
            pivot_pow(s_dir, pow);
            // obrót w miejscu też kręci kołami – odśwież „postęp”,
            // aby po powrocie do jazdy licznik utknięcia startował świeży
            s_last_pulses = pulses;
            s_progress_t  = now;

            int dmin = diag_l < diag_r ? diag_l : diag_r;
            bool clear = (front_raw == 0 || front_raw > D_CLEAR) && dmin > D_BLOCK;

            if (clear && (now - s_state_t) >= T_TURN_MIN) {
                s_escape_count = 0;                 // czyste ominięcie = zdrowo
                ESP_LOGI(TAG, "Tor wolny – wracam do jazdy (przod=%d diag_min=%d mm)",
                         open_mm(front_raw), dmin);
                reset_progress();
                enter(ST_CRUISE);
            } else if (now - s_state_t > T_AVOID_MAX) {
                ESP_LOGW(TAG, "Nie moge ominac (slepy zaulek) – ucieczka.");
                begin_escape(s_alt, true);
                s_alt = -s_alt;
            }
            break;
        }

        case ST_BACK: {
            // cofanie po nieudanej próbie ominięcia / utknięciu (lidar+odometria)
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_BACK;
            motor_backward(pow);
            if (now - s_state_t > T_BACK) {
                motor_stop();
                ESP_LOGI(TAG, "Cofanie zakonczone – obrot w %s (%lu ms).",
                         s_blind_dir > 0 ? "LEWO" : "PRAWO", (unsigned long)s_blind_ms);
                enter(ST_BLIND);
            }
            break;
        }

        case ST_BLIND: {
            int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_TURN;
            pivot_pow(s_blind_dir, pow);
            if (now - s_state_t > s_blind_ms) {
                ESP_LOGI(TAG, "Obrot po ucieczce zakonczony – wracam do jazdy.");
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

// ── API ─────────────────────────────────────────────────────
void autonomy_init(void) {
    if (s_task) return;
    xTaskCreatePinnedToCore(autonomy_task, "autonomy", 4096, NULL, 6, &s_task, 1);
    ESP_LOGI(TAG, "Modul autonomii gotowy (wylaczony). Predkosc jazdy=%d%%.", SP_CRUISE);
}

void autonomy_set_enabled(bool enable) {
    if (enable) {
        s_target_hits  = 0;
        s_escape_count = 0;
        s_alt          = +1;
        led_set_red(false);
        reset_progress();
        enter(ST_CRUISE);
        s_stopped = false;
        // Nowy przejazd => nowy log (stary, jeśli nie pobrany, jest nadpisywany).
        s_log_n           = 0;
        s_log_full_warned = false;
        s_run_t0          = now_ms();
        s_log_t           = 0;          // wymuś natychmiastowy pierwszy rekord
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

// ── API logu przejazdu ───────────────────────────────────────
uint32_t autonomy_log_count(void) { return s_log_n; }

bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out) {
    if (!out || idx >= s_log_n) return false;
    *out = s_log[idx];
    return true;
}

const char *autonomy_log_state_name(uint8_t state) { return state_name((st_t)state); }