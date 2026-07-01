#include "autonomy.h"
#include "config.h"
#include "motor_driver.h"
#include "lidar.h"
#include "pyrometer.h"
#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

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
 * gdzie poniżej ~35% lekki pojazd nie utrzymuje ruchu (może się zatrzymać
 * w trakcie jazdy). Wróć do 35 (sprzed prób spowalniania) - błąd z
 * opóźnienia magnetometru adresujemy teraz cyklem skan->jazda->stop->skan
 * (patrz CRUISE_SEGMENT_MS niżej), a nie samym zwalnianiem. */
#define SP_CRUISE        35      /* jazda do przodu */

/* Impuls rozruchowy: na początku jazdy silniki dostają krótki, mocny impuls
 * przełamujący tarcie statyczne, po czym schodzą do prędkości przelotowej. */
#define KICK_POWER       90      /* moc impulsu rozruchowego [%] */
#define KICK_MS         150      /* czas trwania impulsu [ms] */

/* Próg wykrycia przeszkody: swobodna przestrzeń z przodu poniżej tej
 * wartości = stop i próba ominięcia (patrz szukanie szczeliny niżej). */
#define FRONT_STOP_MM   400      /* [mm] */
#define D_OPEN_MM      4000      /* brak echa interpretowany jako ta odległość (otwarte) */

/* Etap 2: ominięcie przeszkody przez sprawdzenie korytarza. Zamiast zgadywać
 * szerokość szczeliny z kształtu ciągłego przebiegu wolnej przestrzeni
 * (zawodne przy rogach/prostych ścianach - cięciwa z jednego punktu widzenia
 * nie odpowiada rzeczywistej geometrii), dla każdego kandydującego kierunku
 * sprawdzamy WPROST, czy jazda na wprost w tę stronę jest czysta na
 * GAP_LOOKAHEAD_MM jednocześnie na środku, lewej i prawej krawędzi pojazdu
 * (przeliczonych geometrycznie z GAP_HALF_WIDTH_MM - połowa szerokości
 * pojazdu + zapas na niedokładne wycelowanie i szum LIDAR-u). To bezpośrednio
 * odpowiada na pytanie "czy pojazd się tu zmieści jadąc prosto".
 * GAP_SEARCH_HALF_DEG ogranicza pierwszą (preferowaną) próbę do łuku +/-100
 * st. od przodu; gdy nic nie znajdzie (np. pułapka w kształcie U - otwarcie
 * jest gdzieś z boku/z tyłu poza tym zakresem), druga próba szuka w
 * szerszym GAP_SEARCH_WIDE_DEG. LIDAR widzi 360 st. cały czas, niezależnie
 * od tego, czy pojazd stoi - nie trzeba fizycznie się kręcić, żeby
 * "skanować". GAP_DWELL_MS to krótkie oczekiwanie po zatrzymaniu (i po
 * cofnięciu - patrz BACKUP_MS), by mieć pewność, że dane z LIDAR-u są
 * świeże z obecnej pozycji (co najmniej jeden pełny obrót głowicy). */
#define VEHICLE_WIDTH_MM       250
#define GAP_MARGIN_MM            40    /* zapas po każdej stronie [mm] */
#define GAP_HALF_WIDTH_MM     (VEHICLE_WIDTH_MM / 2 + GAP_MARGIN_MM)
#define GAP_LOOKAHEAD_MM        600    /* jak daleko do przodu musi być czysto */
#define GAP_SEARCH_HALF_DEG     100
#define GAP_SEARCH_WIDE_DEG     170    /* awaryjna, szersza próba (prawie pełny obrót) */
#define GAP_STEP_DEG               5
#define GAP_DWELL_MS             400
#define GAP_MAX_RETRIES             5    /* tyle nieudanych prób z rzędu => trwały stop */
#define GAP_CLEAR_SEGMENT_MS    1200    /* krótki odcinek jazdy przez szczelinę - szybszy powrót do celu */
#define DEG_TO_RAD_F      0.017453292f

/* Cofnięcie przed skanowaniem/obrotem: przy zatrzymaniu blisko przeszkody
 * (zwłaszcza w rogu/ciasnym zakątku) sam obrót w miejscu potrafi zahaczyć
 * o przeszkodę z boku, mimo że docelowy kierunek jest wolny - bo pojazd
 * zamiata swoim obrysem cały łuk pośredni, nie tylko cel. Krótkie cofnięcie
 * daje zapas, zanim ST_SCAN_GAP zacznie oceniać kierunki. */
#define SP_BACK           35      /* moc cofania [%], jak SP_CRUISE */
#define BACKUP_MS        600      /* czas cofania przed skanowaniem */

/* Wyrównywanie do zadanego azymutu (magnetometr IMU, patrz sensors/imu.c).
 *
 * Cykl "skanuj -> jedź odcinek -> stop -> skanuj ponownie" zamiast ciągłej
 * reaktywnej korekty w trakcie jazdy (ta ostatnia przerywała jazdę zbyt
 * często). ALIGN_MAX_MS ogranicza czas samego skanowania/wyrównywania - gdy
 * magnetometr nie zdąży się ustabilizować w tolerancji, pojazd i tak rusza
 * po tym czasie z ostatnim odczytem, zamiast czekać w nieskończoność.
 * CRUISE_SEGMENT_MS to długość jednego odcinka jazdy na wprost, po którym
 * pojazd ZAWSZE się zatrzymuje i skanuje od nowa - niezależnie od tego, czy
 * w międzyczasie realnie zszedł z kursu. Dzięki temu korekty są rzadkie i
 * przewidywalne (raz na odcinek), a nie wyzwalane przy każdym drobnym/
 * pozornym przekroczeniu progu.
 *
 * ALIGN_TOLERANCE_DEG - próg błędu kursu uznawany za "wyrównany".
 * ALIGN_CONFIRM_COUNT - tyle kolejnych przebiegów pętli (co LOOP_MS) błąd
 * musi utrzymać się w tolerancji, zanim uznamy kurs za wyrównany - pojedynczy,
 * chwilowo poprawny odczyt (opóźniony względem rzeczywistego kursu) nie
 * wystarczy. W trakcie potwierdzania pojazd stoi (nie kręci się dalej), żeby
 * dać czujnikowi czas na ustabilizowanie odczytu.
 * REALIGN_TRIGGER_DEG działa już tylko jako awaryjny wyzwalacz przy naprawdę
 * dużym zejściu z kursu w trakcie odcinka (np. potrącenie/poślizg) - nie jako
 * główny mechanizm korekty, stąd wysoki próg. */
#define ALIGN_TOLERANCE_DEG    5.0f
#define ALIGN_CONFIRM_COUNT       4    /* 4 * LOOP_MS(50ms) = 200ms ustabilizowanego odczytu */
#define ALIGN_MAX_MS           3000    /* limit czasu skanowania/wyrównywania */
#define CRUISE_SEGMENT_MS      4000    /* długość odcinka jazdy przed kolejnym skanem */
#define REALIGN_TRIGGER_DEG    40.0f   /* awaryjny próg w trakcie odcinka jazdy */
/* Obrót w miejscu kręci tylko jednym kołem (drugie stoi i dodaje tarcie),
 * więc potrzebuje więcej mocy niż jazda na wprost obu kołami - 25% okazało
 * się za mało (pojazd nie ruszał / ledwo pełzał). 38% to trochę powyżej
 * udokumentowanego progu ruchu obu kół (35%, patrz SP_CRUISE), z zapasem na
 * większe tarcie pojedynczego koła. */
#define ALIGN_TURN_POWER         38
/* 0/1 - odwróć kierunek obrotu wyrównującego, jeśli w praktyce skręca w złą
 * stronę (analogicznie do LID_MIRROR powyżej - dobierz empirycznie).
 * Ustawione na 1 po pierwszym teście: pojazd stabilizował się przy
 * current=target-180 zamiast przy target (oscylując na granicy zmiany znaku
 * błędu kursu) - klasyczna sygnatura odwróconego znaku w pętli zamkniętej. */
#define ALIGN_DIR_FLIP            1

/* Łuki pomiarowe (kąty względem przodu robota), używane do logu/telemetrii
 * i jako gotowa baza pod przyszły etap 2 (skan + wybór szczeliny) -
 * patrz seek_open_dir(). Nie sterują jazdą w tej wersji.
 *   0 st. = przód, +90 st. = lewo, -90 st. = prawo, 180 st. = tył. */
#define FRONT_HALF       28      /* przód: +/-28 st. */
#define DIAG_CENTER      45
#define DIAG_HALF        25
#define SIDE_CENTER      75
#define SIDE_HALF        30

/* Okresy pętli sterowania i logowania. */
#define LOOP_MS          50      /* 20 Hz - pętla sterowania */
#define STATUS_LOG_MS    300     /* ~3,3 Hz - log konsoli + rekord CSV */

/* Bufor logu przejazdu (RAM, do pobrania jako CSV).
 * 2000 rekordów * ~24 B ~= 47 KB. Przy interwale 300 ms daje to ok. 10 minut
 * nagrywania jednego przejazdu. Po zapełnieniu dalsze próbki są odrzucane
 * (przejazd i tak powinien już się zakończyć: przeszkoda/ręczny stop). */
#define AUTO_LOG_MAX   2000

/* Stany maszyny sterującej: obrót do zadanego azymutu, jazda odcinkami,
 * przy przeszkodzie - szukanie szczeliny i ominięcie, a dopiero gdy się nie
 * uda - trwały stop. */
typedef enum {
    ST_IDLE,      /* wyłączony / bezczynny */
    ST_ALIGN,     /* obrót w miejscu do osiągnięcia zadanego kursu (celu lub szczeliny) */
    ST_CRUISE,    /* jazda na wprost wzdłuż zadanego kursu */
    ST_BACKUP,    /* przeszkoda z przodu - krótkie cofnięcie przed skanowaniem/obrotem */
    ST_SCAN_GAP,  /* szukanie szczeliny do ominięcia */
    ST_STOPPED,   /* brak szczeliny / zbyt wiele nieudanych prób - stop na stałe */
} st_t;

static volatile bool s_enabled = false;
static volatile st_t s_state   = ST_IDLE;
static TaskHandle_t  s_task    = NULL;

static uint32_t s_state_t      = 0;   /* ms wejścia w bieżący stan */
static bool     s_stopped      = false;
static uint32_t s_log_t        = 0;   /* ms ostatniego szczegółowego logu */
static int      s_align_confirm = 0;  /* licznik kolejnych przebiegów w tolerancji (ST_ALIGN, tryb magnetometru) */
static int      s_gap_retries   = 0;  /* licznik prób ominięcia z rzędu bez pełnego czystego odcinka */
static bool     s_in_gap_clear  = false;  /* czy bieżący ST_CRUISE to przejazd przez szczelinę (krótszy odcinek) */

/* Zgrubny azymut start->meta, wpisywany z dashboardu przed przejazdem. */
static float s_target_azimuth_deg = 0.0f;
/* Bieżący cel nawigacji - zwykle równy s_target_azimuth_deg, ale tymczasowo
 * podmieniany na kierunek znalezionej szczeliny na czas jej pokonywania. */
static float s_nav_target_deg = 0.0f;

/* Log przejazdu w RAM (eksportowany jako CSV). */
static autonomy_log_rec_t s_log[AUTO_LOG_MAX];
static uint32_t           s_log_n        = 0;     /* liczba zapisanych rekordów */
static uint32_t           s_run_t0       = 0;     /* ms startu bieżącego przejazdu */
static bool      s_log_full_warned = false;

/* Nazwa stanu dla danej wartości enuma. Współdzielona przez podgląd na żywo
 * (autonomy_state_str) oraz log CSV (autonomy_log_state_name). */
static const char *state_name(st_t s) {
    switch (s) {
        case ST_IDLE:     return "Bezczynny";
        case ST_ALIGN:    return "Wyrownywanie azymutu";
        case ST_CRUISE:   return "Jazda";
        case ST_BACKUP:   return "Cofanie";
        case ST_SCAN_GAP: return "Szukanie szczeliny";
        case ST_STOPPED:  return "Zatrzymany";
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

/* Najkrótsza różnica kątowa target-current, znormalizowana do (-180,180].
 * Dodatnia = cel jest zgodnie z ruchem wskazówek zegara od bieżącego kursu
 * (azymut rośnie w tę stronę), więc trzeba skręcić w prawo. */
static inline float heading_error_deg(float target, float current) {
    float d = target - current;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* Obrót w miejscu: dir>0 = w lewo (azymut maleje), dir<0 = w prawo (azymut
 * rośnie) - kręci jeden silnik, drugi stoi. Kierunek dobrany empirycznie
 * (patrz ALIGN_DIR_FLIP), analogicznie do kalibracji LID_MIRROR. */
static inline void pivot_pow(int dir, int pow) {
    if (ALIGN_DIR_FLIP) dir = -dir;
    if (dir > 0) { motor_set_left(pow);  motor_set_right(0); }
    else         { motor_set_left(0);    motor_set_right(pow); }
}

/* Skanuje 12 kierunków dookoła robota (co 30 st.) i zwraca kąt najbardziej
 * otwartego z nich (do *best_angle_out, jeśli != NULL) - obecnie tylko do
 * logu/telemetrii, przyda się jako gotowy budulec pod etap 2 (wybór
 * szczeliny), gdy dojdzie faktyczne sterowanie na jego podstawie. */
static void seek_open_dir(int *best_angle_out) {
    static const int ang[12]  = {0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30};
    static const int half[12] = {FRONT_HALF, 16, 16, 16, 16, 16, 18, 16, 16, 16, 16, 16};
    int best_i = 0, best_cl = -1;
    for (int i = 0; i < 12; i++) {
        int cl = open_mm(arc(ang[i], half[i]));
        if (cl > best_cl) { best_cl = cl; best_i = i; }
    }
    if (best_angle_out) *best_angle_out = ang[best_i];
}

/* Najmniejsza odległość (mm) spośród trzech promieni - środek, lewa i prawa
 * krawędź pojazdu - w kierunku center_deg (względem przodu), przy założeniu
 * jazdy na wprost na lookahead_mm. Kąt krawędzi wyliczony geometrycznie z
 * GAP_HALF_WIDTH_MM (połowa szerokości pojazdu + zapas) przy tym dystansie.
 * To bezpośrednia kontrola korytarza zamiast pojedynczego wąskiego stożka -
 * łapie też przeszkody, które kolidowałyby z bokiem pojazdu, mimo że nie są
 * dokładnie na wprost. */
static int corridor_margin(int center_deg, int lookahead_mm) {
    float edge_rad = atanf((float)GAP_HALF_WIDTH_MM / (float)lookahead_mm);
    int   edge_deg = (int)(edge_rad / DEG_TO_RAD_F + 0.5f);
    int c = open_mm(arc(center_deg,             GAP_STEP_DEG));
    int l = open_mm(arc(center_deg + edge_deg,  GAP_STEP_DEG));
    int r = open_mm(arc(center_deg - edge_deg,  GAP_STEP_DEG));
    int m = c;
    if (l < m) m = l;
    if (r < m) m = r;
    return m;
}
static inline bool corridor_clear(int center_deg, int lookahead_mm) {
    return corridor_margin(center_deg, lookahead_mm) >= lookahead_mm;
}

#define GAP_MAX_CANDS       80
/* Kandydaci o marginesie w obrębie tej wartości od najlepszego znalezionego
 * są traktowani jako porównywalnie bezpieczni - między nimi decyduje kierunek
 * na cel. Bez tego wygrywał kandydat "ledwo przechodzący" próg (np. 605mm
 * przy wymaganych 600mm), bo kątowo bliższy celowi, zamiast dużo bezpieczniej
 * otwartego kierunku dalej w bok - stąd zbyt mały obrót przy ostrych kątach. */
#define GAP_MARGIN_TIE_MM  150.0f

/* Dla każdego kandydującego kierunku w łuku +/-search_half_deg od przodu
 * liczy margines korytarza (corridor_margin) przy GAP_LOOKAHEAD_MM. Spośród
 * kandydatów z marginesem >= GAP_LOOKAHEAD_MM wybiera najpierw ten o
 * największym marginesie (najbezpieczniejszy, najbardziej centralny w wolnej
 * przestrzeni); kierunek na cel (desired_rel_deg) decyduje tylko między
 * porównywalnie bezpiecznymi (patrz GAP_MARGIN_TIE_MM). Zwraca true i
 * wypełnia *out_rel_deg (kąt względem przodu), jeśli coś znaleziono. */
static bool find_gap(float desired_rel_deg, int search_half_deg, int *out_rel_deg) {
    int cand_deg[GAP_MAX_CANDS];
    int cand_margin[GAP_MAX_CANDS];
    int n = 0;

    for (int a = -search_half_deg; a <= search_half_deg && n < GAP_MAX_CANDS; a += GAP_STEP_DEG) {
        int margin = corridor_margin(a, GAP_LOOKAHEAD_MM);
        if (margin >= GAP_LOOKAHEAD_MM) {
            cand_deg[n]    = a;
            cand_margin[n] = margin;
            n++;
        }
    }
    if (n == 0) return false;

    int max_margin = 0;
    for (int i = 0; i < n; i++) if (cand_margin[i] > max_margin) max_margin = cand_margin[i];

    int   best_deg  = cand_deg[0];
    float best_cost = 1e9f;
    for (int i = 0; i < n; i++) {
        if ((float)cand_margin[i] < (float)max_margin - GAP_MARGIN_TIE_MM) continue;
        float cost = fabsf(heading_error_deg(desired_rel_deg, (float)cand_deg[i]));
        if (cost < best_cost) {
            best_cost = cost;
            best_deg  = cand_deg[i];
        }
    }

    if (out_rel_deg) *out_rel_deg = best_deg;
    return true;
}

/* Dopisuje jeden rekord do logu przejazdu (RAM). Wołane co STATUS_LOG_MS. */
static void record_sample(uint32_t now, uint16_t front_raw,
                           int diag_l, int diag_r, int side_l, int side_r,
                           int best_open_deg, pyrometer_data_t pd) {
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

/* Główna pętla sterowania autonomii. */
static void autonomy_task(void *arg) {
    (void)arg;

    while (1) {
        /* Tryb wyłączony: pilnuj, by silniki stały. Stan ST_STOPPED (jeśli
         * to on spowodował wyłączenie) zostaje widoczny na dashboardzie do
         * czasu ponownego włączenia autonomii - żeby było wiadomo, dlaczego
         * pojazd stanął. */
        if (!s_enabled) {
            if (s_state != ST_STOPPED) s_state = ST_IDLE;
            if (!s_stopped) { motor_stop(); s_stopped = true; }
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_stopped = false;
        uint32_t now = now_ms();

        pyrometer_data_t pd  = pyrometer_get_last();
        imu_data_t       imu = imu_get_last();
        float herr = heading_error_deg(s_nav_target_deg, imu.azimuth_deg);

        /* Łuki LIDAR liczone raz na cykl - front_raw to teraz tylko
         * telemetria/log (decyzję o zatrzymaniu podejmuje corridor_clear(),
         * które sprawdza też krawędzie pojazdu, nie tylko wąski stożek). */
        uint16_t front_raw = arc(0, FRONT_HALF);
        int diag_l = open_mm(arc(+DIAG_CENTER, DIAG_HALF));
        int diag_r = open_mm(arc(-DIAG_CENTER, DIAG_HALF));
        int side_l = open_mm(arc(+SIDE_CENTER, SIDE_HALF));
        int side_r = open_mm(arc(-SIDE_CENTER, SIDE_HALF));

        /* Szczegółowy log statusu (konsola) oraz rekord do CSV. */
        if (now - s_log_t >= STATUS_LOG_MS) {
            s_log_t = now;
            int best_open = 0;
            seek_open_dir(&best_open);
            ESP_LOGI(TAG,
                "[%s] silniki L=%d%% R=%d%% | azymut: cel_koncowy=%.1f cel_biezacy=%.1f biezacy=%.1f blad=%.1f (mag=%d) | "
                "lidar mm: przod=%d diag_L=%d diag_R=%d bok_L=%d bok_R=%d | najwiecej_miejsca=%d st | "
                "termo: obiekt=%.1fC otoczenie=%.1fC",
                autonomy_state_str(), motor_get_left_speed(), motor_get_right_speed(),
                s_target_azimuth_deg, s_nav_target_deg, imu.azimuth_deg, herr, imu.mag_initialized,
                open_mm(front_raw), diag_l, diag_r, side_l, side_r, best_open,
                pd.object_temp, pd.ambient_temp);
            record_sample(now, front_raw, diag_l, diag_r, side_l, side_r, best_open, pd);
        }

        /* Stan końcowy - trzymaj stop. */
        if (s_state == ST_STOPPED) {
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        switch (s_state) {

        case ST_IDLE:
            enter(ST_ALIGN);
            s_align_confirm = 0;
            break;

        case ST_ALIGN:
            /* Brak magnetometru - nie ma czego wyrównywać, jedź od razu. */
            if (!imu.mag_initialized) {
                ESP_LOGW(TAG, "Brak magnetometru IMU - pomijam wyrownanie do azymutu.");
                enter(ST_CRUISE);
                s_align_confirm = 0;
                break;
            }
            if (fabsf(herr) <= ALIGN_TOLERANCE_DEG) {
                s_align_confirm++;
                if (s_align_confirm < ALIGN_CONFIRM_COUNT) {
                    /* W trakcie potwierdzania stój - dajemy czujnikowi czas
                     * na ustabilizowanie odczytu, zamiast dalej się kręcić. */
                    motor_stop();
                    break;
                }
                ESP_LOGI(TAG, "Azymut osiagniety i potwierdzony (cel=%.1f biezacy=%.1f blad=%.1f) - jade prosto.",
                         s_nav_target_deg, imu.azimuth_deg, herr);
                enter(ST_CRUISE);
                s_align_confirm = 0;
                break;
            }
            s_align_confirm = 0;   /* wypadło poza tolerancję - zacznij potwierdzanie od nowa */

            /* Limit czasu skanowania - jeśli magnetometr nie zdąży się
             * ustabilizować w tolerancji, jedź dalej z ostatnim odczytem
             * zamiast czekać w nieskończoność. */
            if (now - s_state_t >= ALIGN_MAX_MS) {
                ESP_LOGW(TAG, "Limit czasu wyrownania (%d ms) - jade dalej (blad=%.1f st.).",
                         ALIGN_MAX_MS, herr);
                enter(ST_CRUISE);
                break;
            }
            {
                /* herr>0: cel po stronie rosnacego azymutu -> skrec w prawo (dir=-1). */
                int dir = (herr > 0) ? -1 : +1;
                int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : ALIGN_TURN_POWER;
                pivot_pow(dir, pow);
            }
            break;

        case ST_CRUISE: {
            /* Przeszkoda z przodu -> stop i próba ominięcia (szukanie
             * szczeliny), zamiast trwałego zatrzymania. Licznik prób z rzędu
             * chroni przed pętlą w ciasnym zakątku (patrz GAP_MAX_RETRIES). */
            if (!corridor_clear(0, FRONT_STOP_MM)) {
                motor_stop();
                s_gap_retries++;
                if (s_gap_retries > GAP_MAX_RETRIES) {
                    ESP_LOGE(TAG, "Zbyt wiele nieudanych prob ominiecia (%d) - zatrzymuje pojazd na stale.",
                             s_gap_retries);
                    enter(ST_STOPPED);
                    s_enabled = false;
                    break;
                }
                ESP_LOGI(TAG, "Przeszkoda z przodu/boku (margines=%d mm) - cofam sie przed skanowaniem (proba #%d).",
                         corridor_margin(0, FRONT_STOP_MM), s_gap_retries);
                enter(ST_BACKUP);
                break;
            }

            /* Koniec odcinka jazdy - zawsze stop i ponowny skan azymutu,
             * niezależnie od tego, czy w międzyczasie realnie zszedł z kursu.
             * To główny mechanizm korekty (patrz komentarz przy stałych).
             * Pełny czysty odcinek = realny postęp -> zeruje licznik prób
             * ominięcia i wraca do celowania w prawdziwy cel (a nie w
             * kierunek ostatnio pokonywanej szczeliny). */
            uint32_t seg_ms = s_in_gap_clear ? GAP_CLEAR_SEGMENT_MS : CRUISE_SEGMENT_MS;
            if (now - s_state_t >= seg_ms) {
                ESP_LOGI(TAG, "Koniec odcinka jazdy (%lu ms, szczelina=%d) - stop i ponowne skanowanie azymutu.",
                         (unsigned long)seg_ms, s_in_gap_clear);
                motor_stop();
                s_gap_retries    = 0;
                s_in_gap_clear   = false;
                s_nav_target_deg = s_target_azimuth_deg;
                enter(ST_ALIGN);
                s_align_confirm = 0;
                break;
            }

            /* Awaryjny wyzwalacz przy naprawdę dużym zejściu z kursu w
             * trakcie odcinka (np. poślizg) - nie główny mechanizm korekty. */
            if (imu.mag_initialized && fabsf(herr) > REALIGN_TRIGGER_DEG) {
                ESP_LOGI(TAG, "Bardzo duzy odchyl od azymutu (blad=%.1f st.) - przerywam odcinek, ponowne wyrownanie.", herr);
                motor_stop();
                enter(ST_ALIGN);
                s_align_confirm = 0;
                break;
            }

            /* Jazda na wprost: impuls rozruchowy, a po nim stała prędkość
             * przelotowa - oba koła równo, bez korekty toru. */
            if (now - s_state_t < KICK_MS) {
                motor_set_left(KICK_POWER);
                motor_set_right(KICK_POWER);
            } else {
                motor_set_left(SP_CRUISE);
                motor_set_right(SP_CRUISE);
            }
            break;
        }

        case ST_BACKUP:
            if (now - s_state_t < BACKUP_MS) {
                int pow = (now - s_state_t < KICK_MS) ? KICK_POWER : SP_BACK;
                motor_set_left(-pow);
                motor_set_right(-pow);
            } else {
                motor_stop();
                enter(ST_SCAN_GAP);
            }
            break;

        case ST_SCAN_GAP:
            motor_stop();
            /* Czekaj na świeże dane z LIDAR-u (co najmniej jeden pełny obrót
             * głowicy) z obecnej, już nieruchomej pozycji (po cofnięciu). */
            if (now - s_state_t < GAP_DWELL_MS) {
                break;
            }
            {
                float desired_rel = heading_error_deg(s_target_azimuth_deg, imu.azimuth_deg);
                int   gap_rel_deg = 0;
                bool  ok = find_gap(desired_rel, GAP_SEARCH_HALF_DEG, &gap_rel_deg);
                if (!ok) {
                    /* Nic w preferowanym zakresie (np. pułapka w kształcie U) -
                     * spróbuj szerzej, zanim się poddamy. */
                    ESP_LOGW(TAG, "Brak szczeliny w zasiegu +/-%d st. - probuje szerszego zakresu (+/-%d st.).",
                             GAP_SEARCH_HALF_DEG, GAP_SEARCH_WIDE_DEG);
                    ok = find_gap(desired_rel, GAP_SEARCH_WIDE_DEG, &gap_rel_deg);
                }
                if (ok) {
                    float gap_abs = fmodf(imu.azimuth_deg + (float)gap_rel_deg, 360.0f);
                    if (gap_abs < 0.0f) gap_abs += 360.0f;
                    s_nav_target_deg = gap_abs;
                    s_in_gap_clear   = true;
                    ESP_LOGI(TAG, "Znaleziono szczeline: %d st. wzgledem przodu (nowy cel odcinka=%.1f) - omijam.",
                             gap_rel_deg, s_nav_target_deg);
                    enter(ST_ALIGN);
                    s_align_confirm = 0;
                } else {
                    ESP_LOGW(TAG, "Brak wystarczajaco szerokiej szczeliny nawet w zasiegu +/-%d st. - zatrzymuje pojazd.",
                             GAP_SEARCH_WIDE_DEG);
                    enter(ST_STOPPED);
                    s_enabled = false;
                }
            }
            break;

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
        s_nav_target_deg = s_target_azimuth_deg;   /* na start celujemy w prawdziwy cel, nie w szczelinę */
        s_gap_retries    = 0;
        s_in_gap_clear   = false;
        enter(ST_ALIGN);
        s_align_confirm = 0;
        s_stopped = false;
        /* Nowy przejazd => nowy log (poprzedni, jeśli nie pobrany, zostaje nadpisany). */
        s_log_n           = 0;
        s_log_full_warned = false;
        s_run_t0          = now_ms();
        s_log_t           = 0;          /* wymuś natychmiastowy pierwszy rekord */
        s_enabled = true;
        ESP_LOGI(TAG, "Autonomia WLACZONA (wyrownanie do azymutu %.1f st., log przejazdu wyzerowany).",
                 s_target_azimuth_deg);
    } else {
        s_enabled = false;
        motor_stop();
        if (s_state != ST_STOPPED) enter(ST_IDLE);
        ESP_LOGI(TAG, "Autonomia WYLACZONA.");
    }
}

bool autonomy_is_enabled(void) { return s_enabled; }

const char *autonomy_state_str(void) { return state_name(s_state); }

void autonomy_set_target_azimuth(float deg) {
    /* Normalizacja do [0,360) - wpisany kąt może wyjść poza zakres. */
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    s_target_azimuth_deg = deg;
    ESP_LOGI(TAG, "Zadany azymut start->meta ustawiony na %.1f st.", s_target_azimuth_deg);
}

float autonomy_get_target_azimuth(void) { return s_target_azimuth_deg; }

/* API logu przejazdu. */
uint32_t autonomy_log_count(void) { return s_log_n; }

bool autonomy_log_get(uint32_t idx, autonomy_log_rec_t *out) {
    if (!out || idx >= s_log_n) return false;
    *out = s_log[idx];
    return true;
}

const char *autonomy_log_state_name(uint8_t state) { return state_name((st_t)state); }
