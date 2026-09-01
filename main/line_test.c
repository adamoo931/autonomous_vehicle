#include "line_test.h"
#include "motor_driver.h"
#include "line_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdint.h>

static const char *TAG = "LINETEST";

/* Parametry cyklu skokowej jazdy (wg specyfikacji testu). */
#define LT_SPEED     25    /* prędkość jazdy [% mocy] */
#define LT_MOVE_MS   300   /* czas jazdy w jednym cyklu [ms] */
#define LT_PAUSE_MS  300   /* czas postoju w jednym cyklu [ms] */
/* Okres pętli - na tyle krótki, żeby linia zatrzymywała pojazd
 * praktycznie natychmiast, także w środku fazy jazdy. */
#define LT_LOOP_MS   20

typedef enum {
    LT_IDLE,          /* wyłączony */
    LT_RUN_FWD,       /* cykl jazda/postój do przodu */
    LT_RUN_BACK,      /* cykl jazda/postój do tyłu */
    LT_STOPPED_LINE,  /* zatrzymany całkowicie po wykryciu linii */
} lt_state_t;

static volatile bool       s_enabled     = false;
static volatile bool       s_forward     = true;
static volatile lt_state_t s_state       = LT_IDLE;
static TaskHandle_t        s_task        = NULL;
static bool                s_motors_idle = true;
static uint32_t            s_run_t0      = 0;   /* ms startu bieżącego testu */
static char                s_hit[48]     = "";  /* czujniki, które zatrzymały test */

static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char *state_name(lt_state_t s) {
    switch (s) {
        case LT_IDLE:         return "Bezczynny";
        case LT_RUN_FWD:      return "Jazda do przodu";
        case LT_RUN_BACK:     return "Jazda do tylu";
        case LT_STOPPED_LINE: return "Zatrzymany (linia)";
        default:              return "?";
    }
}

/* Buduje opis czujników, które sygnalizują linię (np. "przod-L tyl-P "). */
static void format_hits(const line_sensor_data_t *ls, char *out, size_t cap) {
    snprintf(out, cap, "%s%s%s%s",
             ls->front_left  ? "przod-L " : "",
             ls->front_right ? "przod-P " : "",
             ls->back_left   ? "tyl-L "   : "",
             ls->back_right  ? "tyl-P "   : "");
}

static void line_test_task(void *arg) {
    (void)arg;

    while (1) {
        /* Tryb wyłączony: pilnuj, by silniki stały. Stan LT_STOPPED_LINE
         * (jeśli to on wyłączył test) zostaje widoczny na dashboardzie do
         * czasu ponownego uruchomienia - żeby było wiadomo, że zadziałała
         * linia, a nie ręczny stop. */
        if (!s_enabled) {
            if (!s_motors_idle) { motor_stop(); s_motors_idle = true; }
            if (s_state != LT_STOPPED_LINE) s_state = LT_IDLE;
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        s_motors_idle = false;

        /* Wykrycie linii którymkolwiek czujnikiem => całkowity stop i koniec
         * testu. Sprawdzane jako pierwsze, żeby zadziałało też w trakcie
         * fazy jazdy, nie tylko w postoju. */
        line_sensor_data_t ls = line_sensor_read();
        if (ls.front_left || ls.front_right || ls.back_left || ls.back_right) {
            motor_stop();
            s_motors_idle = true;
            format_hits(&ls, s_hit, sizeof(s_hit));
            s_state   = LT_STOPPED_LINE;
            s_enabled = false;
            ESP_LOGI(TAG, "Linia wykryta - czujnik(i): %s- pojazd zatrzymany, test zakonczony.", s_hit);
            continue;
        }

        /* Cykl: LT_MOVE_MS jazdy z prędkością LT_SPEED, potem LT_PAUSE_MS
         * postoju. Faza liczona z modulo czasu od startu testu. */
        uint32_t phase = (now_ms() - s_run_t0) % (LT_MOVE_MS + LT_PAUSE_MS);
        if (phase < LT_MOVE_MS) {
            int pow = s_forward ? LT_SPEED : -LT_SPEED;
            motor_set_left(pow);
            motor_set_right(pow);
            s_state = s_forward ? LT_RUN_FWD : LT_RUN_BACK;
        } else {
            motor_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(LT_LOOP_MS));
    }
}

void line_test_init(void) {
    if (s_task) return;
    xTaskCreatePinnedToCore(line_test_task, "line_test", 3072, NULL, 6, &s_task, 1);
    ESP_LOGI(TAG, "Modul testu wykrywania linii gotowy (wylaczony). Cykl: %d%% przez %d ms / postoj %d ms.",
             LT_SPEED, LT_MOVE_MS, LT_PAUSE_MS);
}

void line_test_start(bool forward) {
    s_forward     = forward;
    s_hit[0]      = '\0';
    s_run_t0      = now_ms();
    s_motors_idle = false;
    s_state       = forward ? LT_RUN_FWD : LT_RUN_BACK;
    s_enabled     = true;
    ESP_LOGI(TAG, "Test wykrywania linii WLACZONY (kierunek: %s).",
             forward ? "do przodu" : "do tylu");
}

void line_test_stop(void) {
    bool was_on = s_enabled;
    s_enabled = false;
    motor_stop();
    s_motors_idle = true;
    if (s_state != LT_STOPPED_LINE) s_state = LT_IDLE;
    if (was_on) ESP_LOGI(TAG, "Test wykrywania linii WYLACZONY (przerwany recznie).");
}

bool line_test_is_running(void) { return s_enabled; }

bool line_test_get_forward(void) { return s_forward; }

const char *line_test_state_str(void) { return state_name(s_state); }

const char *line_test_hit_str(void) { return s_hit; }
