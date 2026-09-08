#include "line_sensor.h"
#include "config.h"
#include "ads1115.h"
#include "esp_log.h"

static const char *TAG = "LINE";

/*
 * Wszystkie cztery czujniki CNY70 idą analogowo przez ADS1115:
 *   A0 przód-prawy, A1 przód-lewy, A2 tył-lewy, A3 tył-prawy.
 * Linia wykryta = napięcie PONIŻEJ progu danego kanału (LINE_*_THRESHOLD_V
 * w config.h). Przy odwrotnym okablowaniu/polaryzacji zmień porównania w
 * line_sensor_read().
 */

void line_sensor_init(void) {
    ESP_LOGI(TAG,
             "Czujniki linii: 4x analogowo przez ADS1115 - wykrycie ponizej progu [V]: "
             "przod-P(A0)=%.2f przod-L(A1)=%.2f tyl-L(A2)=%.2f tyl-P(A3)=%.2f",
             (double)LINE_FR_THRESHOLD_V, (double)LINE_FL_THRESHOLD_V,
             (double)LINE_BL_THRESHOLD_V, (double)LINE_BR_THRESHOLD_V);
}

line_sensor_data_t line_sensor_read(void) {
    ads1115_data_t a = ads1115_get_last();

    line_sensor_data_t d = {
        .front_left    = (a.line_fl_v < LINE_FL_THRESHOLD_V),
        .front_right   = (a.line_fr_v < LINE_FR_THRESHOLD_V),
        .back_left     = (a.line_bl_v < LINE_BL_THRESHOLD_V),
        .back_right    = (a.line_br_v < LINE_BR_THRESHOLD_V),
        .front_left_v  = a.line_fl_v,
        .front_right_v = a.line_fr_v,
        .back_left_v   = a.line_bl_v,
        .back_right_v  = a.line_br_v,
    };
    return d;
}

bool line_sensor_any_edge(void) {
    line_sensor_data_t d = line_sensor_read();
    return d.front_left || d.front_right || d.back_left || d.back_right;
}
