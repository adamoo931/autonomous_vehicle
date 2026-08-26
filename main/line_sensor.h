#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    bool front_left;   // true = krawędź / poza planszą
    bool front_right;
    bool back_left;
    bool back_right;
} line_sensor_data_t;

void line_sensor_init(void);
line_sensor_data_t line_sensor_read(void);
bool line_sensor_any_edge(void);   // którykolwiek sygnalizuje krawędź
bool line_sensor_edge_detected(void); // czy detekcja krawędzi zaszła (ISR)
void line_sensor_clear_edge_flag(void);
void line_sensor_set_notify_task(TaskHandle_t task);

// Zwraca ostatnio wykryty sensor krawędzi (która czujnik FL/FR/BL/BR zadziałał).
line_sensor_data_t line_sensor_get_last_trigger(void);
