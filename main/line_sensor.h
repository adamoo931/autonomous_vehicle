#pragma once
#include <stdbool.h>

typedef struct {
    bool front_left;   // true = krawędź / poza planszą
    bool front_right;
    bool back_left;
    bool back_right;
} line_sensor_data_t;

void line_sensor_init(void);
line_sensor_data_t line_sensor_read(void);
bool line_sensor_any_edge(void);   // którykolwiek sygnalizuje krawędź
