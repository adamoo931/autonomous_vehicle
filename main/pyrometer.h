#pragma once
#include <esp_err.h>
#include <stdbool.h>

typedef struct {
    float ambient_temp;   // °C
    float object_temp;    // °C
    bool  finish_detected;
    bool  initialized;
} pyrometer_data_t;

esp_err_t pyrometer_init(void);
esp_err_t pyrometer_read(pyrometer_data_t *out);
pyrometer_data_t pyrometer_get_last(void);
