#pragma once
#include <esp_err.h>
#include <stdbool.h>

typedef struct {
    float accel_x, accel_y, accel_z;   // [g]
    float gyro_x,  gyro_y,  gyro_z;    // [°/s]
    float temp;                          // [°C]
    bool  initialized;
} imu_data_t;

esp_err_t imu_init(void);
esp_err_t imu_read(imu_data_t *out);
imu_data_t imu_get_last(void);
