#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t pulses_left;
    uint32_t pulses_right;
    float    dist_left_mm;
    float    dist_right_mm;
    float    dist_total_mm;
    bool     finish_detected;   // czujnik Halla na mecie
} odometry_data_t;

void odometry_init(void);
void odometry_reset(void);
odometry_data_t odometry_get(void);
