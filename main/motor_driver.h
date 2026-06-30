#pragma once
#include <stdint.h>

/* Sterownik dwóch silników DC (mostek TB6612FNG): PWM przez LEDC oraz piny
 * kierunkowe. Piny i parametry LEDC w config.h. Moduł pamięta ostatnio
 * ustawione prędkości, dostępne przez motor_get_*_speed(). */

void motor_init(void);

/* Prędkość lewego silnika: -100 (pełny wstecz) .. 0 (stop) .. 100 (pełny przód). */
void motor_set_left(int speed);

/* Prędkość prawego silnika: -100 .. 100. */
void motor_set_right(int speed);

void motor_forward(uint8_t speed);    /* jazda do przodu (0-100)  */
void motor_backward(uint8_t speed);   /* jazda do tyłu   (0-100)  */
void motor_turn_left(uint8_t speed);  /* skręt w lewo             */
void motor_turn_right(uint8_t speed); /* skręt w prawo            */
void motor_stop(void);                /* zatrzymanie obu silników */

int  motor_get_left_speed(void);
int  motor_get_right_speed(void);
