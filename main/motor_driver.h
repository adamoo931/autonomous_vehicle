#pragma once
#include <stdint.h>
#include "esp_attr.h"

/* Sterownik dwóch silników DC (mostek TB6612FNG): PWM przez LEDC oraz piny
 * kierunkowe. Piny i parametry LEDC w config.h. Moduł pamięta ostatnio
 * ustawione prędkości, dostępne przez motor_get_*_speed(). */

void motor_init(void);

/* Prędkość lewego silnika: -100 (pełny wstecz) .. 0 (stop) .. 100 (pełny przód). */
void motor_set_left(int speed);

/* Prędkość prawego silnika: -100 .. 100. */
void motor_set_right(int speed);

void motor_forward(uint8_t speed);   // jedź do przodu (0-100)
void motor_backward(uint8_t speed);  // jedź do tyłu  (0-100)
void motor_turn_left(uint8_t speed); // obrót w lewo
void motor_turn_right(uint8_t speed);// obrót w prawo
void motor_stop(void);             // zatrzymaj oba silniki

int  motor_get_left_speed(void);
int  motor_get_right_speed(void);
