#pragma once
#include <stdint.h>

/**
 * Inicjalizacja sterownika silników (LEDC PWM + GPIO kierunki).
 */
void motor_init(void);

/**
 * Ustaw prędkość lewego silnika.
 * @param speed  -100 (pełny wstecz) .. 0 (stop) .. 100 (pełny przód)
 */
void motor_set_left(int speed);

/**
 * Ustaw prędkość prawego silnika.
 * @param speed  -100 .. 100
 */
void motor_set_right(int speed);

void motor_forward(uint8_t speed);   // jedź do przodu (0-100)
void motor_backward(uint8_t speed);  // jedź do tyłu  (0-100)
void motor_turn_left(uint8_t speed); // obrót w lewo
void motor_turn_right(uint8_t speed);// obrót w prawo
void motor_stop(void);               // zatrzymaj oba silniki

int  motor_get_left_speed(void);
int  motor_get_right_speed(void);
