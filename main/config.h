#pragma once
#include <stdint.h>

// ============================================================
//  Silniki  (zakładamy sterownik TB6612FNG lub podobny)
// ============================================================
#define PIN_PWMA        19      // Motor A – lewy  – PWM
#define PIN_AIN1         5      // Motor A – lewy  – kierunek 1
#define PIN_AIN2        18      // Motor A – lewy  – kierunek 2

// UWAGA: GPIO0 = pin BOOT! Nie trzymaj go LOW podczas resetu.
#define PIN_PWMB         0      // Motor B – prawy – PWM
#define PIN_BIN1         4      // Motor B – prawy – kierunek 1
#define PIN_BIN2        16      // Motor B – prawy – kierunek 2

// ============================================================
//  Czujniki linii CNY70
// ============================================================
#define PIN_LINE_FL     14      // Przód lewy
#define PIN_LINE_FR     39      // Przód prawy  (input-only GPIO, brak pull-up)
#define PIN_LINE_BL     15      // Tył  lewy
#define PIN_LINE_BR     23      // Tył  prawy

// ============================================================
//  Czujniki Halla
// ============================================================
#define PIN_HALL_LEFT    2      // Odometria – lewe koło
#define PIN_HALL_RIGHT  36      // Odometria – prawe koło (SENSOR_VP, input-only)
#define PIN_HALL_FINISH 34      // Wykrywanie mety          (input-only)

// ============================================================
//  I2C – wspólna magistrala (IMU + pirometr)
// ============================================================
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define I2C_PORT        I2C_NUM_0
// WAZNE: MLX90614 to SMBus i pracuje MAX 100 kHz. Przy 400 kHz pirometr
// nie odpowiada i potrafi zablokowac CALA magistrale (wtedy IMU/INA/SHT
// tez "nie dzialaja"). 100 kHz jest bezpieczne dla wszystkich czujnikow.
#define I2C_FREQ_HZ     100000

// ============================================================
//  LIDAR – UART
//  !! GPIO1=UART0_TX, GPIO3=UART0_RX – te same co USB-Serial !!
//  Podłączenie LIDARa fizycznie wyłącza monitor szeregowy.
//  Ustaw LIDAR_ENABLED 0 podczas debugowania przez USB-Serial.
// ============================================================
// UWAGA: LIDAR na GPIO1/3 = te same piny co USB-Serial (UART0).
// Uzywamy UART_NUM_2 + GPIO matrix (omija konflikt z IO MUX UART0).
// Przypisanie pinow (POTWIERDZONE dzialajacym kodem):
//   ESP TX (komendy do lidara) = GPIO3
//   ESP RX (dane z lidara)     = GPIO1
// Monitor szeregowy USB przestaje dzialac – uzyj monitora webowego.
#define LIDAR_ENABLED    1
#define PIN_LIDAR_TX     3     // ESP TX → LIDAR RX (komendy)
#define PIN_LIDAR_RX     1      // LIDAR TX → ESP RX (dane!)
#define LIDAR_UART_PORT  UART_NUM_2
#define LIDAR_BAUD_RATE  230400

// ============================================================
//  LEDs
// ============================================================
#define PIN_RED_LED     13
#define PIN_YELLOW_LED  12
#define PIN_GREEN_LED   26

// ============================================================
//  Odometria
// ============================================================
#define WHEEL_DIAMETER_MM       55.0f
#define PULSES_PER_REVOLUTION    1      // Liczba magnesów na kole
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265f)
#define MM_PER_PULSE            (WHEEL_CIRCUMFERENCE_MM / PULSES_PER_REVOLUTION)

// ============================================================
//  WiFi / HTTP
// ============================================================
#define WIFI_SSID   "Test2"
#define WIFI_PASS   "12345678"
#define HTTP_PORT    80

// ============================================================
//  I2C adresy
// ============================================================
#define MLX90614_ADDR  0x5A     // pirometr (domyślny)
#define ICM20948_ADDR  0x68     // IMU (AD0=GND→0x68, AD0=VCC→0x69; auto-probe)

// INA219 – wg schematu A0/A1 podciagniete do VBAT (R21/R22 4.7k) => 0x45.
// SHT40 zajmuje 0x44, wiec INA NIE moze tam byc. Driver i tak sam wykryje
// adres (lista kandydatow w ina219.c) i wypisze go na monitorze webowym.
#define INA219_ADDR        0x45
#define INA219_SHUNT_OHMS  0.05f   // R13 = 50 mOhm (ZWERYFIKUJ z BOM!)

// SHT40 – adres staly 0x44.
#define SHT40_ADDR         0x44

// ============================================================
//  Buzzer pasywny (tranzystor BC847 -> aktywny stanem WYSOKIM)
//  !! Numeru GPIO nie dalo sie pewnie odczytac ze schematu (etykieta
//     "ESP32_GPIOxx" w niskiej rozdzielczosci). USTAW ponizej zgodnie
//     z wlasnym projektem KiCad – tam net bazy R41 jest czytelny.
//     GPIO33 to bezpieczny domyslny pin (wolny, nie strapping).
// ============================================================
#define PIN_BUZZER         33
#define BUZZER_LEDC_TIMER    LEDC_TIMER_1     // inny niz silniki (TIMER_0)
#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_2   // inny niz silniki (0,1)

// ============================================================
//  LEDC (PWM silników)
// ============================================================
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1
#define LEDC_RESOLUTION     LEDC_TIMER_10_BIT  // 0-1023
#define LEDC_FREQ_HZ        1000

// ============================================================
//  Próg temperatury mety
// ============================================================
#define PYROMETER_FINISH_THRESHOLD_C  30.0f
