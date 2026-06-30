#pragma once
#include <stdint.h>

/*
 * config.h — centralna konfiguracja sprzętowa pojazdu.
 *
 * Wszystkie numery pinów GPIO, adresy I2C, parametry magistrali i stałe
 * kalibracyjne są zebrane w jednym miejscu. Moduły (sterownik silników,
 * czujniki, sieć) korzystają wyłącznie z tych definicji — żaden pin ani
 * adres nie jest zaszyty bezpośrednio w kodzie sterowników.
 */

/* ============================================================
 *  Silniki — mostek H typu TB6612FNG (2 kanały: lewy A / prawy B)
 * ============================================================ */
#define PIN_PWMA        19      /* kanał A (lewy)  — sygnał PWM prędkości   */
#define PIN_AIN1         5      /* kanał A (lewy)  — wejście kierunku 1     */
#define PIN_AIN2        18      /* kanał A (lewy)  — wejście kierunku 2     */

/* GPIO0 jest pinem strapping (BOOT). Stan niski podczas resetu wprowadza
 * ESP32 w tryb bootloadera, więc nie może być trzymany w dół na starcie. */
#define PIN_PWMB         0      /* kanał B (prawy) — sygnał PWM prędkości   */
#define PIN_BIN1         4      /* kanał B (prawy) — wejście kierunku 1     */
#define PIN_BIN2        16      /* kanał B (prawy) — wejście kierunku 2     */

/* ============================================================
 *  Czujniki linii CNY70 (odbiciowe) — wykrywanie krawędzi planszy
 * ============================================================ */
#define PIN_LINE_FL     14      /* przód-lewy                               */
#define PIN_LINE_FR     39      /* przód-prawy  (GPIO tylko-wejście)        */
#define PIN_LINE_BL     15      /* tył-lewy                                 */
#define PIN_LINE_BR     23      /* tył-prawy                                */

/* ============================================================
 *  Czujniki Halla — odometria kół + detekcja mety
 * ============================================================ */
#define PIN_HALL_LEFT    2      /* enkoder lewego koła                      */
#define PIN_HALL_RIGHT  36      /* enkoder prawego koła (GPIO tylko-wejście)*/
#define PIN_HALL_FINISH 34      /* czujnik mety        (GPIO tylko-wejście) */

/* ============================================================
 *  Magistrala I2C — wspólna dla wszystkich czujników I2C/SMBus
 * ============================================================ */
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define I2C_PORT        I2C_NUM_0

/* Pirometr MLX90614 to układ SMBus pracujący maksymalnie z zegarem 100 kHz.
 * Przy 400 kHz nie odpowiada i może zablokować całą magistralę, przez co
 * pozostałe czujniki również przestają działać. 100 kHz jest bezpieczne
 * dla wszystkich urządzeń na tej magistrali. */
#define I2C_FREQ_HZ     100000

/* Tylko-wejściowe GPIO 34/36/39 nie mają wewnętrznych rezystorów
 * podciągających — wymagają zewnętrznych rezystorów pull-up. */

/* ============================================================
 *  LIDAR (LD06 / LD14P) — interfejs UART
 * ============================================================
 *  Głowica jest podłączona do GPIO1 (UART0_TX) i GPIO3 (UART0_RX), czyli
 *  do tych samych pinów, których ESP32 używa dla konsoli USB-Serial.
 *  Sterownik (sensors/lidar.c) odłącza je od UART0 i przypina do UART2
 *  przez macierz GPIO. Skutek: konsola szeregowa po USB przestaje działać,
 *  dlatego logi są dostępne przez monitor webowy (zob. web_monitor.c).
 *  Ustaw LIDAR_ENABLED na 0, aby debugować przez konsolę USB-Serial.
 */
#define LIDAR_ENABLED    1
#define PIN_LIDAR_TX     3      /* ESP TX -> RX lidaru (komendy)            */
#define PIN_LIDAR_RX     1      /* ESP RX <- TX lidaru (dane pomiarowe)     */
#define LIDAR_UART_PORT  UART_NUM_2
#define LIDAR_BAUD_RATE  230400

/* ============================================================
 *  Diody sygnalizacyjne
 * ============================================================ */
#define PIN_RED_LED     13
#define PIN_YELLOW_LED  12
#define PIN_GREEN_LED   26

/* ============================================================
 *  Odometria — przeliczenie impulsów Halla na drogę
 * ============================================================ */
#define WHEEL_DIAMETER_MM       55.0f
#define PULSES_PER_REVOLUTION    1      /* liczba magnesów na kole          */
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265f)
#define MM_PER_PULSE            (WHEEL_CIRCUMFERENCE_MM / PULSES_PER_REVOLUTION)

/* ============================================================
 *  Sieć — WiFi (tryb stacji) i serwer HTTP
 * ============================================================ */
#define WIFI_SSID   "Test2"
#define WIFI_PASS   "12345678"
#define HTTP_PORT    80

/* ============================================================
 *  Adresy I2C — stałe, zweryfikowane na docelowym sprzęcie
 * ============================================================ */
#define MLX90614_ADDR   0x5A    /* pirometr MLX90614 (SMBus)                */
#define ICM20948_ADDR   0x69    /* IMU ICM-20948 (AD0 = VCC)                */
#define INA219_ADDR     0x40    /* monitor prądu/napięcia (A0 = A1 = GND)   */
#define SHT40_ADDR      0x44    /* czujnik temperatury i wilgotności        */

/* Rezystancja bocznika pomiarowego INA219 (R13) w omach. */
#define INA219_SHUNT_OHMS  0.05f

/* ============================================================
 *  Buzzer pasywny — sterowany tranzystorem BC847, ton generuje LEDC
 * ============================================================ */
#define PIN_BUZZER         25
#define BUZZER_LEDC_TIMER    LEDC_TIMER_1     /* inny timer niż silniki      */
#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_2   /* inny kanał niż silniki      */

/* ============================================================
 *  LEDC — generator PWM dla silników
 * ============================================================ */
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1
#define LEDC_RESOLUTION     LEDC_TIMER_10_BIT  /* rozdzielczość 10 bit (0-1023) */
#define LEDC_FREQ_HZ        1000

/* ============================================================
 *  Próg detekcji mety przez pirometr (temperatura obiektu)
 * ============================================================ */
#define PYROMETER_FINISH_THRESHOLD_C  30.0f
