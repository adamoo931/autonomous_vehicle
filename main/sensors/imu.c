#include "imu.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <math.h>

static const char *TAG = "IMU";

/* Rejestry banku 0 (ICM-20948). */
#define REG_BANK_SEL   0x7F
#define REG_WHO_AM_I   0x00
#define REG_USER_CTRL  0x03   /* I2C_MST_RST / I2C_MST_EN                   */
#define REG_LP_CONFIG  0x05   /* I2C_MST_CYCLE                              */
#define REG_PWR_MGMT1  0x06
#define REG_PWR_MGMT2  0x07
#define REG_ACCEL_XOUT 0x2D   /* pierwszy z 6 bajtów akcelerometru          */
#define REG_GYRO_XOUT  0x33   /* pierwszy z 6 bajtów żyroskopu              */
#define REG_TEMP_OUT   0x39   /* 2 bajty temperatury                        */
#define REG_EXT_SLV_SENS_DATA_00 0x3B  /* dane z AK09916 wczytane przez SLV0 */

/* Stała wartość rejestru WHO_AM_I identyfikująca ICM-20948. */
#define ICM_WHO_AM_I_EXPECTED 0xEA

/* Rejestry banku 3 (ICM-20948) - wewnętrzny I2C master, przejście do AK09916. */
#define REG_I2C_MST_ODR_CONFIG 0x00
#define REG_I2C_MST_CTRL        0x01
#define REG_I2C_SLV0_ADDR       0x03
#define REG_I2C_SLV0_REG        0x04
#define REG_I2C_SLV0_CTRL       0x05
#define REG_I2C_SLV0_DO         0x06

/* AK09916 (magnetometr) - slave za wewnętrznym I2C masterem ICM-20948. */
#define AK09916_I2C_ADDR          0x0C
#define AK09916_READ              0x80
#define AK09916_WRITE             0x00
#define MAG_WIA2                  0x01
#define MAG_ST1                   0x10   /* + HXL..HZH, TMPS, ST2 = 9 bajtów */
#define MAG_CNTL2                 0x31
#define MAG_CNTL3                 0x32
#define AK09916_WHO_AM_I_EXPECTED 0x09
#define MAG_UT_PER_LSB            0.15f
#define RAD_TO_DEG                57.29577951308232f

/* Kalibracja magnetometru (hard-iron + faza montażu). MAG_OFFSET_X/Y to
 * stałe przesunięcie pola (zakłócenie od silników/elementów metalowych na
 * płytce, ew. przedmiotów w otoczeniu) - bez niego oś X/Y magnetometru nie
 * jest wyśrodkowana w (0,0), co psuje atan2() nieliniowo w zależności od
 * kierunku. MAG_AZIMUTH_PHASE_DEG koryguje orientację montażu czujnika
 * względem przodu pojazdu. To pole magnetyczne w otoczeniu (a nie tylko
 * sam czujnik) zmienia się przy każdym przestawieniu pojazdu czy zmianie
 * otoczenia, dlatego kalibracja jest teraz wykonywana w czasie pracy
 * (imu_mag_calibration_start()/imu_mag_set_north()) i trzymana w NVS -
 * wartości poniżej to tylko domyślne dane startowe, używane dopóki w NVS
 * nie ma jeszcze zapisanej kalibracji. */
#define MAG_OFFSET_X_DEFAULT          48.0f
#define MAG_OFFSET_Y_DEFAULT          24.25f
#define MAG_AZIMUTH_PHASE_DEG_DEFAULT 102.0f

#define MAG_NVS_NAMESPACE  "imu_mag"
#define MAG_NVS_KEY_OFF_X  "off_x"
#define MAG_NVS_KEY_OFF_Y  "off_y"
#define MAG_NVS_KEY_PHASE  "phase"

static float s_mag_offset_x = MAG_OFFSET_X_DEFAULT;
static float s_mag_offset_y = MAG_OFFSET_Y_DEFAULT;
static float s_mag_phase_deg = MAG_AZIMUTH_PHASE_DEG_DEFAULT;

/* Stan trwającej kalibracji offsetu (krok 1 - zbieranie min/max). */
static volatile bool s_cal_active = false;
static int64_t s_cal_end_us = 0;
static float   s_cal_min_x, s_cal_max_x, s_cal_min_y, s_cal_max_y;

static imu_data_t s_last = {0};
static uint8_t    s_addr = ICM20948_ADDR;   /* stały adres z config.h        */

/* Zapis pojedynczego bajtu do rejestru układu. */
static esp_err_t icm_write(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_write_byte(h, val, true);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return ret;
}

/* Odczyt 'len' kolejnych bajtów począwszy od rejestru 'reg'. */
static esp_err_t icm_read(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, reg, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (s_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(h, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(h, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return ret;
}

/* ICM-20948 dzieli rejestry na 4 banki przełączane przez REG_BANK_SEL. */
static void select_bank(uint8_t bank) {
    icm_write(REG_BANK_SEL, (bank & 0x03) << 4);
}

/* Składa dwa bajty (starszy, młodszy) w liczbę 16-bitową ze znakiem. */
static int16_t to_int16(uint8_t h, uint8_t l) {
    return (int16_t)((uint16_t)h << 8 | l);
}

/* Wczytuje kalibrację magnetometru z NVS, jeśli była wcześniej zapisana.
 * Brak zapisanych danych (pierwsze uruchomienie) nie jest błędem - zostają
 * wartości domyślne. */
static void mag_cal_load(void) {
    nvs_handle_t h;
    if (nvs_open(MAG_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "Brak zapisanej kalibracji magnetometru w NVS - używam wartości domyślnych");
        return;
    }
    size_t sz = sizeof(float);
    bool ok = nvs_get_blob(h, MAG_NVS_KEY_OFF_X, &s_mag_offset_x, &sz) == ESP_OK;
    sz = sizeof(float);
    ok = ok && nvs_get_blob(h, MAG_NVS_KEY_OFF_Y, &s_mag_offset_y, &sz) == ESP_OK;
    sz = sizeof(float);
    ok = ok && nvs_get_blob(h, MAG_NVS_KEY_PHASE, &s_mag_phase_deg, &sz) == ESP_OK;
    nvs_close(h);

    if (ok)
        ESP_LOGI(TAG, "Wczytano kalibrację magnetometru z NVS: offset=(%.2f,%.2f) faza=%.2f",
                 s_mag_offset_x, s_mag_offset_y, s_mag_phase_deg);
    else
        ESP_LOGW(TAG, "Niekompletna kalibracja w NVS - używam wartości domyślnych");
}

/* Zapisuje bieżącą kalibrację magnetometru do NVS. */
static void mag_cal_save(void) {
    nvs_handle_t h;
    if (nvs_open(MAG_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "Nie udało się otworzyć NVS do zapisu kalibracji magnetometru");
        return;
    }
    nvs_set_blob(h, MAG_NVS_KEY_OFF_X, &s_mag_offset_x, sizeof(float));
    nvs_set_blob(h, MAG_NVS_KEY_OFF_Y, &s_mag_offset_y, sizeof(float));
    nvs_set_blob(h, MAG_NVS_KEY_PHASE, &s_mag_phase_deg, sizeof(float));
    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Zapis kalibracji magnetometru do NVS nie powiódł się (%s)", esp_err_to_name(err));
}

/* Zapis pojedynczego rejestru AK09916 przez przejście SLV0 (bank 3). */
static void ak_write(uint8_t reg, uint8_t val) {
    select_bank(3);
    icm_write(REG_I2C_SLV0_ADDR, AK09916_WRITE | AK09916_I2C_ADDR);
    icm_write(REG_I2C_SLV0_REG, reg);
    icm_write(REG_I2C_SLV0_DO, val);
    icm_write(REG_I2C_SLV0_CTRL, 0x81);   /* enable + 1 bajt */
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* Odczyt pojedynczego rejestru AK09916 przez przejście SLV0 (bank 3),
 * wynik trzeba odebrać z EXT_SLV_SENS_DATA_00 w banku 0. */
static uint8_t ak_read(uint8_t reg) {
    select_bank(3);
    icm_write(REG_I2C_SLV0_ADDR, AK09916_READ | AK09916_I2C_ADDR);
    icm_write(REG_I2C_SLV0_REG, reg);
    icm_write(REG_I2C_SLV0_CTRL, 0x81);
    vTaskDelay(pdMS_TO_TICKS(1));

    select_bank(0);
    uint8_t val = 0xFF;
    icm_read(REG_EXT_SLV_SENS_DATA_00, &val, 1);
    return val;
}

/* Uruchamia wewnętrzny I2C master ICM-20948 i magnetometr AK09916, a na
 * koniec konfiguruje SLV0 tak, by przy każdym cyklu mastera automatycznie
 * wczytywał 9 bajtów (ST1..ST2) do EXT_SLV_SENS_DATA_00 w banku 0 -
 * dzięki temu imu_read() czyta magnetometr zwykłym odczytem z banku 0,
 * bez ręcznego sterowania przejściem przy każdej próbce. */
static esp_err_t mag_init(void) {
    mag_cal_load();

    select_bank(0);
    icm_write(REG_USER_CTRL, 0x02);      /* I2C_MST_RST */
    vTaskDelay(pdMS_TO_TICKS(10));
    icm_write(REG_USER_CTRL, 0x20);      /* I2C_MST_EN  */
    vTaskDelay(pdMS_TO_TICKS(100));
    icm_write(REG_LP_CONFIG, 0x40);      /* I2C_MST_CYCLE */
    vTaskDelay(pdMS_TO_TICKS(10));

    select_bank(3);
    icm_write(REG_I2C_MST_CTRL, 0x07);        /* zegar wewn. I2C ~400 kHz */
    icm_write(REG_I2C_MST_ODR_CONFIG, 0x03);  /* ODR mastera I2C          */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Miękki reset AK09916, a po nim weryfikacja WIA2. */
    ak_write(MAG_CNTL3, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t who = ak_read(MAG_WIA2);
    if (who != AK09916_WHO_AM_I_EXPECTED) {
        ESP_LOGW(TAG, "AK09916 nie wykryty (WIA2=0x%02X, oczekiwano 0x%02X)",
                 who, AK09916_WHO_AM_I_EXPECTED);
        return ESP_ERR_NOT_FOUND;
    }

    ak_write(MAG_CNTL2, 0x08);   /* tryb ciągłego pomiaru 100 Hz */
    vTaskDelay(pdMS_TO_TICKS(10));

    select_bank(3);
    icm_write(REG_I2C_SLV0_ADDR, AK09916_READ | AK09916_I2C_ADDR);
    icm_write(REG_I2C_SLV0_REG, MAG_ST1);
    icm_write(REG_I2C_SLV0_CTRL, 0x89);   /* enable + 9 bajtów */
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

esp_err_t imu_init(void) {
    s_last.initialized = false;
    s_addr = ICM20948_ADDR;

    /* Weryfikacja obecności: WHO_AM_I musi zwrócić wartość ICM-20948. */
    select_bank(0);
    uint8_t who = 0;
    if (icm_read(REG_WHO_AM_I, &who, 1) != ESP_OK || who != ICM_WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "ICM-20948 nie wykryty pod 0x%02X (WHO_AM_I=0x%02X, oczekiwano 0x%02X)",
                 s_addr, who, ICM_WHO_AM_I_EXPECTED);
        return ESP_ERR_NOT_FOUND;
    }

    /* Bez sprzętowego resetu (H_RESET/0x80): tuż po jego wyzwoleniu układ
     * przez chwilę nie odpowiada na magistrali (clock stretching / brak
     * ACK), co gubiło kolejne zapisy PWR_MGMT_1/PWR_MGMT_2 mimo poprawnego
     * WHO_AM_I. WHO_AM_I już potwierdza, że komunikacja po starcie zasilania
     * działa, więc wystarczy bezpośrednie wybudzenie ze SLEEP - tak samo jak
     * w sprawdzonym sterowniku referencyjnym (STM32). */
    esp_err_t err = icm_write(REG_PWR_MGMT1, 0x01);   /* zegar automatyczny, wyjście ze SLEEP */
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Zapis PWR_MGMT_1 nie powiódł się (%s)", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(100));

    err = icm_write(REG_PWR_MGMT2, 0x00);   /* akcelerometr + żyroskop włączone    */
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Zapis PWR_MGMT_2 nie powiódł się (%s)", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Odczyt zwrotny PWR_MGMT_1: bit SLEEP (0x40) musi być wyzerowany,
     * inaczej dane akcelerometru/żyroskopu pozostaną zamrożone mimo braku
     * błędów I2C przy kolejnych odczytach. */
    uint8_t pwr1 = 0xFF;
    if (icm_read(REG_PWR_MGMT1, &pwr1, 1) != ESP_OK || (pwr1 & 0x40)) {
        ESP_LOGE(TAG, "ICM-20948 nie wybudzony (PWR_MGMT_1=0x%02X) - czujnik pozostaje w SLEEP",
                 pwr1);
        return ESP_ERR_INVALID_STATE;
    }

    s_last.initialized = true;
    ESP_LOGI(TAG, "ICM-20948 OK pod adresem 0x%02X (WHO_AM_I=0xEA)", s_addr);

    /* Brak magnetometru nie przerywa inicjalizacji - akcelerometr/żyroskop
     * działają niezależnie od niego, azymut po prostu nie będzie dostępny. */
    s_last.mag_initialized = (mag_init() == ESP_OK);
    if (s_last.mag_initialized)
        ESP_LOGI(TAG, "AK09916 (magnetometr) OK");
    else
        ESP_LOGW(TAG, "AK09916 nie wykryty - kontynuuje bez magnetometru/azymutu");

    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out) {
    if (!s_last.initialized) return ESP_ERR_INVALID_STATE;

    select_bank(0);
    uint8_t raw[12];

    /* Akcelerometr (6 bajtów) i żyroskop (6 bajtów) w dwóch transakcjach. */
    esp_err_t ra = icm_read(REG_ACCEL_XOUT, raw,     6);
    esp_err_t rg = icm_read(REG_GYRO_XOUT,  raw + 6, 6);
    if (ra != ESP_OK || rg != ESP_OK) {
        ESP_LOGW(TAG, "Odczyt accel/gyro nie powiódł się (accel=%s, gyro=%s)",
                 esp_err_to_name(ra), esp_err_to_name(rg));
        return ESP_FAIL;
    }

    uint8_t tmp[2];
    icm_read(REG_TEMP_OUT, tmp, 2);

    /* Współczynniki dla zakresów domyślnych: akcelerometr ±2 g (16384 LSB/g),
     * żyroskop ±250 °/s (131 LSB/(°/s)). */
    s_last.accel_x = (float)to_int16(raw[0], raw[1]) / 16384.0f;
    s_last.accel_y = (float)to_int16(raw[2], raw[3]) / 16384.0f;
    s_last.accel_z = (float)to_int16(raw[4], raw[5]) / 16384.0f;
    s_last.gyro_x  = (float)to_int16(raw[6],  raw[7])  / 131.0f;
    s_last.gyro_y  = (float)to_int16(raw[8],  raw[9])  / 131.0f;
    s_last.gyro_z  = (float)to_int16(raw[10], raw[11]) / 131.0f;
    s_last.temp    = (float)to_int16(tmp[0], tmp[1]) / 333.87f + 21.0f;

    /* Magnetometr: SLV0 w mag_init() już skonfigurowany do automatycznego
     * wczytywania ST1..ST2 do EXT_SLV_SENS_DATA_00 przy każdym cyklu
     * wewnętrznego mastera I2C - tu tylko odbieramy gotowe dane z banku 0. */
    if (s_last.mag_initialized) {
        uint8_t mbuf[9];
        if (icm_read(REG_EXT_SLV_SENS_DATA_00, mbuf, 9) == ESP_OK) {
            uint8_t drdy = mbuf[0] & 0x01;   /* ST1: dane gotowe        */
            uint8_t hofl = mbuf[8] & 0x08;   /* ST2: przepełnienie pola */
            if (drdy && !hofl) {
                int16_t raw_x = (int16_t)((uint16_t)mbuf[2] << 8 | mbuf[1]);
                int16_t raw_y = (int16_t)((uint16_t)mbuf[4] << 8 | mbuf[3]);
                int16_t raw_z = (int16_t)((uint16_t)mbuf[6] << 8 | mbuf[5]);

                /* Przeniesienie osi AK09916 na układ akcelerometru/żyroskopu -
                 * tak samo jak w sprawdzonym sterowniku referencyjnym. */
                s_last.mag_x = (float)raw_y * MAG_UT_PER_LSB;
                s_last.mag_y = -(float)raw_x * MAG_UT_PER_LSB;
                s_last.mag_z = (float)raw_z * MAG_UT_PER_LSB;

                /* Krok 1 kalibracji w toku: zbieraj min/max surowych Mx/My
                 * zamiast (albo obok) liczenia azymutu - obrót pojazdu o
                 * pełny obrót w tym czasie pozwala wyznaczyć offset hard-iron
                 * jako środek zakresu każdej osi. */
                if (s_cal_active) {
                    if (s_last.mag_x < s_cal_min_x) s_cal_min_x = s_last.mag_x;
                    if (s_last.mag_x > s_cal_max_x) s_cal_max_x = s_last.mag_x;
                    if (s_last.mag_y < s_cal_min_y) s_cal_min_y = s_last.mag_y;
                    if (s_last.mag_y > s_cal_max_y) s_cal_max_y = s_last.mag_y;

                    if (esp_timer_get_time() >= s_cal_end_us) {
                        s_mag_offset_x = (s_cal_min_x + s_cal_max_x) / 2.0f;
                        s_mag_offset_y = (s_cal_min_y + s_cal_max_y) / 2.0f;
                        s_cal_active = false;
                        mag_cal_save();
                        ESP_LOGI(TAG, "Kalibracja offsetu magnetometru zakończona: offset=(%.2f,%.2f)",
                                 s_mag_offset_x, s_mag_offset_y);
                    }
                }

                /* Azymut bez kompensacji przechyłu - zakłada, że pojazd
                 * porusza się po poziomej powierzchni. Odjęcie offsetu
                 * (hard-iron) przed atan2() jest konieczne - patrz komentarz
                 * przy MAG_OFFSET_X_DEFAULT/MAG_OFFSET_Y_DEFAULT. */
                float mx = s_last.mag_x - s_mag_offset_x;
                float my = s_last.mag_y - s_mag_offset_y;
                float az = s_mag_phase_deg - atan2f(my, mx) * RAD_TO_DEG;
                /* phase (0..360) minus atan2 (-180..180) wykracza poza
                 * [0,360) w obie strony (zakres ok. -180..540) - pojedyncze
                 * "if (az<0) az+=360" łapało tylko dolny przypadek, więc przy
                 * części kierunków azymut potrafił pokazać >360 (np. ~450 na
                 * wschodzie) zamiast zawinąć się do 90. fmodf normalizuje
                 * dowolną wartość, niezależnie od tego, jak daleko wyszła
                 * poza zakres. */
                az = fmodf(az, 360.0f);
                if (az < 0.0f) az += 360.0f;
                s_last.azimuth_deg = az;
            }
        }
    }

    if (out) *out = s_last;
    return ESP_OK;
}

imu_data_t imu_get_last(void) { return s_last; }

void imu_mag_calibration_start(uint32_t duration_ms) {
    if (!s_last.mag_initialized) return;
    if (duration_ms < 3000)  duration_ms = 3000;
    if (duration_ms > 60000) duration_ms = 60000;

    /* Zakres startowy odwrotny do normalnego (min=+inf, max=-inf), tak by
     * pierwsza próbka z imu_read() od razu stała się nowym min i max. */
    s_cal_min_x = s_cal_min_y = 1e9f;
    s_cal_max_x = s_cal_max_y = -1e9f;
    s_cal_end_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    s_cal_active = true;

    ESP_LOGI(TAG, "Start kalibracji offsetu magnetometru (%lu ms) - obróć pojazd o pełny obrót",
             (unsigned long)duration_ms);
}

void imu_mag_set_north(void) {
    if (!s_last.mag_initialized) return;

    float mx = s_last.mag_x - s_mag_offset_x;
    float my = s_last.mag_y - s_mag_offset_y;
    float phase = atan2f(my, mx) * RAD_TO_DEG;
    if (phase < 0.0f) phase += 360.0f;

    s_mag_phase_deg = phase;
    mag_cal_save();
    ESP_LOGI(TAG, "Ustawiono bieżący kierunek jako N (faza=%.2f)", s_mag_phase_deg);
}

imu_mag_cal_status_t imu_mag_get_cal_status(void) {
    imu_mag_cal_status_t st = {
        .active       = s_cal_active,
        .remaining_ms = 0,
        .offset_x     = s_mag_offset_x,
        .offset_y     = s_mag_offset_y,
        .phase_deg    = s_mag_phase_deg,
    };
    if (s_cal_active) {
        int64_t remaining_us = s_cal_end_us - esp_timer_get_time();
        st.remaining_ms = remaining_us > 0 ? (uint32_t)(remaining_us / 1000) : 0;
    }
    return st;
}
