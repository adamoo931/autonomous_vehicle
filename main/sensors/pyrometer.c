#include "pyrometer.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "PYRO";

/* Komendy odczytu RAM MLX90614 (protokół SMBus). */
#define MLX_CMD_TA   0x06   /* temperatura otoczenia (ambient)  */
#define MLX_CMD_TOBJ 0x07   /* temperatura obiektu   (object)   */

static pyrometer_data_t s_last = {0};

/* Odczyt jednego słowa (16 bit + bajt PEC) operacją SMBus Read Word.
 * Dane przychodzą little-endian: najpierw młodszy bajt, potem starszy. */
static esp_err_t mlx_read_word(uint8_t cmd, uint16_t *out) {
    uint8_t buf[3] = {0};
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (MLX90614_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, cmd, true);
    i2c_master_start(h);
    i2c_master_write_byte(h, (MLX90614_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(h, buf, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(h);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    if (ret == ESP_OK) {
        *out = (uint16_t)(buf[1] << 8 | buf[0]);
    }
    return ret;
}

/* Surowy wynik MLX90614 ma rozdzielczość 0,02 K i punkt zerowy w 0 K. */
static float raw_to_celsius(uint16_t raw) {
    return (float)(raw & 0x7FFF) * 0.02f - 273.15f;
}

esp_err_t pyrometer_init(void) {
    uint16_t val;
    esp_err_t ret = mlx_read_word(MLX_CMD_TA, &val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MLX90614 nie odpowiada pod 0x%02X (err=%d)", MLX90614_ADDR, ret);
        s_last.initialized = false;
        return ret;
    }
    s_last.initialized = true;
    ESP_LOGI(TAG, "MLX90614 OK - otoczenie=%.1f C", raw_to_celsius(val));
    return ESP_OK;
}

esp_err_t pyrometer_read(pyrometer_data_t *out) {
    uint16_t ta, tobj;
    esp_err_t r1 = mlx_read_word(MLX_CMD_TA,   &ta);
    esp_err_t r2 = mlx_read_word(MLX_CMD_TOBJ, &tobj);
    if (r1 != ESP_OK || r2 != ESP_OK) return ESP_FAIL;
    s_last.ambient_temp    = raw_to_celsius(ta);
    s_last.object_temp     = raw_to_celsius(tobj);
    s_last.finish_detected = (s_last.object_temp >= PYROMETER_FINISH_THRESHOLD_C);
    if (out) *out = s_last;
    return ESP_OK;
}

pyrometer_data_t pyrometer_get_last(void) { return s_last; }
