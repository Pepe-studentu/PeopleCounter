#include "amg8833.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/* AMG8833 register addresses */
#define REG_PCTL  0x00   /* power control */
#define REG_RST   0x01   /* reset */
#define REG_FPSC  0x02   /* frame rate */
#define REG_SCLR  0x05   /* status clear */
#define REG_T01L  0x80   /* pixel output base (low byte of pixel 1) */

#define I2C_TIMEOUT_MS  100

static esp_err_t write_reg(i2c_port_t port, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, AMG_I2C_ADDR, buf, 2,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

bool amg_init(i2c_port_t port)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = AMG_I2C_SDA,
        .scl_io_num       = AMG_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AMG_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));

    /* probe: check the sensor ACKs */
    uint8_t probe_reg = REG_PCTL;
    esp_err_t ret = i2c_master_write_to_device(port, AMG_I2C_ADDR,
                                               &probe_reg, 1,
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) return false;

    write_reg(port, REG_RST,  0x3F);          /* full reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    write_reg(port, REG_SCLR, 0x0E);          /* clear status flags */
    vTaskDelay(pdMS_TO_TICKS(10));
    write_reg(port, REG_PCTL, 0x00);          /* normal mode */
    write_reg(port, REG_FPSC, AMG_FRAME_RATE_HZ == 1 ? 0x01 : 0x00);

    return true;
}

bool amg_read_pixels(i2c_port_t port, float out[64])
{
    /* Read all 128 bytes (64 pixels × 2 bytes) in four 32-byte chunks */
    uint8_t buf[128];

    for (int chunk = 0; chunk < 4; chunk++) {
        uint8_t reg_addr = (uint8_t)(REG_T01L + chunk * 32);
        esp_err_t ret = i2c_master_write_read_device(
            port, AMG_I2C_ADDR,
            &reg_addr, 1,
            buf + chunk * 32, 32,
            pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if (ret != ESP_OK) return false;
    }

    for (int i = 0; i < 64; i++) {
        /* 12-bit signed (two's complement), LSB in buf[2i], bits 11:8 in buf[2i+1] bits 3:0 */
        int16_t raw = ((int16_t)(buf[2*i+1] & 0x07) << 8) | buf[2*i];
        if (buf[2*i+1] & 0x08) raw |= (int16_t)0xF800; /* sign-extend */
        out[i] = raw * 0.25f;
    }
    return true;
}
