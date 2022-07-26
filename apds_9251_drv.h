#ifndef _APDS_9251_DRV_H_
#define _APDS_9251_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "apds_9251_regs.h"

#define APDS_I2C_ADD    0x52
#define CHIP_ID         0xB5

#define I2C_SDA         21 /* Default I2C pins for ESP32-WROOM32 */
#define I2C_SCL         22
#define I2C_FREQ    100000

typedef struct {
    uint8_t ls_meas_rate;
    uint8_t ls_gain;
}apds_9251_config_t;

typedef struct {
    uint8_t ir_data[3];
    uint8_t g_data[3];
    uint8_t b_data[3];
    uint8_t r_data[3];
}__attribute__((packed)) apds_9251_ls_data;

typedef struct {
    uint32_t ir;
    uint32_t green;
    uint32_t red;
    uint32_t blue;
}color_data;

typedef struct {
    uint8_t ir;
    uint8_t green;
    uint8_t red;
    uint8_t blue;
}color_data_8bit;

enum colors {
    IR = 0,
    GREEN,
    RED,
    BLUE
};

bool i2cIsInit(uint8_t i2c_num);
esp_err_t apds_9251_init(uint8_t i2c_num, uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency);
esp_err_t apds_9251_deint(uint8_t i2c_num);
esp_err_t apds_9251_write_byte(uint8_t reg_add, uint8_t data, uint32_t timeOutMillis);
esp_err_t apds_9251_read_byte(uint8_t reg_add, uint8_t *read_data ,uint32_t timeOutMillis);
esp_err_t apds_9251_write(uint8_t reg_add, const uint8_t* buff, size_t size, uint32_t timeOutMillis);
esp_err_t apds_9251_read(uint8_t reg_add, uint8_t *read_data, size_t size, uint32_t timeOutMillis);
uint8_t apds_get_chip_id(void);
esp_err_t apds_enable(void);
esp_err_t apds_config(apds_9251_config_t *config);
esp_err_t apds_read_data(apds_9251_ls_data *ls_data);
uint32_t apds_extract_data(apds_9251_ls_data *ls_data, uint8_t color);
uint8_t apds_extract_data_8bit(apds_9251_ls_data *ls_data, uint8_t color);

#ifdef __cplusplus
}
#endif

#endif /*_APDS_9251_DRV_H_*/