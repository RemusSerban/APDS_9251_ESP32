#include "apds_9251_drv.h"
#include "esp32-hal.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "soc/soc_caps.h"
#include "soc/i2c_periph.h"
#include "hal/i2c_hal.h"
#include "hal/i2c_ll.h"
#include "driver/i2c.h"


typedef volatile struct {
    bool initialized;
    uint32_t frequency;
} i2c_bus_t;

static i2c_bus_t bus[SOC_I2C_NUM];
static uint8_t i2c_num_used = 0;

bool i2cIsInit(uint8_t i2c_num){
    if(i2c_num >= SOC_I2C_NUM){
        return false;
    }
    return bus[i2c_num].initialized;
}

esp_err_t apds_9251_init(uint8_t i2c_num, uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency)
{
    if(i2c_num >= SOC_I2C_NUM){
        return ESP_ERR_INVALID_ARG;
    }

    if(bus[i2c_num].initialized){
        log_e("bus is already initialized");
        return ESP_FAIL;
    }

    if(!frequency){
        frequency = 100000UL;
    } else if(frequency > 1000000UL){
        frequency = 1000000UL;
    }

    i2c_config_t conf = { };
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = (gpio_num_t)scl_pin;
    conf.sda_io_num = (gpio_num_t)sda_pin;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE; 
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; 
    conf.master.clk_speed = frequency;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; //Any one clock source that is available for the specified frequency may be choosen

    esp_err_t ret = i2c_param_config((i2c_port_t)i2c_num, &conf);
    if (ret != ESP_OK) {
        log_e("i2c_param_config failed");
    } else {
        ret = i2c_driver_install((i2c_port_t)i2c_num, conf.mode, 0, 0, 0);
        if (ret != ESP_OK) {
            log_e("i2c_driver_install failed");
        } else {
            bus[i2c_num].initialized = true;
            bus[i2c_num].frequency = frequency;
            i2c_num_used = i2c_num;
            //Clock Stretching Timeout: 20b:esp32, 5b:esp32-c3, 24b:esp32-s2
            i2c_set_timeout((i2c_port_t)i2c_num, I2C_LL_MAX_TIMEOUT);
        }
    }
    return ret;
}

esp_err_t apds_9251_deint(uint8_t i2c_num){
    esp_err_t err = ESP_FAIL;
    if(i2c_num >= SOC_I2C_NUM){
        return ESP_ERR_INVALID_ARG;
    }

    if(!bus[i2c_num].initialized){
        log_e("bus is not initialized");
    } else {
        err = i2c_driver_delete((i2c_port_t)i2c_num);
        if(err == ESP_OK){
            bus[i2c_num].initialized = false;
        }
    }

    return err;
}

esp_err_t apds_9251_write_byte(uint8_t reg_add, uint8_t data, uint32_t timeOutMillis)
{
    uint8_t i2c_data[2];

    i2c_data[0] = reg_add;
    i2c_data[1] = data;

    uint8_t size = sizeof(i2c_data);

    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = NULL;

    ret = ESP_OK;
    uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };
    cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }
    ret = i2c_master_write_byte(cmd, (APDS_I2C_ADD << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        goto end;
    }
    if(size){
        ret = i2c_master_write(cmd, i2c_data, size, true);
        if (ret != ESP_OK) {
            goto end;
        }
    }
    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        goto end;
    }
    ret = i2c_master_cmd_begin((i2c_port_t)i2c_num_used, cmd, timeOutMillis / portTICK_RATE_MS);

end:
    if(cmd != NULL){
        i2c_cmd_link_delete_static(cmd);
    }
    return ret;
}

esp_err_t apds_9251_read_byte(uint8_t reg_add, uint8_t *read_data ,uint32_t timeOutMillis)
{
    esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = NULL;

    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        goto end;
    } 
    
    uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(3)] = { 0 };
    cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(3));
    
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, APDS_I2C_ADD << 1 | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, reg_add, true);
    if(ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, APDS_I2C_ADD << 1 | I2C_MASTER_READ, true);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_read(cmd, read_data, 1, I2C_MASTER_LAST_NACK);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_cmd_begin((i2c_port_t)i2c_num_used, cmd, timeOutMillis / portTICK_RATE_MS);

    end:
        if(cmd != NULL){
            i2c_cmd_link_delete_static(cmd);
        }
        return ret;
}

esp_err_t apds_9251_write(uint8_t reg_add, const uint8_t* buff, size_t size, uint32_t timeOutMillis)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = NULL;

    ret = ESP_OK;
    uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };
    cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }
    ret = i2c_master_write_byte(cmd, (APDS_I2C_ADD << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, reg_add, true);
    if (ret != ESP_OK) {
        goto end;
    }

    if(size){
        ret = i2c_master_write(cmd, buff, size, true);
        if (ret != ESP_OK) {
            goto end;
        }
    }

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        goto end;
    }
    ret = i2c_master_cmd_begin((i2c_port_t)i2c_num_used, cmd, timeOutMillis / portTICK_RATE_MS);

end:
    if(cmd != NULL){
        i2c_cmd_link_delete_static(cmd);
    }
    return ret;
}

esp_err_t apds_9251_read(uint8_t reg_add, uint8_t *read_data, size_t size, uint32_t timeOutMillis)
{
        esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = NULL;

    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        goto end;
    } 
    
    uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(3)] = { 0 };
    cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(3));
    
    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, APDS_I2C_ADD << 1 | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, reg_add, true);
    if(ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_write_byte(cmd, APDS_I2C_ADD << 1 | I2C_MASTER_READ, true);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_read(cmd, read_data, size, I2C_MASTER_LAST_NACK);
    if (ret != ESP_OK) {
        goto end;
    }

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        goto end;
    }
    
    ret = i2c_master_cmd_begin((i2c_port_t)i2c_num_used, cmd, timeOutMillis / portTICK_RATE_MS);

    end:
        if(cmd != NULL){
            i2c_cmd_link_delete_static(cmd);
        }
        return ret;

}

uint8_t apds_get_chip_id(void)
{
    uint8_t chip_id = 0;
    esp_err_t ret;

    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        return ret;
    } 

    ret = apds_9251_read_byte(PART_ID, &chip_id, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error reading chip ID");

    return chip_id;    
}

esp_err_t apds_enable(void)
{
    esp_err_t ret;

    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        return ret;
    } 

    uint8_t data = SW_RESET;

    ret = apds_9251_write_byte(MAIN_CTRL, data, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error writing to chip");

    delay(20);
    
    data = 0x00;
    ret = apds_9251_write_byte(MAIN_CTRL, data, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error writing to chip");
    
    delay(20);

    data = CS_ON | LS_EN;
    ret = apds_9251_write_byte(MAIN_CTRL, data, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error writing to chip");

    return ret;
}

esp_err_t apds_config(apds_9251_config_t *config)
{
    esp_err_t ret;

    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        return ret;
    } 

    uint8_t data;

    data = config->ls_meas_rate;
    ret = apds_9251_write_byte(LS_MEAS_RATE, data, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error writing to chip");

    data = config->ls_gain;
    ret = apds_9251_write_byte(LS_GAIN, data, portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error writing to chip");


    return ret;
}

esp_err_t apds_read_data(apds_9251_ls_data *ls_data)
{
    esp_err_t ret;
    if(!bus[i2c_num_used].initialized){
        log_e("bus is not initialized");
        return ret;
    } 

    ret = apds_9251_read(LS_DATA_IR_0, &ls_data->ir_data[0], sizeof(apds_9251_ls_data), portMAX_DELAY);
    if(ret != ESP_OK)
        log_e("Error reading from chip");

    return ret;
}

uint32_t apds_extract_data(apds_9251_ls_data *ls_data, uint8_t color)
{
    uint32_t color_data = 0;

    if (color == IR) {
        color_data = (ls_data->ir_data[2] << 16) | (ls_data->ir_data[1] << 8) | ls_data->ir_data[0];
    }
    else if (color == GREEN) {
        color_data = (ls_data->g_data[2] << 16) | (ls_data->g_data[1] << 8) | ls_data->g_data[0];
    }
    else if (color == RED) {
        color_data = (ls_data->r_data[2] << 16) | (ls_data->r_data[1] << 8) | ls_data->r_data[0];        
    }
    else if (color == BLUE) {
        color_data = (ls_data->b_data[2] << 16) | (ls_data->b_data[1] << 8) | ls_data->b_data[0];
    }
    else {
        log_e("Error, incorect parameter");
    }
        
    return color_data;
}

uint8_t apds_extract_data_8bit(apds_9251_ls_data *ls_data, uint8_t color)
{
    uint8_t color_data = 0;
    uint32_t temp;

    if (color == IR) {
        temp = (ls_data->ir_data[2] << 16) | (ls_data->ir_data[1] << 8) | ls_data->ir_data[0];
        color_data = temp/32;
    }
    else if (color == GREEN) {
        temp = (ls_data->g_data[2] << 16) | (ls_data->g_data[1] << 8) | ls_data->g_data[0];
        color_data = temp/32;
    }
    else if (color == RED) {
        temp = (ls_data->r_data[2] << 16) | (ls_data->r_data[1] << 8) | ls_data->r_data[0];
        color_data = temp/32;        
    }
    else if (color == BLUE) {
        temp = (ls_data->b_data[2] << 16) | (ls_data->b_data[1] << 8) | ls_data->b_data[0];
        color_data = temp/32;
    }
    else {
        log_e("Error, incorect parameter");
    }
        
    return color_data;
}
