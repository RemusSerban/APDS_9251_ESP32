#include <Arduino.h>
#include "apds_9251_drv.h"
#include <driver/i2c.h>
#include "analogWrite.h"


apds_9251_config_t conf;
apds_9251_ls_data ls_colors;
color_data colors;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  esp_err_t ret;

  ret = apds_9251_init(I2C_NUM_0, I2C_SDA, I2C_SCL, I2C_FREQ);
  if(ret != ESP_OK)
    Serial.println("Failed to init APDS"); 

  uint8_t chip_id;
  chip_id = apds_get_chip_id();
  if(chip_id != CHIP_ID)
    Serial.println("Wrong Chip_ID");
  
  conf.ls_gain = LS_GAIN1;
  conf.ls_meas_rate = LS_RESOLUTION_18B_100MS | LS_MES_RATE_200MS;

  ret = apds_config(&conf);
  if(ret != ESP_OK)
    Serial.println("Failed to config APDS"); 

  ret = apds_enable();
  if(ret != ESP_OK)
    Serial.println("Failed to enable APDS"); 
  
  delay(40);
  Serial.println("Program start");
}

void loop() {
  esp_err_t err;

  err = apds_read_data(&ls_colors);
  color_opt.green = apds_extract_data(&ls_colors, GREEN);
  color_opt.red = apds_extract_data(&ls_colors, RED);
  color_opt.blue = apds_extract_data(&ls_colors, BLUE);
  colors.ir = apds_extract_data(&ls_colors, IR);
  Serial.print("R: ");
  Serial.print(colors.red);
  Serial.print(" G:");
  Serial.print(colors.green);
  Serial.print(" B:");
  Serial.print(colors.blue);
  Serial.print(" IR:");
  Serial.println(colors.ir);
  
  delay(220);
}

