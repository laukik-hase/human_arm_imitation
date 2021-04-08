#ifndef _I2C_UTILS_H
#define _I2C_UTILS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_attr.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "constants.h"
#include "bno055.h"

typedef enum
{
  // MPU_BASE = 0x00,
  MPU_SHOULDER = 0x01,
  MPU_BICEP,
  MPU_FOREARM,
  MPU_PALM
} mpu_pos_t;

// Initialise the I2C bus and install driver to specified pins
esp_err_t i2c_master_init(void);

//Check whether the given slave is present on the bus
esp_err_t i2c_check_slave(uint8_t slave_addr);

// Selecting I2C Channel
esp_err_t i2c_mux_select(mpu_pos_t joint);

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

void BNO055_delay_msek(u32 msek);

#endif
