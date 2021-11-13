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

//Macro for error checking
#define IS_ESP_OK(x, label) \
  if ((x) != ESP_OK)        \
    goto label;

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_INT_IO 23        /*!< gpio number for I2C master interrupt */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */

#define ESP_INTR_FLAG_DEFAULT 0
#define TCA9548_ADDR 0x70

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x01          /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x00         /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x00               /*!< I2C ack value */
#define NACK_VAL 0x01              /*!< I2C nack value */

typedef enum
{
  MPU_BASE = 0x00,
  MPU_SHOULDER,
  MPU_BICEP,
  MPU_FOREARM,
  MPU_PALM
} mpu_pos_t;

// Initialise the I2C bus and install driver to specified pins
esp_err_t i2c_master_init(void);

//Check whether the given slave is present on the bus
esp_err_t i2c_check_slave(uint8_t slave_addr);

// Write buffer of size len to slave
int i2c_write_to_slave(uint8_t slave_addr, uint8_t reg_addr, size_t len, uint8_t *data_wr);

// Read buffer of size len to slave
int i2c_read_from_slave(uint8_t slave_addr, uint8_t reg_add, size_t len, uint8_t *data_rd);

// Selecting I2C Channel
int i2c_mux_select(mpu_pos_t joint);

// Clock utility functions
int esp32_get_clock_ms(uint32_t *count);

int esp32_delay_ms(uint32_t num_ms);

#endif