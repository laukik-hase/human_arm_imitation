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

#define I2C_MASTER_SCL_IO 23        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 22        /*!< gpio number for I2C master data  */
#define I2C_MASTER_INT_IO 21        /*!< gpio number for I2C master interrupt */
#define ESP_INTR_FLAG_DEFAULT 0
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x01           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x00          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x00                /*!< I2C ack value */
#define NACK_VAL 0x01               /*!< I2C nack value */

#define ALPHA 0.9834
#define PI 3.1415926
#define RAD_TO_DEG 57.2957795

//Initialise the I2C bus and install driver to specified pins
esp_err_t i2c_master_init(void);

// Write buffer of size len to slave
int i2c_write_to_slave(uint8_t slave_addr, uint8_t reg_addr, size_t len, uint8_t *data_wr);

// Read buffer of size len to slave
int i2c_read_from_slave(uint8_t slave_addr, uint8_t reg_add, size_t len, uint8_t *data_rd);

// Combine two 8-byte values to a 16-byte one
void combine_msb_lsb_raw_data(uint8_t *buf_1, int16_t *buf_2);

// Clock utility functions

int esp32_get_clock_ms(unsigned long *count);

int esp32_delay_ms(unsigned long num_ms);

#endif