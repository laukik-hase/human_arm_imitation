#ifndef _NVS_UTILS_H
#define _NVS_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_system.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "constants.h"

esp_err_t nvs_init(void);

bool nvs_does_key_exist(const char *key);

nvs_type_t nvs_get_key_type(const char *key);

esp_err_t nvs_read_short(const char *key, int16_t *value);

esp_err_t nvs_write_short(const char *key, const int16_t *value);

esp_err_t nvs_load_calib_data(void);

esp_err_t nvs_delete_key(const char *key);

#endif
