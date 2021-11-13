#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_utils.h"
#include "mpu9250.h"

static const char *TAG = "mpu_test";

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());

    mpu_data_t *base = malloc(sizeof(mpu_data_t));
    base->pos = MPU_BASE;

    uint8_t calib_flag = ACCEL_GYRO_CAL | COMPASS_CAL;

    if (initMPU(base, calib_flag) == INV_SUCCESS)
        printMPUInfo(base);
    else
        ESP_LOGE(TAG, "Initialization Failure!");
}