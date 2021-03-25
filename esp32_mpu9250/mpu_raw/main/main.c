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

void mpu_task(void *param)
{
    mpu_data_t *base = (mpu_data_t *)param;
    uint8_t sensors = UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS;

    // Print all MPU Information - FSR, Rate, Biases
    printMPUInfo(base);

    while (1)
    {
        if (dataReady())
        {
            update(sensors, base);

            ESP_LOGD(TAG, "Ax: %0.3f | Ay: %0.3f | Az: %0.3f m/s2",
                     base->acce_raw[X_AXIS], base->acce_raw[Y_AXIS], base->acce_raw[Z_AXIS]);
            ESP_LOGD(TAG, "Gx: %0.3f | Gy: %0.3f | Gz: %0.3f rad/s",
                     base->gyro_raw[X_AXIS], base->gyro_raw[Y_AXIS], base->gyro_raw[Z_AXIS]);
            ESP_LOGD(TAG, "Mx: %0.3f | My: %0.3f | Mz: %0.3f uT",
                     base->mag_raw[X_AXIS], base->mag_raw[Y_AXIS], base->mag_raw[Z_AXIS]);
            printf("\n");
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());

    mpu_data_t *base = malloc(sizeof(mpu_data_t));
    base->pos = MPU_BASE;

    // uint8_t calib_flag = ACCEL_GYRO_CAL | COMPASS_CAL;
    uint8_t calib_flag = ACCEL_GYRO_CAL;

    if (initMPU(base, calib_flag) != INV_SUCCESS)
        ESP_LOGE(TAG, "Initialization Failure!");
    else
        xTaskCreate(mpu_task, "mpu_task", 4096, (void *)base, 5, NULL);
}