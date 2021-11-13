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

    loadCompassOffsets(base);
    printMPUInfo(base);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1)
    {
        MadgwickFilter(base);
        computeEulerAngles(base, true);

        // Quaternions
        // ESP_LOGI(TAG, "Qw: %f | Qx: %f | Qy: %f | Qz: %f",
        //          base->qw, base->qx, base->qy, base->qz);
        
        // Euler Angles (Tait-Bryan Notation)
        ESP_LOGI(TAG, "Roll: %0.2f | Pitch: %0.2f | Yaw: %0.2f",
                 base->roll, base->pitch, base->yaw);
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
        xTaskCreatePinnedToCore(mpu_task, "mpu_task", 4096, (void *)base, 10, NULL, 1);
}