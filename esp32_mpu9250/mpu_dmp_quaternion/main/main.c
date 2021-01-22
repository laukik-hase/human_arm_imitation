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

    while (1)
    {
        if (fifoAvailable() && dmpUpdateFifo(base) == INV_SUCCESS)
        {
            ESP_LOGD(TAG, "Qw: %f | Qx: %f | Qy: %f | Qz: %f",
                     base->qw, base->qx, base->qy, base->qz);
                     
            // computeEulerAngles(base, true);
            // ESP_LOGD(TAG, "Roll: %0.2f | Pitch: %0.2f | Yaw: %0.2f",
            //          base->roll, base->pitch, base->yaw);
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());

    mpu_data_t *base = malloc(sizeof(mpu_data_t));
    base->pos = MPU_BASE;

    // TODO: For better quaternion results directly from the DMP, 
    // push the Acce and Gyro biases in the DMP registers.
    uint8_t calib_flag = 0x00;

    if (initMPUwithDMP(base, calib_flag) != INV_SUCCESS)
        ESP_LOGE(TAG, "Initialization Failure!");
    else
        xTaskCreate(mpu_task, "mpu_task", 4096, (void *)base, 5, NULL);
}
