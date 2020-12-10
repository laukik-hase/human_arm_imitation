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

// task that will react to data
void mpu_task(void *arg)
{
    // infinite loop
    while (1)
    {
        if (fifoAvailable() && dmpUpdateFifo() == INV_SUCCESS)
        {
            print_mpu_raw_quat();
            // computeEulerAngles(true);
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");

    if (i2c_master_init() == ESP_OK && begin() == INV_SUCCESS)
    {
        dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 200);
        xTaskCreate(mpu_task, "mpu_int_task", 4096, NULL, 5, NULL);
    }
    else
        ESP_LOGE(TAG, "Initialization Failure!");
}
