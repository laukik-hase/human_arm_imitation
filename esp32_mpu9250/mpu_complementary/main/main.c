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

void mpu_task(void *arg)
{
    while (1)
    {
        if (dataReady())
        {
            update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
            complementary_filter();
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");

    if (i2c_master_init() == ESP_OK && begin() == INV_SUCCESS)
        xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 5, NULL);
    else
        ESP_LOGE(TAG, "Initialization Failure!");
}