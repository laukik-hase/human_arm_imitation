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

    mpu_data_t *shoulder = malloc(sizeof(mpu_data_t));
    shoulder->pos = MPU_SHOULDER;

    static uint64_t counter = 0;

    // uint8_t calib_flag = ACCEL_GYRO_CAL | COMPASS_CAL;
    uint8_t calib_flag = ACCEL_GYRO_CAL;

    if (initMPU(base, calib_flag) == INV_SUCCESS && initMPU(shoulder, calib_flag) == INV_SUCCESS)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        while (1)
        {
            if (!i2c_mux_select(MPU_BASE))
            {
                if (dataReady())
                {
                    MadgwickFilter(base);
                    computeEulerAngles(base, true);

                    if (!i2c_mux_select(MPU_SHOULDER))
                    {
                        if (dataReady())
                        {
                            MadgwickFilter(shoulder);
                            computeEulerAngles(shoulder, true);

                            counter++;

                            float roll_diff = (base->roll - shoulder->roll);
                            float pitch_diff = (base->pitch - shoulder->pitch);
                            float yaw_diff = (base->yaw - shoulder->yaw);

                            // CSV style printing
                            printf("%lld, %f, %f, %f\n", counter, roll_diff, pitch_diff, yaw_diff);
                        }
                    }
                }
            }
        }
    }
    else
        ESP_LOGE(TAG, "Init Failure!");
}