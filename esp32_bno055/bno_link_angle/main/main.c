#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno055_utils.h"

static const char *TAG = "bno_calib_st";

/*
    **Euler angle axes** -- Refer bno055_init_routine(...)
    Pitch: Y-Axis
    Roll: X-Axis
    Heading: Z-Axis
*/


void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());

    // Initializing the NVS and loading the already defined calibration matrix
    // Refer: nvs_utils.c
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(nvs_load_calib_data());

    // sensors = ACCEL | GYRO | MAG -> for full calibration, NONE -> for nothing
    uint8_t sensors = LOAD_FROM_NVS;

    struct bno055_t link;
    i2c_mux_select(MPU_PALM); // Connect to PALM slot
    bno055_init_routine(&link, sensors, MPU_PALM);
    
    struct bno055_euler_float_t link_angle;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1)
    {
        bno055_get_rph(&link_angle);
        ESP_LOGD(TAG, "Roll: %0.4f | Pitch: %0.4f | Yaw: %0.4f", link_angle.r, link_angle.p, link_angle.h);
    }
}
