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
    **Euler angle axes**
    Pitch: X-Axis
    Roll: Y-Axis
    Heading: Z-Axis
*/

#define IF_CALIBRATION (false)

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(nvs_init());

    struct bno055_t link;
    uint8_t sensors = NONE;

#if IF_CALIBRATION == true
    sensors = ACCEL | GYRO | MAG;
#endif

    // Connect to shoulder
    i2c_mux_select(MPU_SHOULDER);
    bno055_init_routine(&link, sensors, MPU_SHOULDER);

    struct bno055_euler_float_t link_angle;

    uint64_t timestamp = 0;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    uint64_t start = (uint64_t)(esp_timer_get_time() / 1000);

    while (1)
    {
        bno055_get_rph(&link_angle);

        timestamp = (uint64_t)(esp_timer_get_time() / 1000) - start;
        printf("%lld, %0.2f, %0.2f, %0.2f\n", timestamp, link_angle.r, link_angle.p, link_angle.h);
    }
}
