#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bno055_utils.h"

static const char *TAG = "bno_test";

/*
    **Euler angle axes**
    Pitch: X-Axis
    Roll: Y-Axis
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

    struct bno055_t link[NO_OF_LINKS];

    i2c_mux_select(MPU_SHOULDER);
    bno055_init_routine(&link[MPU_SHOULDER], sensors, MPU_SHOULDER);

    i2c_mux_select(MPU_BICEP);
    bno055_init_routine(&link[MPU_BICEP], sensors, MPU_BICEP);

    i2c_mux_select(MPU_FOREARM);
    bno055_init_routine(&link[MPU_FOREARM], sensors, MPU_FOREARM);

    struct bno055_euler_float_t link_angle[NO_OF_LINKS];
    struct joint_angle_t joint_angle;

    float timestamp = 0;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    float start = (float)(esp_timer_get_time() / 1000000.0f);
    printf("Time, Rs, Ps, Ys, Pe\n");

    while (1)
    {
        i2c_mux_select(MPU_SHOULDER);
        bno055_get_rph(&link_angle[MPU_SHOULDER]);

        i2c_mux_select(MPU_BICEP);
        bno055_get_rph(&link_angle[MPU_BICEP]);

        i2c_mux_select(MPU_FOREARM);
        bno055_get_rph(&link_angle[MPU_FOREARM]);

        joint_angle.shoulder[ROLL] = (link_angle[MPU_SHOULDER].r - link_angle[MPU_BICEP].r);
        joint_angle.shoulder[PITCH] = (link_angle[MPU_SHOULDER].p - link_angle[MPU_BICEP].p);
        joint_angle.shoulder[HEADING] = get_corrected_joint_yaw(link_angle[MPU_SHOULDER].h - link_angle[MPU_BICEP].h);
        joint_angle.elbow[PITCH] = (link_angle[MPU_BICEP].p - link_angle[MPU_FOREARM].p);

        timestamp = (float)(esp_timer_get_time() / 1000000.0f) - start;

        // CSV-style printing
        printf("%0.3f, %0.2f, %0.2f, %0.2f, %0.2f\n",
               timestamp, joint_angle.shoulder[ROLL], joint_angle.shoulder[PITCH], joint_angle.shoulder[HEADING], joint_angle.elbow[PITCH]);
    }
}
