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

#define IF_CALIBRATION (false)

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(nvs_init());

    uint8_t sensors = NONE;

#if IF_CALIBRATION == true
    sensors = ACCEL | GYRO | MAG;
#endif

    struct bno055_t link[NO_OF_LINKS];

    i2c_mux_select(MPU_SHOULDER);
    bno055_init_routine(&link[MPU_SHOULDER], sensors, MPU_SHOULDER);

    i2c_mux_select(MPU_BICEP);
    bno055_init_routine(&link[MPU_BICEP], sensors, MPU_BICEP);

    // i2c_mux_select(MPU_FOREARM);
    // bno055_init_routine(&link[MPU_FOREARM], sensors, MPU_FOREARM);

    struct bno055_euler_float_t link_angle[NO_OF_LINKS];
    struct joint_angle_t joint_angle;

    uint64_t timestamp = 0;
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    uint64_t start = (uint64_t)(esp_timer_get_time() / 1000);

    while (1)
    {
        i2c_mux_select(MPU_SHOULDER);
        bno055_get_rph(&link_angle[MPU_SHOULDER]);

        i2c_mux_select(MPU_BICEP);
        bno055_get_rph(&link_angle[MPU_BICEP]);

        // i2c_mux_select(MPU_FOREARM);
        // bno055_get_rph(&link_angle[MPU_FOREARM]);

        // Inversion of Pitch and Roll due to mounting orientation on arm
        joint_angle.shoulder[PITCH] = (link_angle[MPU_SHOULDER].r - link_angle[MPU_BICEP].r);
        joint_angle.shoulder[ROLL] = (link_angle[MPU_SHOULDER].p - link_angle[MPU_BICEP].p);
        joint_angle.shoulder[HEADING] = (link_angle[MPU_SHOULDER].h - link_angle[MPU_BICEP].h);

        timestamp = (uint64_t)(esp_timer_get_time() / 1000) - start;
        printf("%lld, %0.2f, %0.2f, %0.2f\n", timestamp, joint_angle.shoulder[ROLL], joint_angle.shoulder[PITCH], joint_angle.shoulder[HEADING]);

        // joint_angle.elbow[PITCH] = (link_angle[MPU_BICEP].r - link_angle[MPU_FOREARM].r);
        // joint_angle.elbow[ROLL] = (link_angle[MPU_BICEP].p - link_angle[MPU_FOREARM].p);
        // joint_angle.elbow[HEADING] = (link_angle[MPU_BICEP].h - link_angle[MPU_FOREARM].h);

        // printf("%0.2f, %0.2f, %0.2f\n", joint_angle.elbow[ROLL], joint_angle.elbow[PITCH], joint_angle.elbow[HEADING]);
    }
}
