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
static SemaphoreHandle_t xSemaphore = NULL;

// interrupt service routine, called when data is received
void IRAM_ATTR mpu_dmp_isr_handler(void *arg)
{
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

esp_err_t enable_gpio_interrupt(void)
{
    gpio_config_t io_conf;
    // bit mask for the pin, each bit map to a GPIO
    io_conf.pin_bit_mask = 1ULL << I2C_MASTER_INT_IO;
    // set gpio mode to input
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull up resistors
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    // disable pull down resistors
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // set active low interrupts
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to initialize GPIO!");

    xSemaphore = xSemaphoreCreateBinary();

    // install ISR service with default configuration
    err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // attach the interrupt service routine
    gpio_isr_handler_add(I2C_MASTER_INT_IO, mpu_dmp_isr_handler, NULL);

    return err;
}

// task that will react to data
void mpu_int_task(void *arg)
{
    // infinite loop
    while (1)
    {
        // wait for the notification from the ISR
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // ets_printf("Interrupt Triggered: State: %d\n", uxSemaphoreGetCount(xSemaphore));
            update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
            print_mpu_raw_agm();

            xSemaphoreGive(xSemaphore);
        }
    }
}

void app_main()
{
    ESP_LOGI(TAG, "Hello From ESP32!");

    if (i2c_master_init() == ESP_OK && begin() == INV_SUCCESS)
    {
        setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
        setSampleRate(100);       
        setCompassSampleRate(100); 

        enableInterrupt(true);
        setIntLevel(INT_ACTIVE_LOW);
        setIntLatched(INT_NOT_LATCHED);

        enable_gpio_interrupt();
        xTaskCreate(mpu_int_task, "mpu_int_task", 4096, NULL, 5, NULL);
    }
    else
        ESP_LOGE(TAG, "Initialization Failure!");
}