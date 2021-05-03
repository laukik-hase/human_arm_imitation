#include "gpio_utils.h"

// static const char *TAG_GPIO_UTILS = "GPIO_UTILS";

// GPIO Masks

static const uint64_t gripper = (1ULL << GRIPPER_SWITCH);

void IRAM_ATTR gpio_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(gpio_TaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

void gripper_task(void *arg)
{
    uint32_t ulNotifiedValue = 0;
    BaseType_t xResult = pdFAIL;

    while (1)
    {
        xResult = xTaskNotifyWait(0x00, 0x00, &ulNotifiedValue, portMAX_DELAY);
        if (xResult == pdPASS)
            is_gripper_open = !is_gripper_open;
    }
}

esp_err_t init_gpio_pins(void)
{
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = gripper;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE; // Default state is pulled-up
    IS_ESP_OK(gpio_config(&io_conf));

    return ESP_OK;
}

esp_err_t init_gpio_isr(void)
{
    IS_ESP_OK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    IS_ESP_OK(gpio_isr_handler_add(GRIPPER_SWITCH, gpio_isr, NULL));

    return ESP_OK;
}
