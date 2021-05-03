#ifndef _GPIO_UTILS_H
#define _GPIO_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_system.h"
#include "esp_event.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "constants.h"

bool is_gripper_open;

TaskHandle_t gpio_TaskHandle;

void IRAM_ATTR gpio_isr(void *arg);

void gripper_task(void *arg);

esp_err_t init_gpio_pins(void);

esp_err_t init_gpio_isr(void);

#endif
