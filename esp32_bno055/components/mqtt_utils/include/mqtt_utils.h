#ifndef _MQTT_UTILS_H
#define _MQTT_UTILS_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "cJSON.h"
#include "mqtt_client.h"

#include "constants.h"

// MQTT Data struct
typedef struct
{
    float timestamp;
    float shoulder[3];
    float elbow[3];
    float wrist[3];
    bool gripper_state;
} mqtt_data_t;

// MQTT Data queue handle
QueueHandle_t mqtt_data_queue;

// MQTT Client Handle
esp_mqtt_client_handle_t mqtt_client;

void mqtt_get_angles_task(void *arg);

esp_err_t mqtt_send_joint_angles(mqtt_data_t *joint_angles);

esp_err_t mqtt_callback(esp_mqtt_event_handle_t event);

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void init_mqtt(void);

#endif
