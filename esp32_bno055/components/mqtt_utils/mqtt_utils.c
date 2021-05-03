#include "mqtt_utils.h"

static const char *TAG_MQTT_UTILS = "MQTT_UTILS";

// MQTT Event Group
static EventGroupHandle_t s_mqtt_event_group;

void mqtt_get_angles_task(void *arg)
{
    mqtt_data_t recv_data = {0};

    while (true)
    {
        if (xQueueReceive(mqtt_data_queue, &recv_data, (portTickType)portMAX_DELAY))
        {
            // ESP_LOGI(TAG_MQTT_UTILS, "Received data in MQTT Queue");
            mqtt_send_joint_angles(&recv_data);
        }
    }
}

esp_err_t mqtt_send_joint_angles(mqtt_data_t *joint_angle)
{
    cJSON *root = cJSON_CreateObject();
    esp_err_t err = ESP_FAIL;

    if (root != NULL)
    {
        cJSON_AddItemToObject(root, "timestamp", cJSON_CreateNumber(joint_angle->timestamp));
        cJSON_AddItemToObject(root, "is_gripper_open", cJSON_CreateBool(joint_angle->gripper_state));
        
        cJSON *shoulder = cJSON_CreateObject();
        cJSON_AddItemToObject(shoulder, "roll", cJSON_CreateNumber(joint_angle->shoulder[ROLL]));
        cJSON_AddItemToObject(shoulder, "pitch", cJSON_CreateNumber(joint_angle->shoulder[PITCH]));
        cJSON_AddItemToObject(shoulder, "yaw", cJSON_CreateNumber(joint_angle->shoulder[HEADING]));

        cJSON *elbow = cJSON_CreateObject();
        cJSON_AddItemToObject(elbow, "pitch", cJSON_CreateNumber(joint_angle->elbow[PITCH]));

        cJSON_AddItemToObject(root, "shoulder", shoulder);
        cJSON_AddItemToObject(root, "elbow", elbow);

        char *mqtt_msg = cJSON_PrintUnformatted(root);
        ESP_LOGI(TAG_MQTT_UTILS, "Message : %s", mqtt_msg);

        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, mqtt_msg, 0, MQTT_QOS, 1);

        free(mqtt_msg);
        err = ESP_OK;
    }

    cJSON_Delete(root);
    return err;
}

esp_err_t mqtt_callback(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    mqtt_data_t send_data = {0};
    static int mqtt_retry = 0;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT_UTILS, "%s", "MQTT_EVENT_CONNECTED");
        xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);

        // esp_mqtt_client_subscribe(client, MQTT_TOPIC, MQTT_QOS);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG_MQTT_UTILS, "MQTT_EVENT_SUBSCRIBED, msg_id = %d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGD(TAG_MQTT_UTILS, "MQTT_EVENT_UNSUBSCRIBED, msg_id = %d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG_MQTT_UTILS, "MQTT_EVENT_PUBLISHED, msg_id = %d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG_MQTT_UTILS, "%s", "MQTT_EVENT_DATA");
        break;

    case MQTT_EVENT_DISCONNECTED:
    case MQTT_EVENT_ERROR:
        if (mqtt_retry < MQTT_RECONN_ATTEMPTS)
        {
            ESP_LOGI(TAG_MQTT_UTILS, "Retrying MQTT Connection: Attempt %d", mqtt_retry);
            esp_mqtt_client_reconnect(mqtt_client);
            mqtt_retry++;
            xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);

            vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGW(TAG_MQTT_UTILS, "%s", "Restarting...");
            esp_restart();
        }

        break;

    default:
        ESP_LOGD(TAG_MQTT_UTILS, "Other event id: %d", event->event_id);
        break;
    }

    return ESP_OK;
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT_UTILS, "Event: %s, ID: %d", base, event_id);
    mqtt_callback(event_data);
}

void init_mqtt(void)
{
    esp_mqtt_client_config_t mqtt_cfg =
    {
        .uri = MQTT_BROKER,
        .task_stack = MQTT_STACK,
        .buffer_size = MQTT_BUFFER_SIZE,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);

    s_mqtt_event_group = xEventGroupCreate();
    xEventGroupWaitBits(s_mqtt_event_group,
                        MQTT_CONNECTED_BIT,
                        pdFALSE,
                        pdFALSE,
                        MQTT_RECONN_TIME_LIMIT * portTICK_PERIOD_MS);
}
