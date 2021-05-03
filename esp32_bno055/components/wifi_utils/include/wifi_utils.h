#ifndef _WIFI_UTILS_H
#define _WIFI_UTILS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_err.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "constants.h"
#include "nvs_flash.h"

//WiFi Function
void wifi_init_sta(void);

#endif


