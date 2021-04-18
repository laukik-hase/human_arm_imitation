idf_component_register(SRCS "nvs_utils.c"
                       INCLUDE_DIRS "include"
                       REQUIRES constants driver esp_event freertos
				        json nvs_flash soc)
