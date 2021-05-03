set(srcs
	"wifi_utils.c")
	
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES constants esp32 esp_event
					   esp_http_server esp_netif esp_wifi freertos 
					   log lwip nvs_flash partition_table)                

