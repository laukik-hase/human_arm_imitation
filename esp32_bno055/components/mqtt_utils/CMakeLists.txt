set(srcs
	"mqtt_utils.c")    
	
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES constants esp_event freertos json mbedtls
								mqtt spiffs vfs)                     
