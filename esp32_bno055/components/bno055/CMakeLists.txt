set(srcs
	"src/i2c_utils.c"
    "src/bno055_support.c"
    "src/bno055.c"
    "src/bno055_utils.c")
    
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES constants driver esp32 esp_common esp_event 
                                esp_rom nvs_utils freertos heap log soc)