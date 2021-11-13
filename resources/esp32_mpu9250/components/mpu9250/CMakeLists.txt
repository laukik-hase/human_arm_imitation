set(srcs
	"src/i2c_utils.c"
	"src/inv_mpu.c"
	"src/inv_mpu_dmp_motion_driver.c"
	"src/mpu9250.c")
    
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES driver efuse esp32 esp_adc_cal esp_common esp_event 
                                esp_rom freertos heap log soc spiffs)