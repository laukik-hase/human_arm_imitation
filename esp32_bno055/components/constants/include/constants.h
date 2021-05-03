#ifndef _CONSTANTS_H
#define _CONSTANTS_H

//Macro for error checking
#define IS_ESP_OK(x)        \
  do                        \
  {                         \
    esp_err_t __;           \
    if ((__ = x) != ESP_OK) \
      return __;            \
  } while (0);

//Macro for error checking
#define IS_BNO_OK(x)                \
  do                                \
  {                                 \
    BNO055_RETURN_FUNCTION_TYPE __; \
    if ((__ = x) != BNO055_SUCCESS) \
      return __;                    \
  } while (0);

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_INT_IO 23        /*!< gpio number for I2C master interrupt */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */

#define ESP_INTR_FLAG_DEFAULT 0
#define TCA9548_ADDR 0x70

/*
  BNO055 uses clock stretching to slow down operation.
  Clock source unit: APB 80 MHz
  Timeout: 0xFFFFF = 1048575 => 12.5 ms
  BNO055 Clock Stretch: 1 ms => 83386 = 0x145BA
*/
#define BNO055_I2C_TIMEOUT (83386)

#define NO_OF_LINKS 4

#define ROLL (0x00)
#define PITCH (0x01)
#define HEADING (0x02)

// Sensor mask for calibration
#define NONE (((uint8_t)0x00)
#define ACCEL ((uint8_t)1 << 0)
#define GYRO ((uint8_t)1 << 1)
#define MAG ((uint8_t)1 << 2)
#define LOAD_FROM_NVS ((uint8_t)1 << 3)

#define RAD_TO_DEG (57.2958f)
#define DEG_TO_RAD (0.0174533f)
#define PI (3.14159265f)

#define EULER_SCALE (16.0f)
#define QUATERNION_SCALE (16384.0f)

// You can either use the menuconfig utility or pass values directly here
#define WIFI_SSID "myssid"  // CONFIG_WIFI_SSID
#define WIFI_PWD  "mypassword"    // CONFIG_WIFI_PWD
#define WIFI_AP_RECONN_ATTEMPTS 5  // CONFIG_WIFI_AP_RECONN_ATTEMPTS

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define MQTT_CONNECTED_BIT BIT2
#define MQTT_RECONN_TIME_LIMIT 30
#define MQTT_RECONN_ATTEMPTS 3

#define MQTT_BROKER "mqtt://test.mosquitto.org"
#define MQTT_TOPIC "fyp/sensors"
#define MQTT_QOS 1
#define MQTT_STACK 8192
#define MQTT_BUFFER_SIZE 4096

#endif
