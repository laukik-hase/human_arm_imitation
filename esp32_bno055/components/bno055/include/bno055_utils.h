#ifndef _BNO055_UTILS_H
#define _BNO055_UTILS_H

#include "i2c_utils.h"
#include "nvs_utils.h"

extern const char *key[36];
extern const int16_t value[36];

typedef enum
{
    ROLL = 0x00,
    PITCH,
    HEADING
} euler_angle_t;

struct joint_angle_t
{
    float shoulder[3];
    float elbow[3];
    float wrist[3];
};

BNO055_RETURN_FUNCTION_TYPE bno055_calib_accel(mpu_pos_t link);

BNO055_RETURN_FUNCTION_TYPE bno055_calib_gyro(mpu_pos_t link);

BNO055_RETURN_FUNCTION_TYPE bno055_calib_mag(mpu_pos_t link);

BNO055_RETURN_FUNCTION_TYPE bno055_load_calib_data(mpu_pos_t link);

BNO055_RETURN_FUNCTION_TYPE bno055_calib_routine(uint8_t sensors, mpu_pos_t link);

void bno055_self_test_routine(void);

void bno055_get_calib_status(void);

BNO055_RETURN_FUNCTION_TYPE bno055_init_routine(struct bno055_t *imu, uint8_t sensors, mpu_pos_t link);

void bno055_get_rph(struct bno055_euler_float_t *bno_rph);

void bno055_get_rph_from_quaternion(struct bno055_euler_float_t *bno_rph);

float get_corrected_joint_yaw(float joint_angle);

#endif