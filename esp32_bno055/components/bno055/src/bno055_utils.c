#include "bno055_utils.h"

static const char *TAG_BNO = "bno055_utils";

////////////////////////////////////////////////////////////////////////////

BNO055_RETURN_FUNCTION_TYPE bno055_calib_accel(mpu_pos_t link)
{
    // data array to hold accelerometer offsets
    struct bno055_accel_offset_t accel_bias;
    ESP_LOGI(TAG_BNO, "Accelerometer Calibration");

    // checking whether the accel is already calibrated
    struct bno055_accel_t accel_raw;
    u8 accel_var = 0;
    IS_BNO_OK(bno055_get_accel_calib_stat(&accel_var));

    if (accel_var == 3)
    {
        IS_BNO_OK(bno055_read_accel_offset(&accel_bias));
        ESP_LOGD(TAG_BNO, "Accel Already Calibrated!");
        ESP_LOGD(TAG_BNO, "Acce Bias: X: %d | Y: %d | Z: %d", accel_bias.x, accel_bias.y, accel_bias.z);

        // Loading the Offsets in NVS
        int k = (link - 1) * 9;
        nvs_write_short(key[k], &accel_bias.x);
        nvs_write_short(key[k + 1], &accel_bias.y);
        nvs_write_short(key[k + 2], &accel_bias.z);

        return BNO055_SUCCESS;
    }

    // Calibration procedure - Move BNO in 45 deg tilts around all the axes till calibration is done
    // NOTE: This is the most tedious sensor to calculate
    do
    {
        IS_BNO_OK(bno055_read_accel_xyz(&accel_raw));
        IS_BNO_OK(bno055_get_accel_calib_stat(&accel_var));
        ESP_LOGD(TAG_BNO, "Accel: X: %d | Y: %d | Z: %d | C: %d", accel_raw.x, accel_raw.y, accel_raw.z, accel_var);
        BNO055_delay_msek(10);
    } while (accel_var != 3);

    BNO055_delay_msek(1000);
    ESP_LOGD(TAG_BNO, "Accel Calibrated!");
    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_gyro(mpu_pos_t link)
{
    // data array to hold gyro offsets
    struct bno055_gyro_offset_t gyro_bias;
    ESP_LOGI(TAG_BNO, "Gyroscope Calibration");

    // checking whether the gyro is already calibrated
    struct bno055_gyro_t gyro_raw;
    u8 gyro_var = 0;
    IS_BNO_OK(bno055_get_gyro_calib_stat(&gyro_var));

    if (gyro_var == 3)
    {
        IS_BNO_OK(bno055_read_gyro_offset(&gyro_bias));
        ESP_LOGD(TAG_BNO, "Gyro Already Calibrated!");
        ESP_LOGD(TAG_BNO, "Gyro Bias: X: %d | Y: %d | Z: %d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
        
        // Loading the Offsets in NVS
        int k = (link - 1) * 9;
        nvs_write_short(key[k + 3], &gyro_bias.x);
        nvs_write_short(key[k + 4], &gyro_bias.y);
        nvs_write_short(key[k + 5], &gyro_bias.z);

        return BNO055_SUCCESS;
    }

    // Calibration procedure - Keep BNO stable flat on the ground
    do
    {
        IS_BNO_OK(bno055_read_gyro_xyz(&gyro_raw));
        IS_BNO_OK(bno055_get_gyro_calib_stat(&gyro_var));

        ESP_LOGD(TAG_BNO, "Gyro: X: %d | Y: %d | Z: %d | C: %d", gyro_raw.x, gyro_raw.y, gyro_raw.z, gyro_var);
        BNO055_delay_msek(10);
    } while (gyro_var != 3);

    BNO055_delay_msek(1000);

    IS_BNO_OK(bno055_read_gyro_offset(&gyro_bias));
    ESP_LOGD(TAG_BNO, "Gyro Calibrated!");
    ESP_LOGD(TAG_BNO, "Gyro Bias: X: %d | Y: %d | Z: %d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_mag(mpu_pos_t link)
{
    // data array to hold compass offsets
    struct bno055_mag_offset_t mag_bias;
    ESP_LOGI(TAG_BNO, "Compass Calibration");

    u8 mag_var = 0;
    IS_BNO_OK(bno055_get_mag_calib_stat(&mag_var));

    if (mag_var == 3)
    {
        IS_BNO_OK(bno055_read_mag_offset(&mag_bias));
        ESP_LOGD(TAG_BNO, "Accel Already Calibrated!");
        ESP_LOGD(TAG_BNO, "Mag Bias: X: %d | Y: %d | Z: %d", mag_bias.x, mag_bias.y, mag_bias.z);
        
        // Loading the Offsets in NVS
        int k = (link - 1) * 9;
        nvs_write_short(key[k + 6], &mag_bias.x);
        nvs_write_short(key[k + 7], &mag_bias.y);
        nvs_write_short(key[k + 8], &mag_bias.z);
        
        return BNO055_SUCCESS;
    }

    ESP_LOGI(TAG_BNO, "Compass Calibration: Move in figure 8 till done");
    do
    {
        IS_BNO_OK(bno055_get_mag_calib_stat(&mag_var));
        BNO055_delay_msek(10);
    } while (mag_var != 3);

    BNO055_delay_msek(1000);
    IS_BNO_OK(bno055_read_mag_offset(&mag_bias));
    ESP_LOGD(TAG_BNO, "Mag Bias: X: %d | Y: %d | Z: %d", mag_bias.x, mag_bias.y, mag_bias.z);
    ESP_LOGI(TAG_BNO, "Mag Calibrated!");
    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_load_calib_data(mpu_pos_t link)
{
    struct bno055_accel_offset_t accel_bias;
    struct bno055_gyro_offset_t gyro_bias;
    struct bno055_mag_offset_t mag_bias;

    if (i2c_mux_select(link) != ESP_OK)
        return BNO055_ERROR;

    // NOTE:  Starting point of bias data in key array
    int k = (link - 1) * 9;

    // NOTE: Assuming this will not fail, error checks have not been included
    nvs_read_short(key[k], &accel_bias.x);
    nvs_read_short(key[k + 1], &accel_bias.y);
    nvs_read_short(key[k + 2], &accel_bias.z);

    nvs_read_short(key[k + 3], &gyro_bias.x);
    nvs_read_short(key[k + 4], &gyro_bias.y);
    nvs_read_short(key[k + 5], &gyro_bias.z);

    nvs_read_short(key[k + 6], &mag_bias.x);
    nvs_read_short(key[k + 7], &mag_bias.y);
    nvs_read_short(key[k + 8], &mag_bias.z);

    IS_BNO_OK(bno055_write_accel_offset(&accel_bias));
    BNO055_delay_msek(10);

    IS_BNO_OK(bno055_write_gyro_offset(&gyro_bias));
    BNO055_delay_msek(10);

    IS_BNO_OK(bno055_write_mag_offset(&mag_bias));
    BNO055_delay_msek(10);

    ESP_LOGD(TAG_BNO, "Acce Bias: X: %d | Y: %d | Z: %d", accel_bias.x, accel_bias.y, accel_bias.z);
    ESP_LOGD(TAG_BNO, "Gyro Bias: X: %d | Y: %d | Z: %d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    ESP_LOGD(TAG_BNO, "Mag Bias: X: %d | Y: %d | Z: %d", mag_bias.x, mag_bias.y, mag_bias.z);

    ESP_LOGI(TAG_BNO, "Calibration profile loaded successfully!");

    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_routine(uint8_t sensors, mpu_pos_t link)
{
    if (sensors & LOAD_FROM_NVS)
        IS_BNO_OK(bno055_load_calib_data(link));

    if (sensors & ACCEL)
        IS_BNO_OK(bno055_calib_accel(link));

    if (sensors & GYRO)
        IS_BNO_OK(bno055_calib_gyro(link));

    if (sensors & MAG)
        IS_BNO_OK(bno055_calib_mag(link));

    return BNO055_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

void bno055_self_test_routine(void)
{
    u8 accel_var = 0, gyro_var = 0, mag_var = 0, sys_var = 0;

    bno055_get_selftest_accel(&accel_var);
    bno055_get_selftest_gyro(&gyro_var);
    bno055_get_selftest_mag(&mag_var);
    bno055_get_selftest_mcu(&sys_var);

    ESP_LOGD(TAG_BNO, "ST Status: A: %d | G: %d | M: %d | S: %d",
             accel_var, gyro_var, mag_var, sys_var);
}

void bno055_get_calib_status(void)
{
    u8 accel_var = 0, gyro_var = 0, mag_var = 0, sys_var = 0;

    bno055_get_accel_calib_stat(&accel_var);
    bno055_get_mag_calib_stat(&mag_var);
    bno055_get_gyro_calib_stat(&gyro_var);
    bno055_get_sys_calib_stat(&sys_var);

    ESP_LOGD(TAG_BNO, "Calibration Status: A: %d | G: %d | M: %d | S: %d",
             accel_var, gyro_var, mag_var, sys_var);
}

BNO055_RETURN_FUNCTION_TYPE bno055_init_routine(struct bno055_t *imu, uint8_t sensors, mpu_pos_t link)
{
    imu->bus_write = BNO055_I2C_bus_write;
    imu->bus_read = BNO055_I2C_bus_read;
    imu->dev_addr = BNO055_I2C_ADDR1;
    imu->delay_msec = BNO055_delay_msek;

    IS_BNO_OK(bno055_init(imu));

    // Inverting Pitch and Roll according to our application
    IS_BNO_OK(bno055_set_axis_remap_value(BNO055_REMAP_X_Y));
    IS_BNO_OK(bno055_set_remap_x_sign(BNO055_REMAP_AXIS_NEGATIVE));
    IS_BNO_OK(bno055_set_remap_y_sign(BNO055_REMAP_AXIS_POSITIVE));

    /* 
    In NDOF fusion mode - 
    accel full scale is at +/- 4g, ODR is 62.5 Hz
    gyro full scale is at +/- 2000dps, ODR is 32 Hz
    mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
*/
    IS_BNO_OK(bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF));
    IS_BNO_OK(bno055_set_euler_unit(BNO055_EULER_UNIT_DEG));
    IS_BNO_OK(bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG));

    IS_BNO_OK(bno055_calib_routine(sensors, link));
    bno055_self_test_routine();
    bno055_get_calib_status();

    ESP_LOGI(TAG_BNO, "BNO Initialised!");
    return BNO055_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

void bno055_get_rph(struct bno055_euler_float_t *bno_rph)
{
    s16 heading, pitch, roll;

    if (bno055_read_euler_h(&heading) == BNO055_SUCCESS && bno055_read_euler_p(&pitch) == BNO055_SUCCESS && bno055_read_euler_r(&roll) == BNO055_SUCCESS)
    {
        bno_rph->r = roll / EULER_SCALE;
        bno_rph->p = pitch / EULER_SCALE;
        bno_rph->h = heading / EULER_SCALE;
    }
    else
        ESP_LOGE(TAG_BNO, "Measurement Error!");

    BNO055_delay_msek(10);
}

void bno055_get_rph_from_quaternion(struct bno055_euler_float_t *bno_rph)
{
    struct bno055_quaternion_t link_quat;

    if (bno055_read_quaternion_wxyz(&link_quat) == BNO055_SUCCESS)
    {
        float qw = link_quat.w / QUATERNION_SCALE;
        float qx = link_quat.x / QUATERNION_SCALE;
        float qy = link_quat.y / QUATERNION_SCALE;
        float qz = link_quat.z / QUATERNION_SCALE;

        double qw2 = qw * qw;
        double qx2 = qx * qx;
        double qy2 = qy * qy;
        double qz2 = qz * qz;

        bno_rph->h = -atan2f(2.0f * (qx * qy + qw * qz), qw2 + qx2 - qy2 - qz2) * RAD_TO_DEG;
        bno_rph->r = asinf(2.0f * (qx * qz - qw * qy)) * RAD_TO_DEG;
        bno_rph->p = -atan2f(2.0f * (qw * qx + qy * qz), qw2 - qx2 - qy2 + qz2) * RAD_TO_DEG;

        if (bno_rph->h < 0)
            bno_rph->h += 360;
    }
    else
        ESP_LOGE(TAG_BNO, "Measurement Error!");

    // NOTE: For NDOF Fusion rate, the algo calling rate is 100Hz.
    BNO055_delay_msek(10);
}

////////////////////////////////////////////////////////////////////////////

float get_corrected_joint_yaw(float joint_angle)
{
    if (joint_angle >= 180)
        joint_angle -= 360.0f;
    else if (joint_angle <= -180)
        joint_angle += 360.0f;

    return joint_angle;
}