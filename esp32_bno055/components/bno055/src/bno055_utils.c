#include "bno055_utils.h"

static const char *TAG_BNO = "bno055_utils";

BNO055_RETURN_FUNCTION_TYPE bno055_calib_accel(void)
{
    // data array to hold accelerometer offsets
    struct bno055_accel_offset_t accel_bias;

    ESP_LOGI(TAG_BNO, "Accelerometer Calibration");
    BNO055_delay_msek(2000);

    // Set the operation mode
    // In NDOF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz
    IS_BNO_OK(bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG));
    IS_BNO_OK(bno055_set_accel_power_mode(BNO055_ACCEL_NORMAL));
    IS_BNO_OK(bno055_set_accel_range(BNO055_ACCEL_RANGE_4G));
    IS_BNO_OK(bno055_set_accel_bw(BNO055_ACCEL_BW_62_5HZ));

    int cnt = 256;
    struct bno055_accel_t accel_raw;

    for (int i = 0; i < cnt; i++)
    {
        IS_BNO_OK(bno055_read_accel_xyz(&accel_raw));

        accel_bias.x += accel_raw.x;
        accel_bias.y += accel_raw.y;
        accel_bias.z += accel_raw.z;

        // At 62.5 Hz ODR, new accel data is available every 16 ms
        BNO055_delay_msek(20);
    }

    // Get average accel bias in mg
    accel_bias.x /= cnt;
    accel_bias.y /= cnt;
    accel_bias.z /= cnt;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accel_bias.z > 0L)
        accel_bias.z -= 1000.;
    else
        accel_bias.z += 1000.;

    IS_BNO_OK(bno055_write_accel_offset(&accel_bias));

    ESP_LOGD(TAG_BNO, "Acce Bias: X: %d | Y: %d | Z: %d", accel_bias.x, accel_bias.y, accel_bias.z);
    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_gyro(void)
{
    // data array to hold gyro offsets
    struct bno055_gyro_offset_t gyro_bias;

    ESP_LOGI(TAG_BNO, "Gyroscope Calibration");
    BNO055_delay_msek(2000);

    // Set the operation mode
    // In NDOF fusion mode, gyro full scale is at +/- 2000dps, ODR is 32 Hz
    IS_BNO_OK(bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG));
    IS_BNO_OK(bno055_set_gyro_power_mode(BNO055_GYRO_POWER_MODE_NORMAL));
    IS_BNO_OK(bno055_set_gyro_range(BNO055_GYRO_RANGE_2000DPS));
    IS_BNO_OK(bno055_set_gyro_bw(BNO055_GYRO_BW_32HZ));

    int cnt = 256;
    struct bno055_gyro_t gyro_raw;

    for (int i = 0; i < cnt; i++)
    {
        IS_BNO_OK(bno055_read_gyro_xyz(&gyro_raw));

        gyro_bias.x += gyro_raw.x;
        gyro_bias.y += gyro_raw.y;
        gyro_bias.z += gyro_raw.z;

        // At 32 Hz ODR, new accel data is available every 31 ms
        BNO055_delay_msek(35);
    }

    // get average gyro bias in counts
    gyro_bias.x /= cnt;
    gyro_bias.y /= cnt;
    gyro_bias.z /= cnt;

    IS_BNO_OK(bno055_write_gyro_offset(&gyro_bias));
    ESP_LOGD(TAG_BNO, "Gyro Bias: X: %d | Y: %d | Z: %d", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_mag(void)
{
    // data array to hold compass offsets
    struct bno055_mag_offset_t mag_bias;

    float mag_temp[3] = {0, 0, 0};
    float mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

    ESP_LOGI(TAG_BNO, "Mag Calibration: Wave device in a figure eight until done!");
    BNO055_delay_msek(4000);

    // In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
    IS_BNO_OK(bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG));
    IS_BNO_OK(bno055_set_mag_power_mode(BNO055_MAG_POWER_MODE_FORCE_MODE));
    IS_BNO_OK(bno055_set_mag_data_output_rate(BNO055_MAG_DATA_OUTRATE_20HZ));

    struct bno055_mag_t mag_raw;
    int cnt = 256;
    for (int i = 0; i < cnt; i++)
    {
        IS_BNO_OK(bno055_read_mag_xyz(&mag_raw));

        mag_temp[0] = mag_raw.x;
        mag_temp[1] = mag_raw.y;
        mag_temp[2] = mag_raw.z;

        for (int j = 0; j < 3; j++)
        {
            if (i == 0)
            {
                mag_max[j] = mag_temp[j]; // Offsets may be large enough that mag_temp[i] may not be bipolar!
                mag_min[j] = mag_temp[j]; // This prevents max or min being pinned to 0 if the values are unipolar...
            }
            else
            {
                if (mag_temp[j] > mag_max[j])
                    mag_max[j] = mag_temp[j];
                if (mag_temp[j] < mag_min[j])
                    mag_min[j] = mag_temp[j];
            }
        }
        BNO055_delay_msek(50); // at 20 Hz ODR, new mag data is available every 50 ms
    }

    mag_bias.x = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias.y = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias.z = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    IS_BNO_OK(bno055_write_mag_offset(&mag_bias));

    ESP_LOGD(TAG_BNO, "Mag Bias: X: %d | Y: %d | Z: %d", mag_bias.x, mag_bias.y, mag_bias.z);
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

    ESP_LOGI(TAG_BNO, "Calibration profile loaded successfully!");

    return BNO055_SUCCESS;
}

BNO055_RETURN_FUNCTION_TYPE bno055_calib_routine(uint8_t sensors, mpu_pos_t link)
{
    if (sensors & ACCEL)
        IS_BNO_OK(bno055_calib_accel());

    if (sensors & GYRO)
        IS_BNO_OK(bno055_calib_gyro());

    if (sensors & MAG)
        IS_BNO_OK(bno055_calib_mag());

    if (sensors == NONE)
        IS_BNO_OK(bno055_load_calib_data(link));

    return BNO055_SUCCESS;
}

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
    IS_BNO_OK(bno055_calib_routine(sensors, link));
    IS_BNO_OK(bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF));
    IS_BNO_OK(bno055_set_euler_unit(BNO055_EULER_UNIT_DEG));

    bno055_self_test_routine();
    bno055_get_calib_status();

    ESP_LOGI(TAG_BNO, "BNO Initialised!");

    return BNO055_SUCCESS;
}

void bno055_get_rph(struct bno055_euler_float_t *bno_rph)
{
    s16 heading, pitch, roll;

    if (bno055_read_euler_h(&heading) == BNO055_SUCCESS && bno055_read_euler_p(&pitch) == BNO055_SUCCESS && bno055_read_euler_r(&roll) == BNO055_SUCCESS)
    {
        bno_rph->r = roll / 16.0f;
        bno_rph->p = pitch / 16.0f;
        bno_rph->h = heading / 16.0f;
    }
    else
        ESP_LOGE(TAG_BNO, "Measurement Error!");

    BNO055_delay_msek(10);
}