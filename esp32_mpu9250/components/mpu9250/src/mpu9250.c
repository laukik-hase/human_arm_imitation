#include "mpu9250_register_map.h"
#include "mpu9250.h"

#include "inv_mpu.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static const char *TAG_MPU = "mpu_9250";

static uint32_t time;

///////////////////////////////////////////////////////////////////////////////
// Misc utiltiy functions

void printMPUInfo(mpu_data_t *mpu)
{
    ESP_LOGD(TAG_MPU, "Acce FSR => ±%d g | Gyro FSR => %d dps | Compass FSR => ±%d uT",
             getAccelFSR(), getGyroFSR(), getMagFSR());
    ESP_LOGD(TAG_MPU, "Acce & Gyro Rate => %d Hz | Compass Rate => %d Hz",
             getSampleRate(), getCompassSampleRate());

    ESP_LOGD(TAG_MPU, "Acce Bias => X: %0.3f | Y: %0.3f | Z: %0.3f",
             mpu->acce_bias[X_AXIS], mpu->acce_bias[Y_AXIS], mpu->acce_bias[Z_AXIS]);
    ESP_LOGD(TAG_MPU, "Gyro Bias => X: %0.3f | Y: %0.3f | Z: %0.3f",
             mpu->gyro_bias[X_AXIS], mpu->gyro_bias[Y_AXIS], mpu->gyro_bias[Z_AXIS]);

    ESP_LOGD(TAG_MPU, "Compass Bias => X: %0.3f | Y: %0.3f | Z: %0.3f",
             mpu->mag_bias[X_AXIS], mpu->mag_bias[Y_AXIS], mpu->mag_bias[Z_AXIS]);
    ESP_LOGD(TAG_MPU, "Compass Scale => X: %0.3f | Y: %0.3f | Z: %0.3f",
             mpu->mag_scale[X_AXIS], mpu->mag_scale[Y_AXIS], mpu->mag_scale[Z_AXIS]);
}

float qToFloat(int32_t number, uint8_t q)
{
    return (float)((double)number / (double)((int32_t)1 << q));
}

int32_t floatToQ(float number, uint8_t q)
{
    return (int32_t)(number * ((int32_t)1 << q));
}

///////////////////////////////////////////////////////////////////////////////
// Interrupt Settings

inv_error_t enableInterrupt(uint8_t enable)
{
    return set_int_enable(enable);
}

inv_error_t setIntLevel(uint8_t active_low)
{
    return mpu_set_int_level(active_low);
}

inv_error_t setIntLatched(uint8_t enable)
{
    return mpu_set_int_latched(enable);
}

int16_t getIntStatus(void)
{
    int16_t status;
    if (mpu_get_int_status(&status) == INV_SUCCESS)
    {
        return status;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Sensor Settings - FSR, Sample Rate, LPF, Senstivity

inv_error_t setGyroFSR(uint16_t fsr)
{
    return mpu_set_gyro_fsr(fsr);
}

uint16_t getGyroFSR(void)
{
    uint16_t tmp;
    if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
        return tmp;
    
    return 0;
}

inv_error_t setAccelFSR(uint8_t fsr)
{
    return mpu_set_accel_fsr(fsr);
}

uint8_t getAccelFSR(void)
{
    uint8_t tmp;
    if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
        return tmp;

    return 0;
}

uint16_t getMagFSR(void)
{
    uint16_t tmp;
    if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
        return tmp;

    return 0;
}

inv_error_t setLPF(uint16_t lpf)
{
    return mpu_set_lpf(lpf);
}

uint16_t getLPF(void)
{
    uint16_t tmp;
    if (mpu_get_lpf(&tmp) == INV_SUCCESS)
        return tmp;
    
    return 0;
}

inv_error_t setSampleRate(uint16_t rate)
{
    return mpu_set_sample_rate(rate);
}

uint16_t getSampleRate(void)
{
    uint16_t tmp;
    if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
        return tmp;
    
    return 0;
}

inv_error_t setCompassSampleRate(uint16_t rate)
{
    return mpu_set_compass_sample_rate(rate);
}

uint16_t getCompassSampleRate(void)
{
    uint16_t tmp;
    if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
        return tmp;

    return 0;
}

inv_error_t setMPUSensitivity(mpu_data_t *mpu)
{
    mpu->m_sens = COMPASS_SENSITIVITY; // Constant - 32760 / 4915
    mpu->a_sens = getAccelSens();      // Updated after accel FSR is set
    mpu->g_sens = getGyroSens();       // Updated after gyro FSR is set

    return INV_SUCCESS;
}

float getGyroSens(void)
{
    float sens;
    if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
        return sens;
    
    return 0;
}

float getAccelSens(void)
{
    uint16_t sens;
    if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
        return (float)sens;
    
    return 0;
}

float getMagSens(void)
{
    return 6.665f;
}

///////////////////////////////////////////////////////////////////////////////
// Sensor Settings - Self-tests and Calibration

inv_error_t mpuSelfTest(mpu_data_t *mpu)
{
    int32_t gyro[3], accel[3];
    int8_t res = mpu_run_6500_self_test(gyro, accel, false);

    // NOTE: Acce and Gyro self-test failures are fatal. Stabilize the device and try again.
    // Compass self-test result can be ignored; calibration takes care of it.

    if (!(res & GYRO_SELF_TEST))
        ESP_LOGE(TAG_MPU, "Gyro Self Test Failed!");
    if (!(res & ACCEL_SELF_TEST))
        ESP_LOGE(TAG_MPU, "Acce Self Test Failed!");
    if (!(res & COMPASS_SELF_TEST))
        ESP_LOGW(TAG_MPU, "Compass Self Test Failed!");

    mpu->gyro_bias[X_AXIS] = qToFloat(gyro[X_AXIS], BIAS_Q_FORMAT_N);
    mpu->gyro_bias[Y_AXIS] = qToFloat(gyro[Y_AXIS], BIAS_Q_FORMAT_N);
    mpu->gyro_bias[Z_AXIS] = qToFloat(gyro[Z_AXIS], BIAS_Q_FORMAT_N);

    mpu->acce_bias[X_AXIS] = qToFloat(accel[X_AXIS], BIAS_Q_FORMAT_N);
    mpu->acce_bias[Y_AXIS] = qToFloat(accel[Y_AXIS], BIAS_Q_FORMAT_N);
    mpu->acce_bias[Z_AXIS] = qToFloat(accel[Z_AXIS], BIAS_Q_FORMAT_N);

    return INV_SUCCESS;
}

inv_error_t compassCalibration(mpu_data_t *mpu)
{
    float mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    ESP_LOGI(TAG_MPU, "Wave device in a figure eight until done!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    int sample_count = 1536;
    int16_t data[3];

    for (int i = 0; i < sample_count; i++)
    {
        mpu_get_compass_reg(data, &time);

        mag_temp[X_AXIS] = (float)data[X_AXIS] / mpu->m_sens;
        mag_temp[Y_AXIS] = (float)data[Y_AXIS] / mpu->m_sens;
        mag_temp[Z_AXIS] = (float)data[Z_AXIS] / mpu->m_sens;

        for (int j = X_AXIS; j <= Z_AXIS; j++)
        {
            if (mag_temp[j] > mag_max[j])
                mag_max[j] = mag_temp[j];
            if (mag_temp[j] < mag_min[j])
                mag_min[j] = mag_temp[j];
        }

        // at 100 Hz ODR, new mag data is available every 10 ms
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Get hard iron correction
    mpu->mag_bias[X_AXIS] = (mag_max[X_AXIS] + mag_min[X_AXIS]) / 2.0f; // get bias in x axis
    mpu->mag_bias[Y_AXIS] = (mag_max[Y_AXIS] + mag_min[Y_AXIS]) / 2.0f; // get bias in y axis
    mpu->mag_bias[Z_AXIS] = (mag_max[Z_AXIS] + mag_min[Z_AXIS]) / 2.0f; // get bias in z axis

    // Get soft iron correction estimate
    mpu->mag_scale[X_AXIS] = (mag_max[X_AXIS] - mag_min[X_AXIS]) / 2.0f; // get average x axis max chord length in counts
    mpu->mag_scale[Y_AXIS] = (mag_max[Y_AXIS] - mag_min[Y_AXIS]) / 2.0f; // get average y axis max chord length in counts
    mpu->mag_scale[Z_AXIS] = (mag_max[Z_AXIS] - mag_min[Z_AXIS]) / 2.0f; // get average z axis max chord length in counts

    float avg_rad = (mpu->mag_scale[X_AXIS] + mpu->mag_scale[Y_AXIS] + mpu->mag_scale[Z_AXIS]) / 3.0;

    mpu->mag_scale[X_AXIS] = avg_rad / mpu->mag_scale[X_AXIS]; // mag scales in Gauss
    mpu->mag_scale[Y_AXIS] = avg_rad / mpu->mag_scale[Y_AXIS];
    mpu->mag_scale[Z_AXIS] = avg_rad / mpu->mag_scale[Z_AXIS];

    ESP_LOGI(TAG_MPU, "Magnetometer Calibration Done!");
    return INV_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// FIFO Configuration

uint8_t getFifoConfig(void)
{
    uint8_t sensors;
    if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
        return sensors;
    
    return 0;
}

inv_error_t configureFifo(uint8_t sensors)
{
    return mpu_configure_fifo(sensors);
}

inv_error_t resetFifo(void)
{
    return mpu_reset_fifo();
}

uint16_t fifoAvailable(void)
{
    uint8_t fifoH, fifoL;

    if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
        return 0;
    if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
        return 0;

    return (fifoH << 8) | fifoL;
}

inv_error_t updateFifo(mpu_data_t *mpu)
{
    int16_t gyro[3], accel[3];
    uint32_t timestamp;
    uint8_t sensors, more;

    if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
        return INV_ERROR;

    if (sensors & INV_XYZ_ACCEL)
    {
        mpu->acce_raw[X_AXIS] = (float)accel[X_AXIS];
        mpu->acce_raw[Y_AXIS] = (float)accel[Y_AXIS];
        mpu->acce_raw[Z_AXIS] = (float)accel[Z_AXIS];
    }
    if (sensors & INV_X_GYRO)
        mpu->gyro_raw[X_AXIS] = (float)gyro[X_AXIS];
    if (sensors & INV_Y_GYRO)
        mpu->gyro_raw[Y_AXIS] = (float)gyro[Y_AXIS];
    if (sensors & INV_Z_GYRO)
        mpu->gyro_raw[Z_AXIS] = (float)gyro[Z_AXIS];

    time = timestamp;

    return INV_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// Sensor raw values - Set the sensor config, data ready check and update

inv_error_t setSensors(uint8_t sensors)
{
    return mpu_set_sensors(sensors);
}

bool dataReady()
{
    uint8_t intStatusReg;

    if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
    {
        return (intStatusReg & (1 << INT_STATUS_RAW_DATA_RDY_INT));
    }
    return false;
}

inv_error_t update(uint8_t sensors, mpu_data_t *mpu)
{
    inv_error_t aErr = INV_SUCCESS;
    inv_error_t gErr = INV_SUCCESS;
    inv_error_t mErr = INV_SUCCESS;

    if (sensors & UPDATE_ACCEL)
        aErr = updateAccel(mpu);
    if (sensors & UPDATE_GYRO)
        gErr = updateGyro(mpu);
    if (sensors & UPDATE_COMPASS)
        mErr = updateCompass(mpu);

    return aErr | gErr | mErr;
}

inv_error_t updateAccel(mpu_data_t *mpu)
{
    int16_t data[3];

    if (mpu_get_accel_reg(data, &time))
        return INV_ERROR;

    mpu->acce_raw[X_AXIS] = data[X_AXIS] / mpu->a_sens - mpu->acce_bias[X_AXIS];
    mpu->acce_raw[Y_AXIS] = data[Y_AXIS] / mpu->a_sens - mpu->acce_bias[Y_AXIS];
    mpu->acce_raw[Z_AXIS] = data[Z_AXIS] / mpu->a_sens - mpu->acce_bias[Z_AXIS];

    return INV_SUCCESS;
}

inv_error_t updateGyro(mpu_data_t *mpu)
{
    int16_t data[3];

    if (mpu_get_gyro_reg(data, &time))
        return INV_ERROR;

    mpu->gyro_raw[X_AXIS] = data[X_AXIS] / mpu->g_sens - mpu->gyro_bias[X_AXIS];
    mpu->gyro_raw[Y_AXIS] = data[Y_AXIS] / mpu->g_sens - mpu->gyro_bias[Y_AXIS];
    mpu->gyro_raw[Z_AXIS] = data[Z_AXIS] / mpu->g_sens - mpu->gyro_bias[Z_AXIS];

    return INV_SUCCESS;
}

inv_error_t updateCompass(mpu_data_t *mpu)
{
    int16_t data[3];

    if (mpu_get_compass_reg(data, &time))
        return INV_ERROR;

    mpu->mag_raw[X_AXIS] = ((data[X_AXIS] / mpu->m_sens) - mpu->mag_bias[X_AXIS]) * mpu->mag_scale[X_AXIS];
    mpu->mag_raw[Y_AXIS] = ((data[Y_AXIS] / mpu->m_sens) - mpu->mag_bias[Y_AXIS]) * mpu->mag_scale[Y_AXIS];
    mpu->mag_raw[Z_AXIS] = ((data[Z_AXIS] / mpu->m_sens) - mpu->mag_bias[Z_AXIS]) * mpu->mag_scale[Z_AXIS];

    return INV_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// DMP Configuration

inv_error_t dmpBegin(uint16_t features, uint16_t fifoRate)
{
    uint16_t feat = features;
    uint16_t rate = fifoRate;

    if (dmpLoad() != INV_SUCCESS)
        return INV_ERROR;

    // 3-axis and 6-axis LP quat are mutually exclusive.
    // If both are selected, default to 3-axis
    if (feat & DMP_FEATURE_LP_QUAT)
    {
        feat &= ~(DMP_FEATURE_6X_LP_QUAT);
        dmp_enable_lp_quat(1);
    }
    else if (feat & DMP_FEATURE_6X_LP_QUAT)
        dmp_enable_6x_lp_quat(1);

    if (feat & DMP_FEATURE_GYRO_CAL)
        dmp_enable_gyro_cal(1);

    if (dmpEnableFeatures(feat) != INV_SUCCESS)
        return INV_ERROR;

    rate = constrain(rate, 1, 200);
    if (dmpSetFifoRate(rate) != INV_SUCCESS)
        return INV_ERROR;

    return mpu_set_dmp_state(1);
}

inv_error_t dmpLoad(void)
{
    return dmp_load_motion_driver_firmware();
}

uint16_t dmpGetFifoRate(void)
{
    uint16_t rate;
    if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
        return rate;

    return 0;
}

inv_error_t dmpSetFifoRate(uint16_t rate)
{
    if (rate > MAX_DMP_SAMPLE_RATE)
        rate = MAX_DMP_SAMPLE_RATE;
    return dmp_set_fifo_rate(rate);
}

inv_error_t dmpUpdateFifo(mpu_data_t *mpu)
{
    int16_t gyro[3];
    int16_t accel[3];
    int32_t quat[4];
    uint32_t timestamp;
    int16_t sensors;
    uint8_t more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != INV_SUCCESS)
    {
        return INV_ERROR;
    }
    // TODO: Add sensitivity parameters here
    if (sensors & INV_XYZ_ACCEL)
    {
        mpu->acce_raw[X_AXIS] = (float)accel[X_AXIS];
        mpu->acce_raw[Y_AXIS] = (float)accel[Y_AXIS];
        mpu->acce_raw[Z_AXIS] = (float)accel[Z_AXIS];
    }
    if (sensors & INV_X_GYRO)
        mpu->gyro_raw[X_AXIS] = (float)gyro[X_AXIS];
    if (sensors & INV_Y_GYRO)
        mpu->gyro_raw[Y_AXIS] = (float)gyro[Y_AXIS];
    if (sensors & INV_Z_GYRO)
        mpu->gyro_raw[Z_AXIS] = (float)gyro[Z_AXIS];

    if (sensors & INV_WXYZ_QUAT)
    {
        mpu->qw = qToFloat(quat[0], QUATERNION_Q_FORMAT_N);
        mpu->qx = qToFloat(quat[1], QUATERNION_Q_FORMAT_N);
        mpu->qy = qToFloat(quat[2], QUATERNION_Q_FORMAT_N);
        mpu->qz = qToFloat(quat[3], QUATERNION_Q_FORMAT_N);
    }

    time = timestamp;
    return INV_SUCCESS;
}

inv_error_t dmpEnableFeatures(uint16_t mask)
{
    uint16_t enMask = 0;
    enMask |= mask;
    // Combat known issue where fifo sample rate is incorrect
    // unless tap is enabled in the DMP.
    enMask |= DMP_FEATURE_TAP;
    return dmp_enable_feature(enMask);
}

uint16_t dmpGetEnabledFeatures(void)
{
    uint16_t mask;
    if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
        return mask;
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Madgwick Filter - Initialization, Quaternion Update, Euler Angle conversion

inv_error_t initMadgwickFilter(mpu_data_t *mpu)
{
    CHECK(update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS, mpu));
    mpu->timer = esp_timer_get_time();

    return INV_SUCCESS;
}

void computeEulerAngles(mpu_data_t *mpu, bool degrees)
{
    float qw = mpu->qw;
    float qx = mpu->qx;
    float qy = mpu->qy;
    float qz = mpu->qz;

    double qw2 = qw * qw;
    double qx2 = qx * qx;
    double qy2 = qy * qy;
    double qz2 = qz * qz;

    // NOTE: From kriswiner's MPU-9250
    float a12 = 2.0f * (qx * qy + qw * qz);
    float a22 = qw2 + qx2 - qy2 - qz2;
    float a31 = 2.0f * (qw * qx + qy * qz);
    float a32 = 2.0f * (qx * qz - qw * qy);
    float a33 = qw2 - qx2 - qy2 + qz2;

    // NOTE: From Wikipedia
    // float a12 = 2.0f * (qx * qy + qw * qz);
    // float a22 = 1.0f - 2.0f * (qx2 + qy2);
    // float a31 = 2.0f * (qw * qx + qy * qz);
    // float a32 = 2.0f * (qx * qz - qw * qy);
    // float a33 = 1.0f - 2.0f * (qy2 + qz2);

    mpu->pitch = -asinf(a32);
    mpu->roll = atan2f(a31, a33);
    mpu->yaw = atan2f(a12, a22);

    if (degrees)
    {
        mpu->pitch *= RAD_TO_DEG;
        mpu->roll *= RAD_TO_DEG;
        mpu->yaw *= RAD_TO_DEG;

        mpu->yaw += 0.08f; // Declination at Kalyan, India
        if (mpu->yaw < 0)
            mpu->yaw += 360.0f; // Ensure yaw stays between 0 and 360
    }
}

void MadgwickFilter(mpu_data_t *mpu)
{
    // Auxiliary variables to avoid repeated arithmetic
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    float _2q1mx, _2q1my, _2q1mz, _2q2mx;
    float _4bx, _4bz;

    if (dataReady())
        update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS, mpu);

    float dt = (float)(esp_timer_get_time() - mpu->timer) / 1000000;
    mpu->timer = esp_timer_get_time();

/* NOTE: 
    1) The x(y) axis of the accel/gyro is aligned with the y(x) axis of the magnetometer
    2) The magnetometer z-axis (+down) is misaligned with z-axis (+up) of accelerometer and gyro
    3) As gravity is by convention positive down, we need to invert the accel data
    4) Thus, if we set North along the accel (+x), then we have East along the accel (-y) and Down along the accel (-z)
    acce_raw: -ax, ay, az
    gyro_raw: gx * DEG_TO_RAD, -gy * DEG_TO_RAD, -gz * DEG_TO_RAD (as rad/s)
    mag_raw: my, -mx, mz
*/
    float ax = -1.0f * mpu->acce_raw[X_AXIS];
    float ay = mpu->acce_raw[Y_AXIS];
    float az = mpu->acce_raw[Z_AXIS];

    float gx = mpu->gyro_raw[X_AXIS] * DEG_TO_RAD;
    float gy = -1.0f * mpu->gyro_raw[Y_AXIS] * DEG_TO_RAD;
    float gz = -1.0f * mpu->gyro_raw[Z_AXIS] * DEG_TO_RAD;

    float mx = mpu->mag_raw[Y_AXIS];
    float my = -1.0f * mpu->mag_raw[X_AXIS];
    float mz = mpu->mag_raw[Z_AXIS];

    float q1 = mpu->qw;
    float q2 = mpu->qx;
    float q3 = mpu->qy;
    float q4 = mpu->qz;

    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - BETA * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - BETA * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - BETA * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - BETA * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;
    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
    norm = 1.0f / norm;

    mpu->qw = q1 * norm;
    mpu->qx = q2 * norm;
    mpu->qy = q3 * norm;
    mpu->qz = q4 * norm;
}

///////////////////////////////////////////////////////////////////////////////
// MPU Sensors Initialization Routine functions

inv_error_t initMPUData(mpu_data_t *mpu)
{
    // Initializing gyro bias
    mpu->gyro_bias[X_AXIS] = mpu->gyro_bias[Y_AXIS] = mpu->gyro_bias[Z_AXIS] = 0.0f;
    // Initializing gyro bias
    mpu->acce_bias[X_AXIS] = mpu->acce_bias[Y_AXIS] = mpu->acce_bias[Z_AXIS] = 0.0f;

    // Initializing compass bias
    mpu->mag_bias[X_AXIS] = mpu->mag_bias[Y_AXIS] = mpu->mag_bias[Z_AXIS] = 0.0f;
    // Initializing compass scale
    mpu->mag_scale[X_AXIS] = mpu->mag_scale[Y_AXIS] = mpu->mag_scale[Z_AXIS] = 1.0f;

    // Initializing quaternion
    mpu->qw = 1.0f;
    mpu->qx = 0.0f;
    mpu->qy = 0.0f;
    mpu->qz = 0.0f;
    
    return INV_SUCCESS;
}

void loadCompassOffsets(mpu_data_t *mpu)
{
    int pos = mpu->pos;

    switch (pos)
    {
    case MPU_BASE:;
        mpu->mag_bias[X_AXIS] = 5.026f;
        mpu->mag_bias[Y_AXIS] = -6.452f;
        mpu->mag_bias[Z_AXIS] = -3.976;

        mpu->mag_scale[X_AXIS] = 0.980f;
        mpu->mag_scale[Y_AXIS] = 1.013f;
        mpu->mag_scale[Z_AXIS] = 1.007f;
        break;

    case MPU_SHOULDER:;
    case MPU_BICEP:;
    case MPU_FOREARM:;
    case MPU_PALM:;
    default:;
    }
}

inv_error_t begin(mpu_data_t *mpu, uint8_t calib_flag)
{
    // InitialiZing MPU
    CHECK(mpu_init());
    // Place all slaves (including compass) on primary bus
    CHECK(mpu_set_bypass(1));
    // Enable acce, gyro and compass
    CHECK(setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS));
    // Set sample rates for the sensors
    CHECK(setSampleRate(ACCEL_GYRO_SAMPLE_RATE));
    CHECK(setCompassSampleRate(COMPASS_SAMPLE_RATE));
    // Set the full-scale range for the sensors
    CHECK(setAccelFSR(ACCEL_FSR));
    CHECK(setGyroFSR(GYRO_FSR));
    // Set sensitivity for sensors
    CHECK(setMPUSensitivity(mpu));
    // Initialize the data to default settings
    CHECK(initMPUData(mpu));

    if (calib_flag & ACCEL_GYRO_CAL)
        CHECK(mpuSelfTest(mpu));

    if (calib_flag & COMPASS_CAL)
        CHECK(compassCalibration(mpu));

    if (!(calib_flag & COMPASS_CAL))
        loadCompassOffsets(mpu);

    ESP_LOGD(TAG_MPU, "Initialized MPU: %d", mpu->pos);
    return INV_SUCCESS;
}

inv_error_t initMPU(mpu_data_t *mpu, uint8_t calib_flag)
{
    if (i2c_mux_select(mpu->pos))
        return INV_ERROR;

    CHECK(begin(mpu, calib_flag));
    CHECK(initMadgwickFilter(mpu));

    return INV_SUCCESS;
}

inv_error_t initMPUwithDMP(mpu_data_t *mpu, uint8_t calib_flag)
{
    if (i2c_mux_select(mpu->pos))
        return INV_ERROR;

    CHECK(begin(mpu, calib_flag));
    CHECK(dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL,
                   DMP_SAMPLE_RATE));

    return INV_SUCCESS;
}