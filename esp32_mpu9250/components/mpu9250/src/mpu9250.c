#include "mpu9250_register_map.h"
#include "mpu9250.h"

#include "inv_mpu.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static const char *TAG_MPU = "mpu_9250";

static uint8_t mpu9250_orientation;
static uint8_t tap_count;
static uint8_t tap_direction;
static bool _tap_available;

static void orient_cb(uint8_t orient);
static void tap_cb(uint8_t direction, uint8_t count);

static float ax, ay, az;
static float gx, gy, gz;
static float mx, my, mz;
static long qw, qx, qy, qz;
static long temperature;
static unsigned long time;
static float pitch, roll, yaw;

static float mag_bias[3] = {-293.33f, -1780.0f, 1426.67f}, mag_scale[3] = {1.25f, 0.94f, 0.87f};
static float mag_calib[3] = {297, 299, 287};
/*
Bias: X: -293.333313, Y: -1779.999878, Z: 1426.666626
Scale: X: 1.258065, Y: 0.942598, Z: 0.873950
*/
static unsigned short _aSense;
static float _gSense, _mSense;

const signed char defaultOrientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1};

void print_mpu_raw_agm(void)
{
    ESP_LOGI(TAG_MPU, "Ax: %f Ay: %f Az: %f | Gx: %f Gy: %f Gz: %f | Mx: %f My: %f Mz: %f",
             ax, ay, az, gx, gy, gz, mx, my, mz);
}

void print_mpu_raw_quat(void)
{
    ESP_LOGI(TAG_MPU, "Qw: %0.3f Qx: %0.3f Qy: %0.3f Qz: %0.3f",
             calcQuat(qw), calcQuat(qx), calcQuat(qy), calcQuat(qz));
}

void mpu_set_sensitivity(void)
{
    _mSense = 0.15f; // Constant - 4915 / 32760
    _aSense = 0.0f;  // Updated after accel FSR is set
    _gSense = 0.0f;  // Updated after gyro FSR is set
}

inv_error_t begin(void)
{
    inv_error_t result;
    struct int_param_s int_param;

    mpu_set_sensitivity();
    result = mpu_init(&int_param);

    if (result)
        return result;

    mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

    setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    _gSense = getGyroSens();
    _aSense = getAccelSens();

    return result;
}

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

short getIntStatus(void)
{
    short status;
    if (mpu_get_int_status(&status) == INV_SUCCESS)
    {
        return status;
    }
    return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40,
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t lowPowerAccel(unsigned short rate)
{
    return mpu_lp_accel_mode(rate);
}

inv_error_t setGyroFSR(unsigned short fsr)
{
    inv_error_t err;
    err = mpu_set_gyro_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        _gSense = getGyroSens();
    }
    return err;
}

inv_error_t setAccelFSR(uint8_t fsr)
{
    inv_error_t err;
    err = mpu_set_accel_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        _aSense = getAccelSens();
    }
    return err;
}

unsigned short getGyroFSR(void)
{
    unsigned short tmp;
    if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

uint8_t getAccelFSR(void)
{
    uint8_t tmp;
    if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

unsigned short getMagFSR(void)
{
    unsigned short tmp;
    if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setLPF(unsigned short lpf)
{
    return mpu_set_lpf(lpf);
}

unsigned short getLPF(void)
{
    unsigned short tmp;
    if (mpu_get_lpf(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setSampleRate(unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

unsigned short getSampleRate(void)
{
    unsigned short tmp;
    if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setCompassSampleRate(unsigned short rate)
{
    return mpu_set_compass_sample_rate(rate);
}

unsigned short getCompassSampleRate(void)
{
    unsigned short tmp;
    if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }

    return 0;
}

float getGyroSens(void)
{
    float sens;
    if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

unsigned short getAccelSens(void)
{
    unsigned short sens;
    if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

float getMagSens(void)
{
    return 0.15; // Static, 4915/32760
}

uint8_t getFifoConfig(void)
{
    uint8_t sensors;
    if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
    {
        return sensors;
    }
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

unsigned short fifoAvailable(void)
{
    uint8_t fifoH, fifoL;

    if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
        return 0;
    if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
        return 0;

    return (fifoH << 8) | fifoL;
}

inv_error_t updateFifo(void)
{
    short gyro[3], accel[3];
    unsigned long timestamp;
    uint8_t sensors, more;

    if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
        return INV_ERROR;

    if (sensors & INV_XYZ_ACCEL)
    {
        ax = accel[X_AXIS];
        ay = accel[Y_AXIS];
        az = accel[Z_AXIS];
    }
    if (sensors & INV_X_GYRO)
        gx = gyro[X_AXIS];
    if (sensors & INV_Y_GYRO)
        gy = gyro[Y_AXIS];
    if (sensors & INV_Z_GYRO)
        gz = gyro[Z_AXIS];

    time = timestamp;

    return INV_SUCCESS;
}

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

inv_error_t update(uint8_t sensors)
{
    inv_error_t aErr = INV_SUCCESS;
    inv_error_t gErr = INV_SUCCESS;
    inv_error_t mErr = INV_SUCCESS;
    inv_error_t tErr = INV_SUCCESS;

    if (sensors & UPDATE_ACCEL)
        aErr = updateAccel();
    if (sensors & UPDATE_GYRO)
        gErr = updateGyro();
    if (sensors & UPDATE_COMPASS)
        mErr = updateCompass();
    if (sensors & UPDATE_TEMP)
        tErr = updateTemperature();

    return aErr | gErr | mErr | tErr;
}

int updateAccel(void)
{
    short data[3];

    if (mpu_get_accel_reg(data, &time))
    {
        return INV_ERROR;
    }
    ax = (float)data[X_AXIS] / (float)_aSense;
    ay = (float)data[Y_AXIS] / (float)_aSense;
    az = (float)data[Z_AXIS] / (float)_aSense;

    return INV_SUCCESS;
}

int updateGyro(void)
{
    short data[3];

    if (mpu_get_gyro_reg(data, &time))
    {
        return INV_ERROR;
    }
    gx = (float)data[X_AXIS] / (float)_gSense;
    gy = (float)data[Y_AXIS] / (float)_gSense;
    gz = (float)data[Z_AXIS] / (float)_gSense;

    return INV_SUCCESS;
}

int updateCompass(void)
{
    short data[3];

    if (mpu_get_compass_reg(data, &time))
    {
        return INV_ERROR;
    }

    mx = ((float)data[X_AXIS] / _mSense - mag_bias[X_AXIS]) * mag_scale[X_AXIS];
    my = ((float)data[Y_AXIS] / _mSense - mag_bias[Y_AXIS]) * mag_scale[Y_AXIS];
    mz = ((float)data[Z_AXIS] / _mSense - mag_bias[Z_AXIS]) * mag_scale[Z_AXIS];

    return INV_SUCCESS;
}

inv_error_t updateTemperature(void)
{
    return mpu_get_temperature(&temperature, &time);
}

int selfTest(uint8_t debug)
{
    long gyro[3], accel[3];
    return mpu_run_self_test(gyro, accel);
}

inv_error_t dmpBegin(unsigned short features, unsigned short fifoRate)
{
    unsigned short feat = features;
    unsigned short rate = fifoRate;

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

    if (dmpSetOrientation(defaultOrientation) != INV_SUCCESS)
        return INV_ERROR;

    return mpu_set_dmp_state(1);
}

inv_error_t dmpLoad(void)
{
    return dmp_load_motion_driver_firmware();
}

unsigned short dmpGetFifoRate(void)
{
    unsigned short rate;
    if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
        return rate;

    return 0;
}

inv_error_t dmpSetFifoRate(unsigned short rate)
{
    if (rate > MAX_DMP_SAMPLE_RATE)
        rate = MAX_DMP_SAMPLE_RATE;
    return dmp_set_fifo_rate(rate);
}

inv_error_t dmpUpdateFifo(void)
{
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    uint8_t more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != INV_SUCCESS)
    {
        return INV_ERROR;
    }

    if (sensors & INV_XYZ_ACCEL)
    {
        ax = (float)accel[X_AXIS];
        ay = (float)accel[Y_AXIS];
        az = (float)accel[Z_AXIS];
    }
    if (sensors & INV_X_GYRO)
        gx = (float)gyro[X_AXIS];
    if (sensors & INV_Y_GYRO)
        gy = (float)gyro[Y_AXIS];
    if (sensors & INV_Z_GYRO)
        gz = (float)gyro[Z_AXIS];
    if (sensors & INV_WXYZ_QUAT)
    {
        qw = quat[0];
        qx = quat[1];
        qy = quat[2];
        qz = quat[3];
    }

    time = timestamp;

    return INV_SUCCESS;
}

inv_error_t dmpEnableFeatures(unsigned short mask)
{
    unsigned short enMask = 0;
    enMask |= mask;
    // Combat known issue where fifo sample rate is incorrect
    // unless tap is enabled in the DMP.
    enMask |= DMP_FEATURE_TAP;
    return dmp_enable_feature(enMask);
}

unsigned short dmpGetEnabledFeatures(void)
{
    unsigned short mask;
    if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
        return mask;
    return 0;
}

inv_error_t dmpSetTap(unsigned short xThresh, unsigned short yThresh, unsigned short zThresh, uint8_t taps, unsigned short tapTime, unsigned short tapMulti)
{

    uint8_t axes = 0;
    if (xThresh > 0)
    {
        axes |= TAP_X;
        xThresh = constrain(xThresh, 1, 1600);
        if (dmp_set_tap_thresh(1 << X_AXIS, xThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (yThresh > 0)
    {
        axes |= TAP_Y;
        yThresh = constrain(yThresh, 1, 1600);
        if (dmp_set_tap_thresh(1 << Y_AXIS, yThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (zThresh > 0)
    {
        axes |= TAP_Z;
        zThresh = constrain(zThresh, 1, 1600);
        if (dmp_set_tap_thresh(1 << Z_AXIS, zThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (dmp_set_tap_axes(axes) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_count(taps) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
        return INV_ERROR;

    dmp_register_tap_cb(tap_cb);

    return INV_SUCCESS;
}

uint8_t getTapDir(void)
{
    _tap_available = false;
    return tap_direction;
}

uint8_t getTapCount(void)
{
    _tap_available = false;
    return tap_count;
}

bool tapAvailable(void)
{
    return _tap_available;
}

inv_error_t dmpSetOrientation(const signed char *orientationMatrix)
{
    unsigned short scalar;
    scalar = orientation_row_2_scale(orientationMatrix);
    scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
    scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;

    dmp_register_android_orient_cb(orient_cb);

    return dmp_set_orientation(scalar);
}

uint8_t dmpGetOrientation(void)
{
    return mpu9250_orientation;
}

inv_error_t dmpEnable3Quat(void)
{
    unsigned short dmpFeatures;

    // 3-axis and 6-axis quat are mutually exclusive
    dmpFeatures = dmpGetEnabledFeatures();
    dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
    dmpFeatures |= DMP_FEATURE_LP_QUAT;

    if (dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
        return INV_ERROR;

    return dmp_enable_lp_quat(1);
}

unsigned long dmpGetPedometerSteps(void)
{
    unsigned long steps;
    if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS)
    {
        return steps;
    }
    return 0;
}

inv_error_t dmpSetPedometerSteps(unsigned long steps)
{
    return dmp_set_pedometer_step_count(steps);
}

unsigned long dmpGetPedometerTime(void)
{
    unsigned long walkTime;
    if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS)
    {
        return walkTime;
    }
    return 0;
}

inv_error_t dmpSetPedometerTime(unsigned long time)
{
    return dmp_set_pedometer_walk_time(time);
}

float calcQuat(long axis)
{
    return qToFloat(axis, 30);
}

float qToFloat(long number, uint8_t q)
{
    unsigned long mask = 0;
    for (int i = 0; i < q; i++)
    {
        mask |= (1 << i);
    }
    return (number >> q) + ((number & mask) / (float)(2 << (q - 1)));
}

void computeEulerAngles(bool degrees)
{
    float dqw = qToFloat(qw, 30);
    float dqx = qToFloat(qx, 30);
    float dqy = qToFloat(qy, 30);
    float dqz = qToFloat(qz, 30);

    double ysqr = dqy * dqy;

    // Roll (x-axis rotation)
    double t0 = +2.0 * (dqw * dqx + dqy * dqz);
    double t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
    roll = atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2);

    // Yaw (z-axis rotation)
    double t3 = +2.0 * (dqw * dqz + dqx * dqy);
    double t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    yaw = atan2(t3, t4);

    if (degrees)
    {
        pitch *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;
    }

    ESP_LOGI(TAG_MPU, "Roll: %0.2f | Pitch: %0.2f | Yaw: %0.2f", roll, pitch, yaw);
}

unsigned short orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

static void tap_cb(uint8_t direction, uint8_t count)
{
    _tap_available = true;
    tap_count = count;
    tap_direction = direction;
}

static void orient_cb(uint8_t orient)
{
    mpu9250_orientation = orient;
}

// Compute the accelerometer angle using the raw data
void compute_acce_angle(float *acce_angle)
{
    acce_angle[0] = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * RAD_TO_DEG;
    acce_angle[1] = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * RAD_TO_DEG;
}

// Compute the gyroscope angle using the raw data
void compute_gyro_angle(float dt, float *gyro_angle)
{
    gyro_angle[0] = gx * dt;
    gyro_angle[1] = gy * dt;
    // gyro_angle[2] = gz * dt;

    /*
        In cases the roll angle varies widely when only the pitch angle changes, activate the following equations.
        gyro_angle[0] = dt * (gx + gy * sin(gyro_angle[0]) * tan(gyro_angle[1]) + gz * cos(gyro_angle[0]) * tan(gyro_angle[1]));
        gyro_angle[1] = dt * (gy * cos(gyro_angle[0]) - gz * sin(gyro_angle[0]));
        gyro_angle[2] = dt * (gy * sin(gyro_angle[0]) / cos(gyro_angle[1]) + gz * cos(gyro_angle[0]) / sin(gyro_angle[1]));
    */
}

void compute_mag_angle(float *mag_angle)
{
    if (my == 0)
        *mag_angle = (mx < 0) ? 180.0 : 0;
    else
        *mag_angle = atan2(my, mx) * RAD_TO_DEG + 180;
}

// Fuse the gyroscope and accelerometer angle in a complementary fashion
void complementary_filter(void)
{
    static bool is_initial_reading = true;
    static uint32_t timer;
    static float dt;

    static float acce_angle[2], gyro_angle[2], mag_angle;
    static float fusion_angle[2];
    int i = 0;

    if (is_initial_reading)
    {
        is_initial_reading = false;
        compute_acce_angle(acce_angle);

        for (i = 0; i < 2; i++)
            fusion_angle[i] = acce_angle[i];

        timer = esp_timer_get_time();
        return;
    }

    dt = (float)(esp_timer_get_time() - timer) / 1000000;
    timer = esp_timer_get_time();

    compute_acce_angle(acce_angle);
    compute_gyro_angle(dt, gyro_angle);
    compute_mag_angle(&mag_angle);

    for (i = 0; i < 2; i++)
        fusion_angle[i] = ALPHA * (fusion_angle[i] + gyro_angle[i]) + (1 - ALPHA) * acce_angle[i];
    
    ESP_LOGI(TAG_MPU, "Roll: %0.2f | Pitch: %0.2f | Yaw: %0.2f", fusion_angle[0], fusion_angle[1], mag_angle);
}

// Calibrate magnetometer -> To-be-fixed in future iterations
void calibrate_mag(void)
{
    float mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    ESP_LOGI(TAG_MPU, "Mag Calibration: Wave device in a figure eight until done!");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    uint16_t sample_count = 20480; // at 100 Hz ODR, new mag data is available every 10 ms
    for (int i = 0; i < sample_count; i++)
    {
        if (updateCompass() == INV_SUCCESS) // Read the mag data
        {
            mag_temp[0] = mx;
            mag_temp[1] = my;
            mag_temp[2] = mz;

            for (int j = 0; j < 3; j++)
            {
                if (mag_temp[j] > mag_max[j])
                    mag_max[j] = mag_temp[j];
                if (mag_temp[j] < mag_min[j])
                    mag_min[j] = mag_temp[j];
            }
        }
    }

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2.0; // get bias in x axis
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2.0; // get bias in y axis
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2.0; // get bias in z axis

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2.0; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2.0; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2.0; // get average z axis max chord length in counts

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0;

    mag_scale[0] = avg_rad / mag_scale[0]; // mag scales in Gauss
    mag_scale[1] = avg_rad / mag_scale[1];
    mag_scale[2] = avg_rad / mag_scale[2];

    ESP_LOGI(TAG_MPU, "Magnetometer Calibration Done!");
    ESP_LOGD(TAG_MPU, "Bias: X: %f, Y: %f, Z: %f", mag_bias[0], mag_bias[1], mag_bias[2]);
    ESP_LOGD(TAG_MPU, "Scale: X: %f, Y: %f, Z: %f", mag_scale[0], mag_scale[1], mag_scale[2]);
}