#ifndef _MPU9250_H
#define _MPU9250_H

#include "i2c_utils.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU9250
#define AK8963_SECONDARY
#define COMPASS_ENABLED

// TODO: Integrating inv_error_t to esp_err_t OR getting the entire inv_error_t family here
typedef int inv_error_t;
#define INV_SUCCESS 0x00
#define INV_ERROR 0x20

#define CHECK(x)                     \
    do                               \
    {                                \
        inv_error_t __;              \
        if ((__ = x) != INV_SUCCESS) \
            return __;               \
    } while (0);

enum t_axisOrder
{
    X_AXIS, // 0
    Y_AXIS, // 1
    Z_AXIS  // 2
};

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL (1 << 1)
#define UPDATE_GYRO (1 << 2)
#define UPDATE_COMPASS (1 << 3)
#define UPDATE_TEMP (1 << 4)

// Define's for Bias
#define GYRO_SELF_TEST (0x01)
#define ACCEL_SELF_TEST (0x02)
#define COMPASS_SELF_TEST (0x04)

#define ACCEL_GYRO_CAL (0x02)
#define COMPASS_CAL (0x01)

// Define's for Interrupt
#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW 1
#define INT_LATCHED 1
#define INT_NOT_LATCHED 0
#define INT_50US_PULSE 0

// Define's for DMP and FIFO
#define MAX_DMP_SAMPLE_RATE (200) // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE (512)    // Max FIFO buffer size

#define ACCEL_FSR (2)                 // 2, 4, 8, or 16 g-range
#define GYRO_FSR (250)                // 250, 500, 1000, or 2000 dps
#define ACCEL_GYRO_SAMPLE_RATE (1000) // 4 - 1000 Hz
#define COMPASS_SAMPLE_RATE (100)     // 1 - 100Hz
#define DMP_SAMPLE_RATE (200)         // 1 - 200Hz
#define COMPASS_SENSITIVITY (6.665f)  // Constant - ±32768 -> ±4915 uT

#define BIAS_Q_FORMAT_N (16)
#define QUATERNION_Q_FORMAT_N (30)

// Mathematical Constants
#define PI (3.1415926f)
#define RAD_TO_DEG (57.2957795f)
#define DEG_TO_RAD (0.0174533f)

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// gyroscope measurement error in rads/s (start at 40 deg/s)
#define GYRO_MEASUREMENT_ERROR (40.0f * DEG_TO_RAD)
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define GYRO_MEASUREMENT_DRIFT (0.0f * DEG_TO_RAD)
// Madgwick Filter Co-efficients
#define BETA (0.866f * GYRO_MEASUREMENT_ERROR)

/**
 * @brief MPU config, raw data and euler angles
 * 
 */
typedef struct
{
    // MPU ID and last sample time
    mpu_pos_t pos;
    uint32_t timer;
    // MPU Sensitivity and Calibration Parameters
    float a_sens, g_sens, m_sens;
    float acce_bias[3], gyro_bias[3];
    float mag_bias[3], mag_scale[3];
    // MPU Raw Data
    float acce_raw[3];
    float gyro_raw[3];
    float mag_raw[3];
    float qw, qx, qy, qz;
    // MPU Euler Orientation - post filtering
    float roll, pitch, yaw;
} mpu_data_t;

/**
 * @brief Final joint angles
 * 
 */
typedef struct
{
    float shoulder[3];
    float elbow[3];
    float palm[3];
} joint_angle_t;

///////////////////////////////////////////////////////////////////////////////
// Misc utiltiy functions

/**
 * @brief Print the MPU configuration and calibration info.
 * 
 * @param mpu Pointer to MPU configure struct
 */
void printMPUInfo(mpu_data_t *mpu);

/**
 * @brief Convert a QN-format number to a float.
 * 
 * @param number input QN-format number
 * @param q number of bits used to designate the fractional portion of the number (e.g. Q16)
 * @return float 
 */
float qToFloat(int32_t number, uint8_t q);

/**
 * @brief Convert a float to a QN-format number.
 * 
 * @param number input float number
 * @param q number of bits used to designate the fractional portion of the number (e.g. Q30)
 * @return int32_t 
 */
int32_t floatToQ(float number, uint8_t q);

///////////////////////////////////////////////////////////////////////////////
// Interrupt Settings

/**
 * @brief Configure the MPU-9250's interrupt output to indicate when new data is ready.
 * 
 * @param enable 0 to disable, >=1 to enable
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t enableInterrupt(uint8_t enable);

/**
 * @brief Configure the MPU-9250's interrupt to be either active-high or active-low.
 * 
 * @param active_low 0 for active-high, 1 for active-low
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t setIntLevel(uint8_t active_low);

/**
 * @brief Configure the MPU-9250's interrupt to latch or operate as a 50 us pulse.
 * 
 * @param enable 0 to disable, >=1 to enable
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t setIntLatched(uint8_t enable);

/**
 * @brief Reads the MPU-9250's INT_STATUS register, which can indicate what 
 * (if anything) caused an interrupt (e.g. FIFO overflow or data read).

 * @return contents of the INT_STATUS register
 */
int16_t getIntStatus(void);

///////////////////////////////////////////////////////////////////////////////
// Sensor Settings - FSR, Sample Rate, LPF, Senstivity

/**
 * @brief Sets the full-scale range of the gyroscope.
 * 
 * @param fsr Gyro DPS - 250, 500, 1000, or 2000
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t setGyroFSR(uint16_t fsr);

/**
 * @brief Returns the current gyroscope FSR.
 * 
 * @return Current Gyro DPS - 250, 500, 1000, or 2000
 */
uint16_t getGyroFSR(void);

/**
 * @brief Sets the FSR of the accelerometer.
 * 
 * @param fsr Accel g range - 2, 4, 8, or 16
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t setAccelFSR(uint8_t fsr);

/**
 * @brief Returns the current accelerometer FSR.
 * 
 * @return Current Accel g - 2, 4, 8, or 16 
 */
uint8_t getAccelFSR(void);

/**
 * @brief Returns the current magnetometer FSR.
 * 
 * @return Current mag uT range: ±4915 (16-bit resolution)
 */
uint16_t getMagFSR(void);

/**
 * @brief Sets the digital low-pass filter of the accel and gyro.
 * 
 * @param lpf 188, 98, 42, 20, 10, or 5 Hz (defaults to 5 if incorrectly set)
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t setLPF(uint16_t lpf);

/**
 * @brief Returns the set value of the LPF.
 * 
 * @return 5, 10, 20, 42, 98, or 188 if set. 0 if the LPF is disabled. 
 */
uint16_t getLPF(void);

/**
 * @brief Set the gyroscope and accelerometer sample rate to a
 * value between 4Hz and 1000Hz (1kHz).
 * The library will make an attempt to get as close as possible to the
 * requested sample rate.
 * 
 * @param rate Value between 4 and 1000, indicating the desired sample rate in Hz
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t setSampleRate(uint16_t rate);

/**
 * @brief Get the currently set sample rate.
 * May differ slightly from what was set in setSampleRate.
 * 
 * @return set sample rate of the accel/gyro. A value between 4-1000.
 */
uint16_t getSampleRate(void);

/**
 * @brief Set the magnetometer sample rate to a value between 1Hz and 100 Hz.
 * The library will make an attempt to get as close as possible to the
 * requested sample rate.
 * 
 * @param rate Value between 1 and 100, indicating the desired sample rate in Hz
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t setCompassSampleRate(uint16_t rate);

/**
 * @brief Get the currently set magnetometer sample rate.
 * May differ slightly from what was set in setCompassSampleRate.
 * 
 * @return set sample rate of the magnetometer. A value between 1-100 Hz 
 */
uint16_t getCompassSampleRate(void);

/**
 * @brief Set the sensor sensitivity for MPU-9250.
 * Always call ths function after the begin() function.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return Always returns INV_SUCCESS (0) 
 */
inv_error_t setMPUSensitivity(mpu_data_t *mpu);

/**
 * @brief Returns current gyroscope sensitivity. The FSR divided by
 * the resolution of the sensor (signed 16-bit).
 * 
 * @return Currently set gyroscope sensitivity (e.g. 131, 65.5, 32.8, 16.4)
 */
float getGyroSens(void);

/**
 * @brief Returns current accelerometer sensitivity. The FSR
 * divided by the resolution of the sensor (signed 16-bit).
 * 
 * @return Currently set accel sensitivity (e.g. 16384, 8192, 4096, 2048) 
 */
float getAccelSens(void);

/**
 * @brief Returns current magnetometer sensitivity. The FSR
 * divided by the resolution of the sensor (signed 16-bit).
 * 
 * @return Currently set mag sensitivity (e.g. 6.665f)
 */
float getMagSens(void);

///////////////////////////////////////////////////////////////////////////////
// Sensor Settings - Self-tests and Calibration

/**
 * @brief Complete MPU Self test routine.
 * Ensure the hardware is stable and away from magnetic interference.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t mpuSelfTest(mpu_data_t *mpu);

/**
 * @brief Compass Calibration routine.
 * Hold the hardware parallel to the ground and move it in a figure 8 pattern until done.
 * During calibration, ensure that the hardware is stable and is far away from any magnetic interference.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t compassCalibration(mpu_data_t *mpu);

///////////////////////////////////////////////////////////////////////////////
// FIFO Configuration

/**
 * @brief Returns the sensors configured to be read into the FIFO.
 * 
 * @return Combination of INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_Y_GYRO, INV_X_GYRO, or INV_Z_GYRO 
 */
uint8_t getFifoConfig(void);

/**
 * @brief Initialize the FIFO, set it to read from a select set of sensors.
 * Any of the following defines can be combined for the [sensors] parameter:
 * INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
 * 
 * @param sensors Combination of sensors to be read into FIFO (e.g. INV_XYZ_ACCEL | INV_XYZ_GYRO)
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t configureFifo(uint8_t sensors);

/**
 * @brief Resets the FIFO's read/write pointers.
 * 
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t resetFifo(void);

/**
 * @brief  Returns the number of bytes currently filled in the FIFO.
 * 
 * @return Number of bytes filled in the FIFO (up to 512)
 */
uint16_t fifoAvailable(void);

/**
 * @brief Reads from the top of the FIFO, and stores the new data
 * in ax, ay, az, gx, gy, or gz elements of mpu (depending on how 
 * the FIFO is configured).
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t updateFifo(mpu_data_t *mpu);

///////////////////////////////////////////////////////////////////////////////
// Sensor raw values - Set the sensor config, data ready check and update

inv_error_t setSensors(uint8_t sensors);

/**
 * @brief Checks to see if new accel/gyro data is available.
 * New magnetometer data cannot be checked, as the library runs that sensor
 * in single-conversion mode.
 * 
 * @return true new accel/gyro data is available
 */
bool dataReady();

/**
 * @brief Reads latest data from the MPU-9250's data registers.
 * Sensors to be updated can be set using the [sensors] parameter..
 * 
 * @param sensors any combination of UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and UPDATE_TEMP.
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error 
 * @note After a successful update the instance sensor variables
 * (e.g. ax, ay, az, gx, gy, gz) will be updated with new data
 */
inv_error_t update(uint8_t sensors, mpu_data_t *mpu);

/**
 * @brief Read from accelerometer and update the instance variables - ax, ay, az.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t updateAccel(mpu_data_t *mpu);

/**
 * @brief Read from gyroscope and update the instance variables - gx, gy, gz.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t updateGyro(mpu_data_t *mpu);

/**
 * @brief Read from compass and update the instance variables - mx, my, mz.
 *
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t updateCompass(mpu_data_t *mpu);

///////////////////////////////////////////////////////////////////////////////
// DMP Configuration

/**
 * @brief Initialize the DMP, enable one or more features, and set the FIFO's sample rate.
 * 
 * @param features Can be any one or an OR'd combination of -
 * DMP_FEATURE_TAP -- Tap detection.
 * DMP_FEATURE_ANDROID_ORIENT -- Orientation (portrait/landscape) detection.
 * DMP_FEATURE_LP_QUAT -- Accelerometer, low-power quaternion calculation.
 * DMP_FEATURE_PEDOMETER -- Pedometer (always enabled).
 * DMP_FEATURE_6X_LP_QUAT -- 6-axis (accel/gyro) quaternion calculation.
 * DMP_FEATURE_GYRO_CAL -- Gyroscope calibration (0's out after 8 seconds of no motion).
 * DMP_FEATURE_SEND_RAW_ACCEL -- Send raw accelerometer values to FIFO.
 * DMP_FEATURE_SEND_RAW_GYRO -- Send raw gyroscope values to FIFO.
 * DMP_FEATURE_SEND_CAL_GYRO -- Send calibrated gyroscop values to FIFO.
 * @param fifoRate FIFO Data rate Between 4 and 200Hz.
 * @return INV_SUCCESS (0) on success, otherwise error
 * @note The Tap, Orientation and Pedometer features are yet to be added in this library, 
 * but you can use them directly from the motion driver source files.
 */
inv_error_t dmpBegin(uint16_t features, uint16_t fifoRate);

/**
 * @brief Loads the DMP with 3062-byte image memory. Must be called to begin DMP.
 * This function is called by the dmpBegin function.
 * 
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t dmpLoad(void);

/**
 * @brief Returns the sample rate of the FIFO.
 * 
 * @return Set sample rate, in Hz, of the FIFO
 */
uint16_t dmpGetFifoRate(void);

/**
 * @brief Sets the rate of the FIFO.
 * 
 * @param rate Requested sample rate in Hz (range: 4-200)
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t dmpSetFifoRate(uint16_t rate);

/**
 * @brief Reads from the top of the FIFO and fills accelerometer, gyroscope,
 * quaternion, and time instance variables (depending on how the DMP is configured).
 * Should be called whenever an MPU interrupt is detected.
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t dmpUpdateFifo(mpu_data_t *mpu);

/**
 * @brief Enable one, or multiple DMP features.
 * 
 * @param mask OR'd list of features (see dmpBegin)
 * @return INV_SUCCESS (0) on success, otherwise error 
 */
inv_error_t dmpEnableFeatures(uint16_t mask);

/**
 * @brief Returns the OR'd list of enabled DMP features.
 * 
 * @return OR'd list of DMP feature's (see dmpBegin) 
 */
uint16_t dmpGetEnabledFeatures(void);

///////////////////////////////////////////////////////////////////////////////
// Madgwick Filter - Initialization, Quaternion Update, Euler Angle conversion

/**
 * @brief Initialise the Madgwick Filter
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t initMadgwickFilter(mpu_data_t *mpu);

/**
 * @brief Compute euler angles based on most recently read qw, qx, qy, and qz.
 * 
 * @param mpu Pointer to MPU configure struct
 * @param degrees boolean indicating whether angle results are presented in degrees or radians
 */
void computeEulerAngles(mpu_data_t *mpu, bool degrees);

/**
 * @brief Madgwick Filter routine - calculate the Quaternion values 
 * based on the latest values of accel, gyro and compass.
 * 
 * @param mpu Pointer to MPU configure struct
 */
void MadgwickFilter(mpu_data_t *mpu);

///////////////////////////////////////////////////////////////////////////////
// MPU Sensors Initialization Routine functions

/**
 * @brief Initialise the MPU configure struct
 * 
 * @param mpu Pointer to MPU configure struct
 * @return INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t initMPUData(mpu_data_t *mpu);

/**
 * @brief Load the supplied (already computed) compass offsets
 * 
 * @param mpu Pointer to MPU configure struct
 */
void loadCompassOffsets(mpu_data_t *mpu);

/**
 * @brief Verifies communication with the MPU-9250 and the AK8963 and initializes them to the default state.
 * All sensors enabled
 * Gyro FSR: +/- 250 dps
 * Accel FSR: +/- 2g
 * Gyro-Accel sample rate: 1000 Hz
 * Compass sample rate: 100 Hz
 * LPF: 42 Hz
 * FIFO: 50 Hz, disabled
 * 
 * @param mpu Pointer to MPU configure struct
 * @param calib_flag  Flags for sensor calibration - can be any one or an OR'd combination of ACCEL_GYRO_CAL and COMPASS_CAL
 * @return inv_error_t INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t begin(mpu_data_t *mpu, uint8_t calib_flag);

/**
 * @brief Select the appropriate MPU (through I2C Mux), initialise sensors and filters
 * 
 * @param mpu Pointer to MPU configure struct
 * @param calib_flag  Flags for sensor calibration - can be any one or an OR'd combination of ACCEL_GYRO_CAL and COMPASS_CAL
 * @return inv_error_t INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t initMPU(mpu_data_t *mpu, uint8_t calib_flag);

/**
 * @brief Select the appropriate MPU (through I2C Mux), initialise MPU-9250 and DMP
 * 
 * @param mpu Pointer to MPU configure struct
 * @param calib_flag  Flags for sensor calibration - can be any one or an OR'd combination of ACCEL_GYRO_CAL and COMPASS_CAL
 * @return inv_error_t INV_SUCCESS (0) on success, otherwise error
 */
inv_error_t initMPUwithDMP(mpu_data_t *mpu, uint8_t calib_flag);

#endif