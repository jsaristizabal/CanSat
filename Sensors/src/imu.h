#ifndef imu_h
#define imu_h

#include <Wire.h>



#define PWR_MGMT_1                    0x6b
#define CONFIG_REG                        0x1A



#define MPU6050_ADDRESS               0x68
#define MPU6050_ACCEL_CONFIG_REG      0x1C
#define MPU6050_GYRO_CONFIG_REG       0x1B
#define MPU6050_ACCEL_XOUT_H_REG      0x3B
#define MPU6050_TEMP_XOUT_H_REG       0x41
#define MPU6050_GYRO_XOUT_H_REG       0x43


#define MPU6050_MOT_THR_REG           0x1F    /// Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR_REG           0x20    /// Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

#define MPU6050_INT_PIN_CFG_REG       0x37
#define MPU6050_INT_ENABLE_REG        0x38
#define MPU6050_INT_STATUS_REG        0x3A




/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
  MPU6050_RANGE_2_G   = 0x0,  ///< +/- 2g (default value)
  MPU6050_RANGE_4_G   = 0x8,  ///< +/- 4g
  MPU6050_RANGE_8_G   = 0x10,  ///< +/- 8g
  MPU6050_RANGE_16_G  = 0x18, ///< +/- 16g
} mpu6050_accel_range_t;



/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
  MPU6050_RANGE_250_DEG   = 0x0,  ///< +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG   = 0x08,  ///< +/- 500 deg/s
  MPU6050_RANGE_1000_DEG  = 0x10, ///< +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG  = 0x18, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;


typedef enum fsync_out {
  MPU6050_FSYNC_OUT_DISABLED,
  MPU6050_FSYNC_OUT_TEMP,
  MPU6050_FSYNC_OUT_GYROX,
  MPU6050_FSYNC_OUT_GYROY,
  MPU6050_FSYNC_OUT_GYROZ,
  MPU6050_FSYNC_OUT_ACCELX,
  MPU6050_FSYNC_OUT_ACCELY,
  MPU6050_FSYNC_OUT_ACCEL_Z,
} mpu6050_fsync_out_t;




/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum {
  MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
  MPU6050_BAND_184_HZ, ///< 184 Hz
  MPU6050_BAND_94_HZ,  ///< 94 Hz
  MPU6050_BAND_44_HZ,  ///< 44 Hz
  MPU6050_BAND_21_HZ,  ///< 21 Hz
  MPU6050_BAND_10_HZ,  ///< 10 Hz
  MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;



/**
 * @brief Accelerometer high pass filter options
 *
 * Allowed values for `setHighPassFilter`.
 */
typedef enum {
  MPU6050_HIGHPASS_DISABLE,
  MPU6050_HIGHPASS_5_HZ,
  MPU6050_HIGHPASS_2_5_HZ,
  MPU6050_HIGHPASS_1_25_HZ,
  MPU6050_HIGHPASS_0_63_HZ,
  MPU6050_HIGHPASS_UNUSED,
  MPU6050_HIGHPASS_HOLD,
} mpu6050_highpass_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum {
  MPU6050_CYCLE_1_25_HZ, ///< 1.25 Hz
  MPU6050_CYCLE_5_HZ,    ///< 5 Hz
  MPU6050_CYCLE_20_HZ,   ///< 20 Hz
  MPU6050_CYCLE_40_HZ,   ///< 40 Hz
} mpu6050_cycle_rate_t;




class MPU6050{
  
  public:
    MPU6050();
    void begin();
    void calibration(float* rateCalibrationArray);
    void readAccelData(float* accelData);
    void readGyroData(float* gyroData);
    void readTempData(float* tempData);
    void calculateAngle(float* anglesData,  float* accelData);

    void printAccel(void);
    void printGyro(void);
    void printTemp(void);
    void printAll(void);
    void reset(void);

    //accel_range_t getAccelRange(void);
    void setAccelRange(mpu6050_accel_range_t);

    //gyro_range_t getGyroRange(void);
    void setGyroRange(mpu6050_gyro_range_t);

    void setInterrupts(void);

    void setLowPassFilter(mpu6050_bandwidth_t lowPassFilter);
    



  private:
    void rawGyroData(int16_t* gyroData);
    void rawAccelData(int16_t* gyroData);
    void writeRegister(uint8_t reg, uint8_t data);
    void readRegisters(uint8_t reg, uint8_t count,uint8_t* data);



};



#endif