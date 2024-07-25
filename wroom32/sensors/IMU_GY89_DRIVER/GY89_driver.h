#ifndef GY89_driver_h
#define GY89_driver_h


#include <Wire.h>


#define LSM303D_ADDRESS   0x1D
#define L3GD20H_ADDR      0x6B // Dirección I2C del L3GD20H

#define CTRL1_REG         0x20
#define CTRL2_REG         0x21
#define CTRL3_REG         0x22
#define CTRL4_REG         0x23
#define CTRL5_REG         0x24
#define CTRL6_REG         0x25
#define CTRL7_REG         0x26


#define ACCEL_XOUT_REG    0X28
#define MAG_XOUT_REG      0X08
#define GYRO_XOUT_REG     0X28



#define MED_MAG_DECLINATION -7.34       //degrees
#define MED_MAG_INCLINATION 29.28       //degrees

#define RIONEGRO_MAG_DECLINATION -7.4   //degrees
#define RIONEGRO_MAG_INCLINATION 29.17  //degrees


#define ACCEL_SCALE_MSK  0x38
#define MAG_SCALE_MSK    0x60
#define GYRO_SCALE_MSK  0x30




/* ------------------------------------------------ SENSORS SCALES ------------------------------------------------ */

typedef enum {
  LSM303D_RANGE_2_G   = 0x0,  ///< +/- 2g (default value)
  LSM303D_RANGE_4_G   = 0x04,  ///< +/- 4g
  LSM303D_RANGE_6_G   = 0x10,  ///< +/- 6g
  LSM303D_RANGE_8_G   = 0x18,  ///< +/- 8g
  LSM303D_RANGE_16_G  = 0x20, ///< +/- 16g
} LSM303D_accel_range_t;

typedef enum {
  LSM303D_RANGE_2_GAUSS   = 0x0,  ///< +/- 2gauss (default value)
  LSM303D_RANGE_4_GAUSS   = 0x04,  ///< +/- 4gauss
  LSM303D_RANGE_8_GAUSS   = 0x18,  ///< +/- 8g
  LSM303D_RANGE_12_GAUSS  = 0x20, ///< +/- 12g
} LSM303D_mag_range_t;

typedef enum {
  L3GD20H_RANGE_245_DEG   = 0x0,  ///< +/- 245 deg/s
  L3GD20H_RANGE_500_DEG   = 0x10,  ///< +/- 500 deg/s
  L3GD20H_RANGE_2000_DEG   = 0x20,  ///< +/- 2000 deg/s
} L3GD20H_gyro_range_t;


struct Angles {
    float roll;
    float pitch;
    float yaw;
};


/*-------------------------------------------------------------------------------------------------*/

class GY89 {
  public:
    GY89();//constructor

    void begin();
    const float* getAccel();
    const float* getMag();
    void getGyro();
    const float* calculateAngles(const float* accelData);

    void printAccel();
    void printMag();
    void printGyro();



        // Métodos para configurar la escala
    void setAccelRange(LSM303D_accel_range_t range);
    void setMagRange(LSM303D_mag_range_t range);
    void setGyroRange(L3GD20H_gyro_range_t range);
    
    void setAccelDR();
  
  private:
    float accelData[3]; // Arreglo estático miembro de la clase
    float gyroData[3]; // Arreglo estático miembro de la clase
    float magData[3];
    float adjGyroData[3]; // Arreglo estático miembro de la clase
    float anglesData[3]; // Arreglo estático miembro de la clase
    float tempData; // Arreglo estático miembro de la clase
    float rateCalibrationArray[3] = {0.0f,0.0f,0.0f}; // Arreglo estático miembro de la clase
    void rawAccelData(int16_t* accelData);
    void rawGyroData(int16_t* gyroData);
    void rawMagData(int16_t* magData);

    float accelLSB = 0;
    float magLSB = 0;
    float gyroLSB = 0;


    void writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t deviceAddress, uint8_t reg);
    void readRegisters(uint8_t deviceAddress, uint8_t reg, uint8_t length, uint8_t *data);
};


#endif