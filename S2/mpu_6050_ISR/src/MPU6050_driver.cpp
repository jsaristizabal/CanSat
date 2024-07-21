#include <cstdint>
#include "MPU6050_driver.h"
#include <Arduino.h>

MPU6050::MPU6050() {}


void MPU6050::turnOn(){
  writeRegister(PWR_MGMT_1, 0x0);

}

void MPU6050::begin(mpu6050_accel_range_t accRange, mpu6050_gyro_range_t gyroRange){
  turnOn();
  setAccelRange(accRange);
  setGyroRange(gyroRange);
}

void MPU6050::readAccelData(float* accelData){
  int16_t rawData[3];
  rawAccelData(rawData);

  accelData[0] = rawData[0] / 2048.0  +0.02;
  accelData[1] = rawData[1] / 2048.0  -0.01;
  accelData[2] = rawData[2] / 2048.0  -0.08;
}

void MPU6050::readGyroData(float* gyroData){
  int16_t rawData[3];
  rawGyroData(rawData);

  gyroData[0] = rawData[0] / 65.5;
  gyroData[1] = rawData[1] / 65.5;
  gyroData[2] = rawData[2] / 65.5;
}

void MPU6050::calculateAngle(float* anglesData, float* accelData){
  anglesData[0]   = atan(accelData[1] / sqrt(accelData[0]*accelData[0] + accelData[2]*accelData[2])) * 1/(3.142/180);   //angleRoll
  anglesData[1]   = atan(-accelData[0] / sqrt(accelData[1]*accelData[1] + accelData[2]*accelData[2])) * 1/(3.142/180);  //anglePitch
  anglesData[2]   = atan(sqrt(accelData[0]*accelData[0] + accelData[1]*accelData[1]) / accelData[2] ) * 1/(3.142/180);  //angleYaw
}




void MPU6050::readTempData(float* tempData){
  uint8_t rawData[2];
  readRegisters(MPU6050_TEMP_XOUT_H_REG, 2, rawData);
  int16_t rawTemp = ((int16_t)rawData[0] << 8) | rawData[1];
  *tempData = (rawTemp/340.0) +36.53;
}


void MPU6050::writeRegister(uint8_t reg,uint8_t data){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void MPU6050::calibration(float * rateCalibrationArray){
  //float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
  int rateCalibrationNumber;
  //float accelData[3];
  float gyroData[3];
  //float anglesData[3];
  //begin();
  //setAccelRange(MPU6050_RANGE_16_G);
  //setGyroRange(MPU6050_RANGE_500_DEG);
  //Wire.beginTransmission(MPU6050_ADDRESS);
  for(rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++){

    //read data
    //imu.readAccelData(accelData);
    readGyroData(gyroData);
    //imu.calculateAngle(anglesData, accelData);

    //rateCalibrationRoll   += gyroData[0];
  //  rateCalibrationPitch  += gyroData[1];
//    rateCalibrationYaw    += gyroData[2];


    //acummulate data for calibration
    // [°/s]
    rateCalibrationArray[0]  += gyroData[0];
    rateCalibrationArray[1]  += gyroData[1];
    rateCalibrationArray[2]  += gyroData[2];

    //[°]
    //rateCalibrationArray[3]  += anglesData[2];
    //rateCalibrationArray[4]  += anglesData[2];
    //ateCalibrationArray[5]  += anglesData[2];

    delay(1);
  }
  rateCalibrationArray[0] /= 2000;
  rateCalibrationArray[1] /= 2000;
  rateCalibrationArray[2] /= 2000;
  Serial.println("Calibration complete!");
}


void MPU6050::readRegisters(uint8_t reg, uint8_t count, uint8_t* data){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, count);
  for(uint8_t i=0; i<count;i++){
    data[i] = Wire.read();
  }
}




void MPU6050::setLowPassFilter(mpu6050_bandwidth_t lowPassFilter){
  writeRegister(CONFIG_REG, lowPassFilter);
  
  
}

void MPU6050::setAccelRange(mpu6050_accel_range_t newRange){
  writeRegister(MPU6050_ACCEL_CONFIG_REG, newRange);
  
}

void MPU6050::setGyroRange(mpu6050_gyro_range_t newRange){
  writeRegister(MPU6050_GYRO_CONFIG_REG, newRange);
  
}


void MPU6050::setupMotion(void){
  writeRegister(MPU6050_MOT_DUR_REG, 40);
  writeRegister(MPU6050_MOT_THR_REG,1);
}

void MPU6050::setInterrupts(void){
  writeRegister(SIGNAL_PATH_RESET_REG, 0x7);// logic level 0=HIGH ; 1= LOW;
  writeRegister(MPU6050_INT_PIN_CFG_REG, 140);// logic level 0=HIGH ; 1= LOW;
  //writeRegister(MPU6050_INT_PIN_CFG_REG, 0<<6);// 0= PUSH-PULL; 1=OPEN-DRAIN
  //writeRegister(MPU6050_INT_PIN_CFG_REG, 0<<5);// 0= 50us pulse ; 1= high until interrupt is cleared

  writeRegister(MPU6050_MOT_THR_REG, 1);
  writeRegister(MPU6050_MOT_DUR_REG, 40);
  writeRegister(MPU6050_MOT_DETECT_CTRL_REG, 0x15);
  writeRegister(MPU6050_INT_ENABLE_REG, 0x40);
}


void MPU6050::reset(void){  
  writeRegister(PWR_MGMT_1, 1<<7);
  Serial.println("System reset...");
}








void MPU6050::rawAccelData(int16_t* accelData){
  uint8_t rawData[6];
  readRegisters(MPU6050_ACCEL_XOUT_H_REG, 6, rawData);
  accelData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  accelData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  accelData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}


void MPU6050::rawGyroData(int16_t* gyroData){
  uint8_t rawData[6];
  
  readRegisters(MPU6050_GYRO_XOUT_H_REG, 6, rawData);
  gyroData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
  gyroData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
  gyroData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}
