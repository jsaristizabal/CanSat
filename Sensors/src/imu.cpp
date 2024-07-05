#include <cstdint>
#include "imu.h"
#include <Arduino.h>

MPU6050::MPU6050() {}

void MPU6050::begin(){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x0);
  Wire.endTransmission();
}

const float* MPU6050::readAccelData(){
  int16_t rawData[3];
  rawAccelData(rawData);
  accelData[0] = rawData[0] / 2048.0  +0.02;
  accelData[1] = rawData[1] / 2048.0  -0.01;
  accelData[2] = rawData[2] / 2048.0  -0.08;
  return accelData;
}

float* MPU6050::readGyroData(){
  int16_t rawData[3];
  rawGyroData(rawData);
  gyroData[0] = rawData[0] / 65.5;
  gyroData[1] = rawData[1] / 65.5;
  gyroData[2] = rawData[2] / 65.5;

  return gyroData;
}


float MPU6050::readTempData(){
  uint8_t rawData[2];
  readRegisters(MPU6050_TEMP_XOUT_H_REG, 2, rawData);
  int16_t rawTemp = ((int16_t)rawData[0] << 8) | rawData[1];
  tempData = (rawTemp/340.0) +36.53;
  return tempData;
}

float* MPU6050::adjustGyroData(){
  float* adjGyroData = readGyroData();

  adjGyroData[0] -= rateCalibrationArray[0];
  adjGyroData[1] -= rateCalibrationArray[1];
  adjGyroData[2] -= rateCalibrationArray[2];
  return adjGyroData;
}

const float* MPU6050::calculateAngle(const float* accelData){
  anglesData[0]   = atan(accelData[1] / sqrt(accelData[0]*accelData[0] + accelData[2]*accelData[2])) * 1/(3.142/180);   //angleRoll
  anglesData[1]   = atan(-accelData[0] / sqrt(accelData[1]*accelData[1] + accelData[2]*accelData[2])) * 1/(3.142/180);  //anglePitch
  anglesData[2]   = atan(sqrt(accelData[0]*accelData[0] + accelData[1]*accelData[1]) / accelData[2] ) * 1/(3.142/180);  //angleYaw
  return anglesData;
}

void MPU6050::calibration(){
  
  int rateCalibrationNumber;

  Serial.println("INITIAL Calibration values: ");
  Serial.print(rateCalibrationArray[0]);
  Serial.print(" , ");
  Serial.print(rateCalibrationArray[1]);
  Serial.print(" , ");
  Serial.print(rateCalibrationArray[2]);

  for(rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++){
    
    const float* gyroData = readGyroData();     //read data
    //acummulate data for calibration
    // [°/s]
    rateCalibrationArray[0]  += gyroData[0];
    rateCalibrationArray[1]  += gyroData[1];
    rateCalibrationArray[2]  += gyroData[2];
    delay(1);
  }

  rateCalibrationArray[0] /= 2000;
  rateCalibrationArray[1] /= 2000;
  rateCalibrationArray[2] /= 2000;
  
  Serial.println();
  Serial.println("FINAL Calibration values: ");
  Serial.print(rateCalibrationArray[0]);
  Serial.print(" , ");
  Serial.print(rateCalibrationArray[1]);
  Serial.print(" , ");
  Serial.print(rateCalibrationArray[2]);
  Serial.println();
}



void MPU6050::printAccelData(){
  const float* accelData = readAccelData();
  Serial.print("aX: ");
  Serial.print(accelData[0]);
  Serial.print("(g)");
  Serial.print(", aY: ");
  Serial.print(accelData[1]);
  Serial.print("(g)");
  Serial.print(", aZ: ");
  Serial.print(accelData[2]);
  Serial.print("(g)");
}

void MPU6050::printGyroData(){
  const float* gyroData = readGyroData();
  
  Serial.print("GyroX: ");
  Serial.print(gyroData[0]);
  Serial.print("(°/s) ");
  Serial.print(", GyroY: ");
  Serial.print(gyroData[1]);
  Serial.print("(°/s) ");
  Serial.print(", GyroZ: ");
  Serial.print(gyroData[2]);
  Serial.print("(°/s) ");
}

void MPU6050::printAdjustedGyroData(){
  float* adjGyroData = adjustGyroData();
  
  Serial.print("GyroX: ");
  Serial.print(adjGyroData[0]);
  Serial.print("(°/s) ");
  Serial.print(", GyroY: ");
  Serial.print(adjGyroData[1]);
  Serial.print("(°/s) ");
  Serial.print(", GyroZ: ");
  Serial.print(adjGyroData[2]);
  Serial.print("(°/s) ");
}

void MPU6050::printTempData(){
  float temp = readTempData();
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("°C");
}


void MPU6050::printAnglesData(const float* accelData){
  const float* anglesData = calculateAngle(accelData);
  Serial.print("Roll angle: ");
  Serial.print(anglesData[0]);
  Serial.print("°, Pitch angle: ");
  Serial.print(anglesData[1]);
  Serial.print("°, Yaw angle: ");
  Serial.print(anglesData[2]);
  Serial.print("°");
}

void MPU6050::printAllData(){
  const float* accelData = readAccelData();
  Serial.println();
  printAccelData();
  Serial.println();
  printAdjustedGyroData();
  Serial.println();
  printTempData();
  Serial.println();
  printAnglesData(accelData);
  Serial.println();
}







void MPU6050::setLowPassFilter(mpu6050_bandwidth_t lowPassFilter){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(CONFIG_REG);                                //config register = 0x1A
  Wire.write(lowPassFilter);
  Wire.endTransmission();
}

void MPU6050::setAccelRange(mpu6050_accel_range_t newRange){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL_CONFIG_REG);
  Wire.write(newRange);
  Wire.endTransmission();
}

void MPU6050::setGyroRange(mpu6050_gyro_range_t newRange){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_GYRO_CONFIG_REG);
  Wire.write(newRange);
  Wire.endTransmission();
}

void MPU6050::setInterrupts(void){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_INT_PIN_CFG_REG);
  Wire.write(1<<7);   // logic level is HIGH
  Wire.write(1<<6);   // Open drain
  Wire.write(1<<5);   // Open drain
  Wire.endTransmission(false);
  Wire.write(MPU6050_INT_PIN_CFG_REG);
  Wire.write(0x02);
  Wire.endTransmission();


  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_INT_ENABLE_REG);
  Wire.write(1<<1);
  Wire.endTransmission();
}


void MPU6050::reset(void){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1);
  Wire.write(1<<7);
  Wire.endTransmission();
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


void MPU6050::writeRegister(uint8_t reg,uint8_t data){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
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