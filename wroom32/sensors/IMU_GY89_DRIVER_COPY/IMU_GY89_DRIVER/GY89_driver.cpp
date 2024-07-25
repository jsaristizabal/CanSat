#include <Arduino.h>
#include <stdint.h>
#include "GY89_driver.h"
#include <Wire.h>


GY89::GY89(){}

void GY89::begin(){
  setAccelRange(LSM303D_RANGE_2_G);
  setMagRange(LSM303D_RANGE_2_GAUSS);
  setGyroRange(L3GD20H_RANGE_245_DEG);
}

void GY89::setAccelRange(LSM303D_accel_range_t range){
  
  uint8_t regValue = readRegister(LSM303D_ADDRESS, CTRL2_REG); //read register value
  regValue &= ~ACCEL_SCALE_MSK; //clear register bits
  Serial.println(range);
  Serial.println(regValue);
  regValue |= range;          //set scale
  Serial.println(regValue);
  writeRegister(LSM303D_ADDRESS, CTRL2_REG, regValue); //write the scale value
   switch (range) {
    case LSM303D_RANGE_2_G:
      accelLSB = 0.061/1000; //mg/LSB
      break;
    case LSM303D_RANGE_4_G:
      accelLSB = 0.122/1000; //mg/LSB
      break;
    case LSM303D_RANGE_6_G:
      accelLSB = 0.183/1000; //mg/LSB
      break;
    case LSM303D_RANGE_8_G:
      accelLSB = 0.244/1000; //mg/LSB
      break;
    case LSM303D_RANGE_16_G:
      accelLSB = 0.732/1000; //mg/LSB
      break;
    default:
      accelLSB = 0;
      break;
  }
}

void GY89::setMagRange(LSM303D_mag_range_t range){
  uint8_t regValue = readRegister(LSM303D_ADDRESS, CTRL6_REG); //read register value
  regValue &= ~MAG_SCALE_MSK;     //clear register bits
  regValue |= range;              //set scale
  writeRegister(LSM303D_ADDRESS, CTRL6_REG , regValue); //write the scale value
  switch (range){
    case LSM303D_RANGE_2_GAUSS:
      magLSB = 0.080/1000; //mg/LSB
      break;
    case LSM303D_RANGE_4_GAUSS:
      magLSB = 0.160/1000; //mg/LSB
      break;
    case LSM303D_RANGE_8_GAUSS:
      magLSB = 0.320/1000; //mg/LSB
      break;
    case LSM303D_RANGE_12_GAUSS:
      magLSB = 0.479/1000; //mg/LSB
      break;
    default:
      accelLSB = 0;
      break;
  }
}

void GY89::setGyroRange(L3GD20H_gyro_range_t range){
  uint8_t regValue = readRegister(L3GD20H_ADDR, CTRL4_REG); //read register value
  regValue &= ~GYRO_SCALE_MSK;     //clear register bits
  regValue |= range;              //set scale
  writeRegister(L3GD20H_ADDR, CTRL4_REG , regValue); //write the scale value

    switch (range){
    case L3GD20H_RANGE_245_DEG:
      gyroLSB = 875 / 1000; // mdps/digit
      break;
    case L3GD20H_RANGE_500_DEG:
      gyroLSB = 17.5 / 1000; // mdps/digit
      break;
    case L3GD20H_RANGE_2000_DEG:
      gyroLSB = 70.0 / 1000; // mdps/digit
      break;
    default:
      accelLSB = 0;
      break;
  }
}


void GY89::getAccel(float accelData[3]){
  
  uint8_t rawData[6];
  readRegisters(LSM303D_ADDRESS, ACCEL_XOUT_REG |0x80, 6, rawData);// 0x80 mask indicates data lecture
  accelData[0] = (int16_t)(rawData[1] << 8) | rawData[0];
  accelData[1] = (int16_t)(rawData[3] << 8) | rawData[2];
  accelData[2] = (int16_t)(rawData[5] << 8) | rawData[4];

  // accelData[0] = accelData[0] * accelLSB;
  // accelData[1] = accelData[1] * accelLSB;
  // accelData[2] = accelData[2] * accelLSB;
}

void GY89::getMag(float magData[4]){
  uint8_t rawData[6];
  readRegisters(LSM303D_ADDRESS, MAG_XOUT_REG |0x80, 6, rawData);// 0x80 mask indicates data lecture
  magData[0] = (int16_t)(rawData[1] << 8) | rawData[0];     //mX
  magData[1] = (int16_t)(rawData[3] << 8) | rawData[2];     //mY
  magData[2] = (int16_t)(rawData[5] << 8) | rawData[4];     //mZ

  magData[0] = magData[0] * magLSB;
  magData[1] = magData[1] * magLSB;
  magData[2] = magData[2] * magLSB;
  magData[3] = (atan2(magData[1],magData[0]) * (180/PI)) + MED_MAG_DECLINATION;   //direction of the geographic north pole
  if (magData[3] > 360){
    magData[3] = magData[3] + 360;
  }
  



}

void GY89::getGyro(float gyroData[3]){
  uint8_t rawData[6];
  readRegisters(LSM303D_ADDRESS, GYRO_XOUT_REG |0x80, 6, rawData);// 0x80 mask indicates data lecture
  gyroData[0] = (int16_t)(rawData[1] << 8) | rawData[0];
  gyroData[1] = (int16_t)(rawData[3] << 8) | rawData[2];
  gyroData[2] = (int16_t)(rawData[5] << 8) | rawData[4];

  gyroData[0] = gyroData[0] * gyroLSB;
  gyroData[1] = gyroData[1] * gyroLSB;
  gyroData[2] = gyroData[2] * gyroLSB;

}

const float* GY89::getAngles(const float accelData[3]){
  anglesData[0]   = atan(accelData[1] / sqrt(accelData[0]*accelData[0] + accelData[2]*accelData[2])) * 1/(3.142/180);   //angleRoll
  anglesData[1]   = atan(-accelData[0] / sqrt(accelData[1]*accelData[1] + accelData[2]*accelData[2])) * 1/(3.142/180);  //anglePitch
  anglesData[2]   = atan(sqrt(accelData[0]*accelData[0] + accelData[1]*accelData[1]) / accelData[2] ) * 1/(3.142/180);  //angleYaw
  return anglesData;
}







/*===============================================PRIVATE FUNCTIONS==========================================*/
void GY89::writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t GY89::readRegister(uint8_t deviceAddress, uint8_t reg) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, (uint8_t)1);
    return Wire.read();
}

void GY89::readRegisters(uint8_t deviceAddress, uint8_t reg, uint8_t length, uint8_t *data) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, length);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
}