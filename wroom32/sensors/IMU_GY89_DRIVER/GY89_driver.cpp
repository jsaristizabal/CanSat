#include <Arduino.h>
#include <stdint.h>
#include "GY89_driver.h"
#include <Wire.h>


GY89::GY89(){}

void GY89::begin(){
  setAccelRange(LSM303D_RANGE_2_G);//set the scale of the accel, 2g,4g,6g,8g,16g
  setMagRange(LSM303D_RANGE_2_GAUSS);//set the scale of the magnetometer, 2gauss,4gauss,6g,12gauss
  setGyroRange(L3GD20H_RANGE_245_DEG);//set scale of the gyro 245 dps, 500dps, 2000dps
}

void GY89::setAccelRange(LSM303D_accel_range_t range){
  
  uint8_t regValue = readRegister(LSM303D_ADDRESS, CTRL2_REG); //read register value
  regValue &= ~ACCEL_SCALE_MSK; //clear register bits

  regValue |= range;          //set scale

  writeRegister(LSM303D_ADDRESS, CTRL2_REG, regValue); //write the scale value

  /*
  read the datasheet if you want to verify the conversion values from raw data to human-legible data
  */
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
    /*
  read the datasheet if you want to verify the conversion values from raw data to human-legible data
  */
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
  /*
  read the datasheet if you want to verify the conversion values from raw data to human-legible data
  */
    switch (range){
    case L3GD20H_RANGE_245_DEG:
      gyroLSB = 8.75 / 1000; // mdps/digit
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



/*
Returns an array with the values of the sensor
*/
const float* GY89::getAccel(){
  int16_t rawData[3];
  rawAccelData(rawData);
  accelData[0] = rawData[0] * accelLSB;
  accelData[1] = rawData[1] * accelLSB;
  accelData[2] = rawData[2] * accelLSB;

  return accelData;
}


/*
Returns an array with the values of the sensor
*/
const float* GY89::getMag(){
  int16_t rawData[4];
  rawMagData(rawData);
  
  magData[0] = rawData[0] * magLSB;
  magData[1] = rawData[1] * magLSB;
  magData[2] = rawData[2] * magLSB;
  magData[3] = (atan2(magData[1],magData[0]) * (180/PI)) + MED_MAG_DECLINATION;   //direction of the geographic north pole
  
  if (magData[3] > 360){
    magData[3] = magData[3] + 360;
  }

  return magData;
}

/*
Returns an array with the values of the sensor
*/
const float* GY89::getGyro(){
  int16_t rawData[3];
  rawGyroData(rawData);
  gyroData[0] = (rawData[0] * gyroLSB) - rateCalibrationArray[0];
  gyroData[1] = (rawData[1] * gyroLSB) - rateCalibrationArray[1];
  gyroData[2] = (rawData[2] * gyroLSB) - rateCalibrationArray[2];
  // Serial.print(gyroData[0]);
  // Serial.print("\t");
  // Serial.print(gyroData[1]);
  // Serial.print("\t");
  // Serial.println(gyroData[2]);

  return gyroData;
}
/*
Returns an array with the values of the orientation using the integral of the gyroscope
*/
const float* GY89::getGyroIntegral(){

  unsigned long currentMillis = millis();
  unsigned long previousMilis = 0;
  const long interval = 100;
  float yaw = 0;
  float pitch = 0;
  float roll = 0;

  const float* gyrData;
  gyrData = getGyro();
  if ((currentMillis - previousMilis) >= interval){
    previousMilis = currentMillis;
    
    gyrData = getGyro();
    float dt = interval / 1000.0;
    anglesIntegral[0] += gyrData[0] * dt;//integral of x gyro rate
    anglesIntegral[1] += gyrData[1] * dt;//integral of y gyro rate
    anglesIntegral[2] += gyrData[2] * dt;//integral of z gyro rate




    if (anglesIntegral[0] >= 360.0) {
      anglesIntegral[0] -= 360.0;
    }
    else if (anglesIntegral[0] < 0.0) {
      anglesIntegral[0] += 360.0;
    }
    
    if (anglesIntegral[1] >= 360.0) {
      anglesIntegral[1] -= 360.0;
    }
    else if (anglesIntegral[1] < 0.0) {
      anglesIntegral[1] += 360.0;
    }


    if (anglesIntegral[2] >= 360.0) {
      anglesIntegral[2] -= 360.0;
    }
    else if (anglesIntegral[2] < 0.0) {
      anglesIntegral[2] += 360.0;
    }


  }//endif

  return anglesIntegral;
}


/*
print the values of the sensor
*/
void GY89::printAccel(){
  const float* accData = getAccel();
  Serial.print("aX: ");
  Serial.print(accData[0]);
  Serial.print("(g)");
  Serial.print("\t aY: ");
  Serial.print(accData[1]);
  Serial.print("(g)");
  Serial.print("\t aZ: ");
  Serial.print(accData[2]);
  Serial.println("(g)");
}

/*
print the values of the sensor
*/
void GY89::printGyro(){
  const float* gyrData = getGyro();
  Serial.print("gX: ");
  Serial.print(gyrData[0]);
  Serial.print("(deg/s)");
  Serial.print("\t gY: ");
  Serial.print(gyrData[1]);
  Serial.print("(deg/s)");
  Serial.print("\t gZ: ");
  Serial.print(gyrData[2]);
  Serial.println("(deg/s)");
}

/*
print the values of the sensor
*/
void GY89::printMag(){
  const float* magnData = getMag();
  Serial.print("mX: ");
  Serial.print(magnData[0]);
  Serial.print("(gauss)");
  Serial.print("\t mY: ");
  Serial.print(magnData[1]);
  Serial.print("(gauss)");
  Serial.print("\t mZ: ");
  Serial.print(magnData[2]);
  Serial.println("(gauss)");
  Serial.print("North:  ");
  Serial.print(magnData[3]);
}



/*
calculate the inclination using the accelerometer sensor
it is neccesary to input the array that contains the [x,y,z] accel values
*/
const float* GY89::calculateAngles(const float* accelData){

  anglesData[0]   = atan(accelData[1] / sqrt(accelData[0]*accelData[0] + accelData[2]*accelData[2])) * 1/(3.142/180);   //angleRoll
  anglesData[1]   = atan(-accelData[0] / sqrt(accelData[1]*accelData[1] + accelData[2]*accelData[2])) * 1/(3.142/180);  //anglePitch
  anglesData[2]   = atan(sqrt(accelData[0]*accelData[0] + accelData[1]*accelData[1]) / accelData[2] ) * 1/(3.142/180);  //angleYaw
  
  return anglesData;
}


/*
static calibration of the gyroscope
*/
void GY89:calibrationStatic(){
  const int numReadings = 1000;

  Serial.println("Static calibration started, please do not move the sensor.");

  for(int i = 0; i < numReadings; i++){
    
    const float* gyroData = getGyro();     //read data
    //acummulate data for calibration
    // [°/s]
    rateCalibrationArray[0]  += gyroData[0];
    rateCalibrationArray[1]  += gyroData[1];
    rateCalibrationArray[2]  += gyroData[2];
    delay(1);
  }

  rateCalibrationArray[0] /= numReadings;
  rateCalibrationArray[1] /= numReadings;
  rateCalibrationArray[2] /= numReadings;

  Serial.println("Calibration finished!");



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




void GY89::rawAccelData(int16_t* accelData){
  uint8_t rawData[6];
  readRegisters(LSM303D_ADDRESS, ACCEL_XOUT_REG |0x80, 6, rawData);// 0x80 mask indicates data lecture
  accelData[0] = (int16_t)(rawData[1] << 8) | rawData[0];
  accelData[1] = (int16_t)(rawData[3] << 8) | rawData[2];
  accelData[2] = (int16_t)(rawData[5] << 8) | rawData[4];
}


void GY89::rawGyroData(int16_t* gyroData){

  uint8_t rawData[6];
  readRegisters(L3GD20H_ADDR, GYRO_XOUT_REG | 0x80, 6, rawData);// 0x80 mask indicates data lecture
  gyroData[0] = (int16_t)(rawData[1] << 8) | rawData[0];//roll
  gyroData[1] = (int16_t)(rawData[3] << 8) | rawData[2];//pitch
  gyroData[2] = (int16_t)(rawData[5] << 8) | rawData[4];//yaw
}


void GY89::rawMagData(int16_t* magData){
  uint8_t rawData[6];
  readRegisters(LSM303D_ADDRESS, MAG_XOUT_REG |0x80, 6, rawData);// 0x80 mask indicates data lecture
  magData[0] = (int16_t)(rawData[1] << 8) | rawData[0];     //mX
  magData[1] = (int16_t)(rawData[3] << 8) | rawData[2];     //mY
  magData[2] = (int16_t)(rawData[5] << 8) | rawData[4];     //mZ
}