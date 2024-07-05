//#include <Arduino.h>
#include "dht_driver.h"
#include "imu.h"


#define DHTPIN 2
#define DHTTYPE DHT11

#define I2C_SCL 9  // MPU6050 SCL
#define I2C_SDA 8  // MPU6050 SDA

MPU6050 imu;
DHTsensor dht(DHTPIN, DHTTYPE);

float calibArray[3];


void beginSensors();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  beginSensors();
}

void loop() {
  //dht.printData();
  // const float* accelData = imu.readAccelData();
  // const float* gyroData = imu.readGyroData();
  // float tempData = imu.readTempData();


  imu.printAllData();
  Serial.println();

  delay(200);
}

// put function definitions here:
void beginSensors(){
  dht.begin();
  dht.sensorDetails();

  imu.begin();
  imu.setAccelRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  
  delay(200);
  //imu.calibration(calibArray);
}