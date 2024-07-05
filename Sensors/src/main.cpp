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
  imu.calibration();
  delay(100);
  imu.printGyroData();
  
}

void loop() {
  //dht.printData();
  imu.printAllData();
  // const float* accelData = imu.readAccelData();
  // float tempData = imu.readTempData();


  //imu.printAllData();
  

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