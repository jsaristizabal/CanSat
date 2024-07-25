//#include <Arduino.h>
#include "dht_driver.h"
#include "bmp_driver.h"
#include "imu.h"


#define DHTPIN 2
#define DHTTYPE DHT11

#define I2C_SCL 9  // MPU6050 SCL
#define I2C_SDA 8  // MPU6050 SDA

MPU6050 imu;
BMPsensor bmp;
DHTsensor dht(DHTPIN, DHTTYPE);

float calibArray[3];


void beginSensors();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  beginSensors();
  delay(100);
}

void loop() {
  //dht.printData();
  // const float* accelData = imu.readAccelData();
  // float tempData = imu.readTempData();


  Serial.println(bmp.readPressure());
  imu.printAllData();
  

  delay(200);
}

// put function definitions here:
void beginSensors(){
  dht.begin();
  dht.sensorDetails();
  
  imu.begin();
  imu.setAccelRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  bmp.begin();
  
  
  delay(100);
  imu.calibration();
}