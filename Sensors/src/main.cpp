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
  float accelData[3];
  float gyroData[3];
  float anglesData[3];
  float temp = 0;
  imu.readAccelData(accelData);
  imu.readGyroData(gyroData);
  imu.readTempData(&temp);

  gyroData[0] -= calibArray[0];
  gyroData[1] -= calibArray[1];
  gyroData[2] -= calibArray[2];

  imu.calculateAngle(anglesData, accelData);


  Serial.print("aX: ");
  Serial.print(accelData[0]);
  Serial.print("(g)");
  Serial.print(", aY: ");
  Serial.print(accelData[1]);
  Serial.print("(g)");
  Serial.print(", aZ: ");
  Serial.print(accelData[2]);
  Serial.print("(g)");
  
  Serial.print("    |   ");

  Serial.print("GyroX: ");
  Serial.print(gyroData[0]);
  Serial.print("(°/s) ");
  Serial.print(", GyroY: ");
  Serial.print(gyroData[1]);
  Serial.print("(°/s) ");
  Serial.print(", GyroZ: ");
  Serial.print(gyroData[2]);
  Serial.print("(°/s) ");
  
  Serial.print("    |   ");

  
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println("°C");

  Serial.print("Roll angle [°]=");
  Serial.print(anglesData[0]);
  Serial.print("Pitch angle [°]=");
  Serial.print(anglesData[1]);
  Serial.print("Yaw angle [°]=");
  Serial.println(anglesData[2]);
  //dht.printData();

  

  delay(150);
}

// put function definitions here:
void beginSensors(){
  dht.begin();
  dht.sensorDetails();

  imu.begin();
  imu.setAccelRange(MPU6050_RANGE_16_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  delay(200);
  imu.calibration(calibArray);
}