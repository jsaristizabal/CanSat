#include <Wire.h>
#include "GY89_driver.h"


GY89 imu;

//float accelData[3],gyroData[3],magData[4];
const float* angleData;
const float* accData;
const float* magnData;
const float* gyrData;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  delay(250);
}

void loop() {
  accData = imu.getAccel();
  magnData = imu.getMag();
  //angleData = imu.calculateAngles(accData);
  imu.getGyro();
  /*
  Serial.println();
  Serial.print("Roll angle: ");
  Serial.print(angleData[0]);
  Serial.print("°, Pitch angle: ");
  Serial.print(angleData[1]);
  Serial.print("°, Yaw angle: ");
  Serial.print(angleData[2]);
  Serial.print("°");
  */

  /*
  Serial.println();
  Serial.print("aX: ");
  Serial.print(gyrData[0]);
  Serial.print("(g)");
  Serial.print("\t aY: ");
  Serial.print(gyrData[1]);
  Serial.print("(g)");
  Serial.print("\t aZ: ");
  Serial.print(gyrData[2]);
  Serial.println("(g)");
  */
  // Serial.print("Geografico: ");
  // Serial.print(magnData[3]);

  delay(250);
  

}
