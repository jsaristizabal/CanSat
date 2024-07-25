#include <Wire.h>
#include "GY89_driver.h"


GY89 imu;

float accelData[3],gyroData[3],magData[4];
const float* anglesData;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  delay(250);
}

void loop() {
  imu.getAccel(accelData);
  imu.getMag(magData);
  anglesData = imu.getAngles(accelData);


  Serial.println();
  Serial.print("aX: ");
  Serial.print(accelData[0]);
  Serial.print("(g)");
  Serial.print("\t aY: ");
  Serial.print(accelData[1]);
  Serial.print("(g)");
  Serial.print("\t aZ: ");
  Serial.print(accelData[2]);
  Serial.println("(g)");
  // Serial.print("Geografico: ");
  // Serial.print(magData[3]);

  delay(250);
  

}
