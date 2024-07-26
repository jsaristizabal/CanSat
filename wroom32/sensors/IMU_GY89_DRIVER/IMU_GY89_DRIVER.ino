#include <Wire.h>
#include "GY89_driver.h"


GY89 imu;

//float accelData[3],gyroData[3],magData[4];
const float* angleData;
const float* accData;
const float* magnData;
const float* gyrData;

unsigned long previousMilis = 0;
const long interval = 100;
float yaw = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  delay(250);
}

void loop() {
  unsigned long currentMillis = millis();


  //accData = imu.getAccel();
  //magnData = imu.getMag();

  Serial.println("\n");
  //imu.printAccel();
  //imu.printGyro();
  //imu.printMag();

  
  
  angleData = imu.getGyroIntegral();


  Serial.println();
  Serial.print("Roll angle: ");
  Serial.print(angleData[0]);
  Serial.print("°, Pitch angle: ");
  Serial.print(angleData[1]);
  Serial.print("°, Yaw angle: ");
  Serial.print(angleData[2]);
  Serial.print("°");
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
  Serial.print("mX: ");
  Serial.print(magnData[0]);
  Serial.print("(gauss)");
  Serial.print("\t mY: ");
  Serial.print(magnData[1]);
  Serial.print("(gauss)");
  Serial.print("\t mZ: ");
  Serial.print(magnData[2]);
  Serial.println("(gauss)");

  Serial.print("Geografico: ");
  Serial.print(magnData[3]);
  */
  delay(250);
  

}
