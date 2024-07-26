#include <Wire.h>


#define CTRL1_REG 0x20
#define CTRL2_REG 0x21
#define CTRL3_REG 0x22
#define CTRL4_REG 0x23
#define CTRL5_REG 0x24
#define CTRL6_REG 0x25
#define CTRL7_REG 0x26

#define MED_MAG_DECLINATION -7.34       //degrees
#define MED_MAG_INCLINATION 29.28       //degrees

#define RIONEGRO_MAG_DECLINATION -7.4   //degrees
#define RIONEGRO_MAG_INCLINATION 29.17  //degrees



const int LSM303D_ADDR = 0x1D; // Dirección I2C del LSM303D
const float ACCEL_SCALE = 0.061 / 1000.0; // 0.061 mg/LSB convertido a g/LSB
const float MAG_SCALE = 0.080 / 1000.0; // 0.061 mgauss/LSB convertido a gauss/LSB

int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

int16_t magX, magY, magZ;
float gaussX, gaussY, gaussZ;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  // Configurar el LSM303D
  writeRegister(LSM303D_ADDR,CTRL1_REG, 0b10000111);    //400Hz accel, enable XYZ
  writeRegister(LSM303D_ADDR, CTRL2_REG, 0b00000000);      //Set ±2g full scale ACCEL
  writeRegister(LSM303D_ADDR, CTRL5_REG, 0b10010100);   //enable TEMP SENSOR, mag LOW RES, 100Hz mag
  writeRegister(LSM303D_ADDR, CTRL6_REG, 0b00000000);    // set ±2 gauss full scale MAG
  writeRegister(LSM303D_ADDR, CTRL7_REG, 0b00000000);     //CONTINOUS MAG

}

void loop() {
  //getAccelData();
  getMagData();
  delay(250);
}




//----------------------------------------------------------------



void writeRegister(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegister(byte address, byte reg, int numBytes, byte *buffer) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, numBytes);
  for (int i = 0; i < numBytes; i++) {
    buffer[i] = Wire.read();
  }
}

void getAccelData() {
  byte buffer[6];
  readRegister(LSM303D_ADDR, 0x28 | 0x80, 6, buffer); // Leer 6 bytes de los registros de aceleración (auto increment)

  accelX = (int16_t)(buffer[1] << 8 | buffer[0]); // Leer los 2 bytes de datos de aceleración en X
  accelY = (int16_t)(buffer[3] << 8 | buffer[2]); // Leer los 2 bytes de datos de aceleración en Y
  accelZ = (int16_t)(buffer[5] << 8 | buffer[4]); // Leer los 2 bytes de datos de aceleración en Z

  gForceX = accelX * ACCEL_SCALE; // Convertir a fuerza G
  gForceY = accelY * ACCEL_SCALE;
  gForceZ = accelZ * ACCEL_SCALE;

  Serial.print("X: ");
  Serial.print(gForceX);
  Serial.print(" g, Y: ");
  Serial.print(gForceY);
  Serial.print(" g, Z: ");
  Serial.println(gForceZ);
}


void getMagData() {
  byte buffer[6];
  readRegister(LSM303D_ADDR, 0x08 | 0x80, 6, buffer); // Leer 6 bytes de los registros de aceleración (auto increment)

  magX = (int16_t)(buffer[1] << 8 | buffer[0]); // Leer los 2 bytes de datos de aceleración en X
  magY = (int16_t)(buffer[3] << 8 | buffer[2]); // Leer los 2 bytes de datos de aceleración en Y
  magZ = (int16_t)(buffer[5] << 8 | buffer[4]); // Leer los 2 bytes de datos de aceleración en Z

  gaussX = magX * MAG_SCALE; // Convertir a fuerza G
  gaussY = magY * MAG_SCALE;
  gaussZ = magZ * MAG_SCALE;


  float angleRad = atan2(gaussY,gaussX);
  float angleDeg = angleRad *(180/PI) ;
  float geografico = angleDeg + RIONEGRO_MAG_DECLINATION;
  if (geografico < 0){
    geografico = geografico + 360;
  }

  Serial.print("X: ");
  Serial.print(gaussX);
  Serial.print(" gauss, Y: ");
  Serial.print(gaussY);
  Serial.print(" gauss, Z: ");
  Serial.print(gaussZ);
  Serial.println(" gauss");
  Serial.print("Azimut: ");
  Serial.print(angleDeg);
  Serial.println(" grados");
  Serial.print("Geografico: ");
  Serial.print(geografico);
  Serial.println(" grados");

}

