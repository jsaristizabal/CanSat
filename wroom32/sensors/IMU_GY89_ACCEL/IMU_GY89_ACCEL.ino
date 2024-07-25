#include <Wire.h>

const int LSM303D_ADDR = 0x1D; // Dirección I2C del LSM303D
const float ACCEL_SCALE = 0.061 / 1000.0; // 0.061 mg/LSB convertido a g/LSB

int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

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

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // Configurar el LSM303D
  writeRegister(LSM303D_ADDR, 0x20, 0x57); // CTRL_REG1_A: 100Hz, enable XYZ
  writeRegister(LSM303D_ADDR, 0x21, 0x00); // CTRL_REG2_A: Set ±2g full scale
}

void loop() {
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
  
  delay(250);
}
