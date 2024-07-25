#include <Wire.h>

const int L3GD20H_ADDR = 0x6B; // Dirección I2C del L3GD20H
const float GYRO_SCALE = 0.00875; // 245 dps (0.00875 dps/LSB)

int16_t gyroX, gyroY, gyroZ;
float dpsX, dpsY, dpsZ;

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
  Serial.begin(115200);
  
  // Configurar el L3GD20H
  writeRegister(L3GD20H_ADDR, 0x20, 0x0F); // CTRL_REG1: Normal power mode, all axes enabled, 95 Hz ODR
  writeRegister(L3GD20H_ADDR, 0x23, 0x00); // CTRL_REG4: 245 dps full scale
}

void loop() {
  byte buffer[6];
  readRegister(L3GD20H_ADDR, 0x28 | 0x80, 6, buffer); // Leer 6 bytes de los registros del giroscopio (auto increment)

  gyroX = (int16_t)(buffer[1] << 8 | buffer[0]); // Leer los 2 bytes de datos del giroscopio en X
  gyroY = (int16_t)(buffer[3] << 8 | buffer[2]); // Leer los 2 bytes de datos del giroscopio en Y
  gyroZ = (int16_t)(buffer[5] << 8 | buffer[4]); // Leer los 2 bytes de datos del giroscopio en Z

  dpsX = gyroX;// * GYRO_SCALE; // Convertir a grados por segundo
  dpsY = gyroY;// * GYRO_SCALE;
  dpsZ = gyroZ;// * GYRO_SCALE;

  Serial.print("X: ");
  Serial.print(dpsX);
  Serial.print(" dps, Y: ");
  Serial.print(dpsY);
  Serial.print(" dps, Z: ");
  Serial.println(dpsZ);
  
  delay(100);
}
