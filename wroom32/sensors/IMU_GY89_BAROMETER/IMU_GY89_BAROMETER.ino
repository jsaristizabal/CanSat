#include <Wire.h>

#define BMP085_ADDRESS 0x77

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
int32_t b5;

float readAltitude(float seaLevelPressure = 101325) {
  float pressure = readPressure();
  float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
  return altitude;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  readCalibrationData();
  if (!calibrationDataIsValid()) {
    Serial.println("Failed to read calibration data");
    while (1) {}
  }
}

void loop() {
  Serial.print("Temperature: ");
  Serial.print(readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure: ");
  Serial.print(readPressure());
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(readAltitude());
  Serial.println(" m");

  delay(500);
}

uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 1);
  return Wire.read();
}

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);
  uint16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

void readCalibrationData() {
  ac1 = read16(0xAA);
  ac2 = read16(0xAC);
  ac3 = read16(0xAE);
  ac4 = read16(0xB0);
  ac5 = read16(0xB2);
  ac6 = read16(0xB4);
  b1 = read16(0xB6);
  b2 = read16(0xB8);
  mb = read16(0xBA);
  mc = read16(0xBC);
  md = read16(0xBE);
}

bool calibrationDataIsValid() {
  // Aquí puedes agregar validaciones para los datos de calibración si es necesario
  return true;
}

int32_t readRawTemperature() {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  return read16(0xF6);
}

int32_t readRawPressure() {
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34);
  Wire.endTransmission();
  delay(8);
  int32_t value = read16(0xF6);
  value = (value << 8) | read8(0xF8);
  value >>= (8 - 3);  // Shift right according to the oversampling setting
  return value;
}

int32_t computeB5(int32_t ut) {
  int32_t X1 = (ut - ac6) * ac5 >> 15;
  int32_t X2 = (mc << 11) / (X1 + md);
  return X1 + X2;
}

float readTemperature() {
  int32_t ut = readRawTemperature();
  b5 = computeB5(ut);
  float temp = (b5 + 8) >> 4;
  temp /= 10;
  return temp;
}

int32_t readPressure() {
  int32_t up = readRawPressure();
  int32_t B6 = b5 - 4000;
  int32_t X1 = (b2 * (B6 * B6 >> 12)) >> 11;
  int32_t X2 = ac2 * B6 >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = (((ac1 * 4 + X3) << 3) + 2) >> 2;
  X1 = ac3 * B6 >> 13;
  X2 = (b1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = (ac4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)up - B3) * (50000 >> 3);
  int32_t p = B7 < 0x80000000 ? (B7 * 2) / B4 : (B7 / B4) * 2;
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p += (X1 + X2 + 3791) >> 4;
  return p;
}


