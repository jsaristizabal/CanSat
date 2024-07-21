#include "bmp_driver.h"
#include <Arduino.h>


// Constructor to initialize the BMP sensor
BMPsensor::BMPsensor() : bmp() {}

bool BMPsensor::begin(uint8_t mode, TwoWire *wire) {
    return bmp.begin(mode, wire);
}

// Method to get the temperature reading from the BMP sensor
float BMPsensor::readTemperature() {
    return bmp.readTemperature();
}

// Method to get the pressure reading from the BMP sensor
int32_t BMPsensor::readPressure() {
    return bmp.readPressure();
}

float BMPsensor::readAltitude( float seaLevelPressure){
    return bmp.readAltitude();
}

// int32_t readSeaLevelPressure(float seaLevelPressure){
//     return bmp.readSealevelPressure();
// }

float BMPsensor::getRealAltitude(float seaLevelPressure){
    return bmp.readAltitude(seaLevelPressure);
}

