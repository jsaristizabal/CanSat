#ifndef bmp_driver_h
#define bmp_driver_h

#include <Adafruit_BMP085.h>
#include <Wire.h>

class BMPsensor{
public:
    BMPsensor();
    bool begin(uint8_t mode = BMP085_ULTRAHIGHRES, TwoWire *wire = &Wire);
    float readTemperature(void);
    int32_t readPressure(void);
    float readAltitude(float seaLevelPressure);
    int32_t readSeaLevelPressure(float seaLevelPressure);
    void sensorDetails();
    float getRealAltitude(float seaLevelPressure);
private:
    Adafruit_BMP085 bmp;
    float seaLevelPressure;
};

#endif