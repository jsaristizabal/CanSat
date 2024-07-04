#ifndef dht_driver_h
#define dht_driver_h

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>



class DHTsensor{
public:
    DHTsensor(int pin, int type);
    void begin();
    float getTemp();
    float getHum();
    void sensorDetails();
    void printData();

private:
    DHT_Unified dht;
    uint32_t delayMS;
};

#endif