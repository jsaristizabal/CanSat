#include <Arduino.h>
#include "dht_driver.h"

// Constructor to initialize the DHT sensor with the specified pin and type
DHTsensor::DHTsensor(int pin, int type) : dht(pin, type) {}

// Method to initialize the DHT sensor
void DHTsensor::begin() {
    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;  // Set delay between readings based on sensor details
}

// Method to get the temperature reading from the sensor
float DHTsensor::getTemp() {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
    }
    return event.temperature;
}

// Method to get the humidity reading from the sensor
float DHTsensor::getHum(){
    sensors_event_t event;
    dht.humidity().getEvent(&event);

    if (isnan(event.relative_humidity)){
        Serial.println(F("Error reading humidity!"));
    }
    return event.relative_humidity;
}

// Method to print detailed sensor information to the Serial Monitor
void DHTsensor::sensorDetails() {
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
}


// Method to print the temperature and humidity readings to the Serial Monitor
void DHTsensor::printData() {
    Serial.print(F("Temperature: "));
    Serial.print(getTemp());
    Serial.print(F("°C"));
    Serial.print(F(" Humidity: "));
    Serial.print(getHum());
    Serial.println(F("%"));
}

