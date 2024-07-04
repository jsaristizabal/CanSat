#include <Arduino.h>
#include "dht_driver.h"

#define DHTPIN 2
#define DHTTYPE DHT11

DHTsensor dht(DHTPIN, DHTTYPE);


void beginSensors();

void setup() {
  Serial.begin(115200);
  beginSensors();
}

void loop() {
  dht.printData();
}

// put function definitions here:
void beginSensors(){
  dht.begin();
  dht.sensorDetails();
}