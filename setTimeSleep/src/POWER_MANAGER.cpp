#include <cstdint>
#include "POWER_MANAGER.h"
#include <WiFi.h>


uint32_t clckFrq = 0;
#define uS_TO_S_FACTOR 1E6  /* Conversion factor for micro seconds to seconds */


void printClck() {
    clckFrq = ESP.getCpuFreqMHz();       //Get the MCU clock frequency
    Serial.print("MCU frequency: ");
    Serial.print(clckFrq);               //Print the actual frequency
    Serial.println("MHz");
}

void updateClock(uint8_t freq) {
    setCpuFrequencyMhz(freq);             //Update the clock frequency
    printClck();        //Get the clock frequency to verify
}


void sleepTime(uint8_t time) {
    Serial.println("Entering Light Sleep mode");
    delay(250);
    esp_sleep_enable_timer_wakeup(time * uS_TO_S_FACTOR); // Configura el temporizador de despertador (wake up timer) para que despierte al ESP32 después de t segundos
    esp_light_sleep_start(); // Coloca al ESP32 en modo de Light Sleep
    Serial.println("Awake from Light Sleep");
}

void deepSleep() {
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}
