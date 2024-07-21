#include <cstdint>
#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "Arduino.h"
#include "esp32-hal-cpu.h"
#include <esp_sleep.h>

void printClck();
void updateClock(uint8_t freq);
void sleepTime(uint8_t time);
void deepSleep();

#endif
