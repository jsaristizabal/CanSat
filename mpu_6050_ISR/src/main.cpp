#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_driver.h"

MPU6050 imu;

//Only RTC IO can be used as a source for external wake
//source. They are pins: 0,2,4,12-15,25-27,32-39.

RTC_DATA_ATTR int bootCount = 0;    //This counter persist in sleep mode



void setup(){
  initSystem();
}


void loop(){
  Serial.println("This will never be printed");
}







void initSystem(){
  Serial.begin(115200);
  Wire.begin();
  delay(500); //Take some time to open up the Serial Monitor

  
  Serial.println("AWAKE!!!");
  ++bootCount;//Increment boot number and print it every reboot
  Serial.println("Boot number: " + String(bootCount));
  //print_wakeup_reason();
  imu.begin(MPU6050_RANGE_SLEEP,MPU6050_RANGE_500_DEG);
  //imu.setAccelRange(MPU6050_RANGE_SLEEP);
  //imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setInterrupts();
  //imu.setupMotion();
  sleepNow();

}


void sleepNow(){
  Serial.println("Sleeping");
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,0);//1 = RISING_EDGE, 0 = FALLING_EDGE
  delay(1500);
  esp_deep_sleep_start(); //Start sleep until interruption
  Serial.println("This will never be printed");
}





/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}