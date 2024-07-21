#include <Arduino.h>
#include <esp_sleep.h>

// Función para despertar del sueño ligero
void wakeupHandler() {
  // Código a ejecutar después de despertar
}

void setup() {
  Serial.begin(115200);

  // Configurar el pin de GPIO que se utilizará para despertar
  pinMode(GPIO_NUM_33, INPUT_PULLUP); // Cambia el número de GPIO según tus necesidades

  // Adjuntar el manejador de la interrupción para el pin
  attachInterrupt(GPIO_NUM_33, wakeupHandler, FALLING);

  // Configurar el temporizador para despertar después de 10 segundos
  esp_sleep_enable_timer_wakeup(10 * 1000000); // 10 segundos en microsegundos

  // Poner el ESP32-S2 en modo de sueño ligero
  esp_light_sleep_start();

  // El código continuará ejecutándose aquí después de despertar
  Serial.println("Woke up!");
}

void loop() {
  // Código del loop principal
}

