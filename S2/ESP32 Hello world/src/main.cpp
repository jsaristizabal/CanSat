#include <Arduino.h>
#include <NeoPixelBus.h>

const uint16_t PixelCount = 1;  // Number of leds
const uint8_t PixelPin = 18;    // RGB LED Pin
uint8_t PixelBrightness = 50;   // Pixel brightness
uint8_t timeDelay = 500;        // Delay time

// Type of the NeoPixel
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

// Prototipo de la función
void setPixelColor(uint8_t red, uint8_t green, uint8_t blue);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Iniciando NeoPixel");

  strip.Begin();
  strip.Show();  // Inicializa el LED apagado

  Serial.println("NeoPixel iniciado");
}

void loop() {
  // Ciclo para cambiar de color cada segundo con intensidad reducida
  Serial.println("Cambiando a Rojo con menor intensidad");
  setPixelColor(4, 0, 0);  // Rojo con menor intensidad
  delay(timeDelay);

  Serial.println("Cambiando a Verde con menor intensidad");
  setPixelColor(0, 4, 0);  // Verde con menor intensidad
  delay(timeDelay);

  Serial.println("Cambiando a Azul con menor intensidad");
  setPixelColor(0, 0, 4);  // Azul con menor intensidad
  delay(timeDelay);

  Serial.println("Cambiando a Blanco con menor intensidad");
  setPixelColor(4, 4, 4);  // Blanco con menor intensidad
  delay(timeDelay);

  Serial.println("Apagando LED");
  setPixelColor(0, 0, 0);  // Apagado
  delay(timeDelay);
}

void setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
  RgbColor color(red, green, blue);
  strip.SetPixelColor(0, color);
  strip.Show();

  Serial.print("Color configurado a: ");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.println(blue);
}
