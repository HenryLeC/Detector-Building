#include <Arduino.h>
#include <Adafruit_NAU7802.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NAU7802 nau;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!nau.begin())
  {
    Serial.println("Failed to find NAU7802");
  }
  else
  {
    Serial.println("Found NAU7802");
  }

  nau.setLDO(NAU7802_3V0);
  Serial.println("LDO voltage set to 3.0V");

  nau.setGain(NAU7802_GAIN_128);
  Serial.println("Gain set to 128x");

  nau.setRate(NAU7802_RATE_10SPS);
  Serial.println("Conversion rate set to 10 SPS");

  // Take 10 readings to flush out readings
  for (uint8_t i = 0; i < 10; i++)
  {
    while (!nau.available())
      delay(1);
    nau.read();
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font
  display.display();
}

void loop()
{
  while (!nau.available())
  {
    delay(1);
  }

  int32_t val = nau.read();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("Henry & Grace");
  display.setCursor(1, 9);
  display.write("Weight Detector");
  display.setCursor(1, 18);
  display.print(val);
  display.display();
}