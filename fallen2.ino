#include "FastLED.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

Adafruit_CAP1188 cap = Adafruit_CAP1188();

// How many leds in your strip?
#define NUM_LEDS 4

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define UI_LEDS_DATA_PIN 13
#define STRIP_LEDS_DATA_PIN 12

volatile boolean interrupt = false;
volatile boolean pressingButton = false;

// Define the array of leds
CRGB ui_leds[NUM_LEDS];
CRGB strip_leds[NUM_LEDS];

// set up global state management for ui lights
int UI_POWER = 0;
int UI_PLUS = 1;
int UI_MINUS = 2;
int UI_SHUFFLE = 3;
int UI_BUTTON_COUNT = 4;

int UI_BRIGHTNESS[] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600);

  // Initialize the sensor, if using i2c you can pass in the i2c address
  if (!cap.begin()) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");
  pinMode(3, INPUT);
  // sensitivity
  cap.writeRegister(0x1F, 0x6F);  // 8x  sensitivity
  // Turn off multitouch so only one button pressed at a time
  cap.writeRegister(0x2A, 0x80);  // 0x2A default 0x80 use 0x41  — Set multiple touches back to off
//  cap.writeRegister(0x41, 0x39);  // 0x41 default 0x39 use 0x41  — Set "speed up" setting back to off
//  cap.writeRegister(0x72, 0x00);  // 0x72 default 0x00  — Sets LED links back to off (default)
//  cap.writeRegister(0x44, 0x41);  // 0x44 default 0x40 use 0x41  — Set interrupt on press but not release
//  cap.writeRegister(0x28, 0x00);  // 0x28 default 0xFF use 0x00  — Turn off interrupt repeat on button hold
  EIFR = 1; // clear flag for interrupt 1
  attachInterrupt(1, routine_Interrupt_CAP1188, FALLING);

  FastLED.addLeds<NEOPIXEL, UI_LEDS_DATA_PIN>(ui_leds, NUM_LEDS);

  // Force LEDs off
  fill_solid(ui_leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  powerUpSequence();
}

void loop() {
  // Serial.println(digitalRead(3));
  uint8_t touched = cap.touched();

  if (interrupt == true) {
  for (uint8_t i=0; i<8; i++) {
    if (touched & (1 << i)) {
      Serial.print("C"); Serial.print(i+1); Serial.print("\t");
    }
  }
    interrupt = false;
  }
}

void routine_Interrupt_CAP1188()  {
  interrupt = true;
}

void powerUpSequence() {
  static uint8_t currentBrightness = 0;
  static uint8_t maxBrightness = 255 * 0.3;
  int fadeAmount = 5;

  static const float duration = ((float)(7.5) / 256)*1000;

  // set this to max bright - 255 = off, 0 = maximum bright
  while(currentBrightness < maxBrightness) {
    currentBrightness += 5;
//    fill_solid(ui_leds, NUM_LEDS, CRGB::White);
//    fadeLightBy(ui_leds, NUM_LEDS, currentBrightness);
//    FastLED.show();
//    delay(duration);

    for (int uiButton = 0; uiButton < UI_BUTTON_COUNT; uiButton++) {
      ui_leds[uiButton] = CRGB::White;
      UI_BRIGHTNESS[uiButton] = currentBrightness;
      ui_leds[uiButton].maximizeBrightness(UI_BRIGHTNESS[uiButton]);
    }

    FastLED.show();
    delay(duration);
  }
}
