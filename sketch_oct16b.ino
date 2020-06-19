#include <Adafruit_CircuitPlayground.h>

bool leftButtonIsDown = false;
bool leftButtonWasDown = false;

unsigned long startTime = 0;
boolean started = false;

void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();
}

void loop() {
  leftButtonIsDown = CircuitPlayground.leftButton();

  if (leftButtonWasDown == false && leftButtonIsDown == true) {
    startTime = millis();
    started = true;
    CircuitPlayground.clearPixels();
    CircuitPlayground.playTone(500, 100);
  }

  if (started) {
    //make into a loop, animate pixels,
    float x = 1000.0;

    float seconds = (millis() - startTime) / 1000 * 1.2;
    if ((millis() - startTime) / seconds == x) {
      int pixelIndex = 0;
      CircuitPlayground.playTone(500, 100);
      CircuitPlayground.setPixelColor(pixelIndex, 255, 0, 0);
      pixelIndex++;
      x = x * 0.5;
    }

    // turn neopixel with index of (x original value / x / 2) red (Areeba's idea)


    if (millis() - startTime > 10000) {
      started = false;
    }
  }

  //Save the button state as previous button state
  leftButtonWasDown = leftButtonIsDown;
}
