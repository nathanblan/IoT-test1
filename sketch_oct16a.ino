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

    int seconds = (millis() - startTime) / 1000;
    CircuitPlayground.setPixelColor(seconds - 1, 255, 0, 0);
    if ((millis() - startTime) / seconds == 1000) {
      CircuitPlayground.playTone(500, 100);
    }

    if (millis() - startTime > 10000) {
      started = false;
    }
  }

  //Save the button state as previous button state
  leftButtonWasDown = leftButtonIsDown;
}
