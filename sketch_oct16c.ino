#include <Adafruit_CircuitPlayground.h>


void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();


}

void loop() {

  // Wait for button press to start timer
  if (CircuitPlayground.leftButton() || CircuitPlayground.rightButton())  {

    // Calculate DT
    unsigned long DT = 10000 / 10 * 1.5;

    // Turn on the NeoPixels and play sound one at a time, waiting DT each time
    for (int p = 0; p < 10; p++) {
      delay(DT);
      CircuitPlayground.setPixelColor(p, 255, 255, 255);
      CircuitPlayground.playTone(500, 100);
      if (p == 9) {
        CircuitPlayground.clearPixels();
      }

    }
  }

}
