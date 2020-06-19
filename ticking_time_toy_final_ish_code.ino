#include <Adafruit_CircuitPlayground.h>


void setup() {
  Serial.begin(9600);
  CircuitPlayground.begin();


}

void loop() {

  float x = 30000;

  // Turn on the NeoPixels one at a time, waiting DT each time
  if (CircuitPlayground.rightButton()) {
    for (int p = 0; p < 10; p++) {

      //calculate DT
      float DT = x / 10 ;
      delay(DT);
      CircuitPlayground.setPixelColor(p, 255, 0, 0);
      CircuitPlayground.playTone(1000, 100);
      //x gets smaller
      x = x * 0.8 ;

      if (p == 9) {
        p = 0;
        CircuitPlayground.clearPixels();
      }

      if (CircuitPlayground.leftButton()) {
        break;
      }
    }
  }
  CircuitPlayground.clearPixels();
}
