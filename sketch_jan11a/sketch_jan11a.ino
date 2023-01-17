#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> 
#endif

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 100
#define PIN_POWER 5
#define PIN_LED 6
#define PIN_BTN 7


Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN_LED, NEO_GRB + NEO_KHZ800);




void setup() {
  Serial.begin(9600);
  pinMode(PIN_POWER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
  #endif
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.setBrightness(30);
}

void loop() { // Set all pixel colors to 'off'
  int buttonState = digitalRead(PIN_BTN);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    digitalWrite(PIN_POWER, HIGH);

    for(int i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(150, 150, 150));
    }
 // Set all pixel colors to 'off'
  delay(5);
  strip.show();
  delay(20000);
  strip.clear();
  digitalWrite(PIN_POWER, LOW);
  }
}
