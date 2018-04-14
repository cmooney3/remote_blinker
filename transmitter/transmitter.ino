#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LED_PIN 5
#define BLINK_PERIOD_US 100000

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO CREATE BLINKS
////////////////////////////////////////////////////////////////////////////////
void ToggleTransmissionLED() {
  // Using a trick to toggle -- writing a 1 into the PIN register toggles the
  // output for atmel chips.  This could be done with more traditional Arduino
  // calls, but it's faster and exactly what we want to do it this way.
  PINC = PINC | 0b01000000;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Timer1.initialize();
  Timer1.setPeriod(BLINK_PERIOD_US);
  Timer1.attachInterrupt(ToggleTransmissionLED); 
}

void loop() {
}
