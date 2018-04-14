#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LED_PIN 6
#define ISR_PERIOD_US 100

int pulse_index, cycles_remaining;
int pattern[] = {200, 200};
int pattern_length = sizeof(pattern) / sizeof(pattern[0]);

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO CREATE BLINKS
////////////////////////////////////////////////////////////////////////////////
void MaybeToggleTransmissionLED() {
  if (!cycles_remaining) {
    // Using a trick to toggle -- writing a 1 into the PIN register toggles the
    // output for atmel chips.  This could be done with more traditional Arduino
    // calls, but it's faster and exactly what we want to do it this way.
    PIND = PIND | 0b01000000;
    pulse_index = (pulse_index + 1) % pattern_length;
    cycles_remaining = pattern[pulse_index];
  }
  cycles_remaining--;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pulse_index = 0;
  cycles_remaining = pattern[pulse_index];

  Timer1.initialize();
  Timer1.setPeriod(ISR_PERIOD_US);
  Timer1.attachInterrupt(MaybeToggleTransmissionLED);
}


void loop() {
}
