#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define HANDSHAKE_LEN 1000

#define LED_PIN 6
#define ISR_PERIOD_US 100

#define BIT_LENGTH 210

// Make sure this is the same as in receiver.ino or nothing works!!
#define DATA_START_MAGIC_NUMBER 0x0F

bool is_handshaking;
int cycles_remaining, handshake_bit = 0;

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO CREATE BLINKS
////////////////////////////////////////////////////////////////////////////////
void TimerHandler() {
  if (!cycles_remaining) {
    if (is_handshaking) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    } else {

    }
    cycles_remaining = BIT_LENGTH;
  }
  cycles_remaining--;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Timer1.initialize();
  Timer1.setPeriod(ISR_PERIOD_US);
  Timer1.attachInterrupt(TimerHandler);
}


void loop() {
  is_handshaking = true;
  delay(10000);
//  is_handshaking = false;;
//  delay(10000);
}
