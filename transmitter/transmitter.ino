#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define HANDSHAKE_LEN 1000

#define LED_PIN 6
#define ISR_PERIOD_US 100

#define BIT_LENGTH 210

#define HANDSHAKE_LENGTH 200

#define IDLE 0
#define HANDSHAKE 1
#define MAGIC_NUM 2
#define LENGTH 3
#define DATA 4

// Make sure this is the same as in receiver.ino or nothing works!!
#define DATA_START_MAGIC_NUMBER 0x0F

int transmission_stage;
int cycles_remaining, handshake_count, magic_bit;;
volatile bool transmission_complete;

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO CREATE BLINKS
////////////////////////////////////////////////////////////////////////////////
void TimerHandler() {
  if (!cycles_remaining) {
    switch(transmission_stage) {
      case HANDSHAKE:
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        handshake_count--;
        if (handshake_count <= 0) {
          transmission_stage = MAGIC_NUM;
        }
        break;
      case MAGIC_NUM:
        digitalWrite(LED_PIN, (DATA_START_MAGIC_NUMBER >> (7 - magic_bit)) & 0x01);
        magic_bit++;
        if (magic_bit >= 8) {
          transmission_stage = LENGTH;
        }
        break;

      default:
        Timer1.detachInterrupt();
        transmission_complete = true;
        break;
    };
    cycles_remaining = BIT_LENGTH;
  }
  cycles_remaining--;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Timer1.initialize();
}

void Send() {
  transmission_stage = HANDSHAKE;
  handshake_count = HANDSHAKE_LENGTH;
  magic_bit = 0;
  transmission_complete = false;

  Timer1.setPeriod(ISR_PERIOD_US);
  Timer1.attachInterrupt(TimerHandler);

  // Wait until the whole message was sent
  while (!transmission_complete) {
    delay(100);
  }
}

void loop() {
  Send();
  delay(3000);
//  is_handshaking = false;;
//  delay(10000);
}
