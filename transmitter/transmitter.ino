#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LED_PIN 6
#define ISR_PERIOD_US 100

#define BIT_LENGTH 110

#define HANDSHAKE_LENGTH 300
#define MAX_DATA_LENGTH 900

#define LENGTH_BITS 16

#define IDLE 0
#define HANDSHAKE 1
#define MAGIC_NUM 2
#define LENGTH 3
#define DATA 4

// Make sure this is the same as in receiver.ino or nothing works!!
#define DATA_START_MAGIC_NUMBER 0x0F

int transmission_stage;
long handshake_count;
int cycles_remaining, magic_bit;;
int16_t data_length;
int data_byte, data_bit, length_bit;
char transmission_data[MAX_DATA_LENGTH];
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
      case LENGTH:
        digitalWrite(LED_PIN, (data_length >> ((LENGTH_BITS - 1) - length_bit)) & 0x01);
        length_bit++;
        if (length_bit >= LENGTH_BITS) {
          transmission_stage = DATA;
        }
        break;
      case DATA:
        digitalWrite(LED_PIN, (transmission_data[data_byte] >> (7 - data_bit)) & 0x01);
        data_bit++;
        if (data_bit >= 8) {
          data_bit = 0;
          data_byte++;
        }
        if (data_byte >= data_length) {
          transmission_stage = IDLE;
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

void Send(String msg) {
  transmission_stage = HANDSHAKE;
  handshake_count = HANDSHAKE_LENGTH;
  magic_bit = 0;
  data_byte = 0;
  data_bit = 0;
  length_bit = 0;
  strcpy(transmission_data, msg.c_str());
  data_length = strlen(transmission_data);
  transmission_complete = false;

  Timer1.setPeriod(ISR_PERIOD_US);
  Timer1.attachInterrupt(TimerHandler);

  // Wait until the whole message was sent
  while (!transmission_complete) {
    delay(100);
  }
}

void loop() {
  Send("This is a test... Working?");
  delay(3000);
  Send("Let's try again with a different, longer message.  Does this one still work? confirming...");
  delay(3000);
  Send("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXA");
  delay(3000);
}
