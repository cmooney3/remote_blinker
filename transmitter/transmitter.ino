#include <util/crc16.h>

#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LED_PIN 6
#define ISR_PERIOD_US 50

#define BIT_LENGTH 110

#define INTERMESSAGE_DELAY_MS 3000

#define HANDSHAKE_LENGTH 300
#define MAX_DATA_LENGTH 900

#define LENGTH_BITS 16
#define CRC16_BITS 16

uint8_t transmission_stage;
#define IDLE 0
#define HANDSHAKE 1
#define MAGIC_NUM 2
#define LENGTH 3
#define CSUM_LENGTH 4
#define DATA 5
#define CSUM_DATA 6

// Make sure this is the same as in receiver.ino or nothing works!!
#define DATA_START_MAGIC_NUMBER 0x0F

uint32_t handshake_count;
int8_t cycles_remaining, magic_bit;;
int16_t data_length, data_byte;
uint8_t data_bit, data_crc16_bit, length_bit, length_crc16_bit;
uint8_t transmission_data[MAX_DATA_LENGTH];
uint16_t data_crc16, length_crc16;
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
          transmission_stage = CSUM_LENGTH;
        }
        break;
      case CSUM_LENGTH:
        digitalWrite(LED_PIN, (length_crc16 >> ((CRC16_BITS - 1) - length_crc16_bit)) & 0x01);
        length_crc16_bit++;
        if (length_crc16_bit >= CRC16_BITS) {
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
          transmission_stage = CSUM_DATA;
        }
        break;
      case CSUM_DATA:
        digitalWrite(LED_PIN, (data_crc16 >> ((CRC16_BITS - 1) - data_crc16_bit)) & 0x01);
        data_crc16_bit++;
        if (data_crc16_bit >= CRC16_BITS) {
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

  strcpy((char*)transmission_data, msg.c_str());
  data_length = strlen((char*)transmission_data);
  data_crc16 = 0;
  for (int i = 0; i < data_length; i++) {
    data_crc16 = _crc16_update(data_crc16, transmission_data[i]);
  }
  data_crc16_bit = 0;

  length_bit = 0;
  length_crc16_bit = 0;
  length_crc16 = _crc16_update(0, (data_length >> 8) & 0xFF);
  length_crc16 = _crc16_update(length_crc16, data_length & 0xFF);

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
  delay(INTERMESSAGE_DELAY_MS);
  Send("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nam id viverra sapien. Suspendisse vel mollis urna. Nullam convallis nisl nec rhoncus consectetur. Aenean nec enim sodales, egestas mauris eget, congue nisl. Quisque blandit vitae risus in aliquet.  That's over 255 chars.  Does it still work?  What if it's way way way way longer?  Will it still work then?");
  delay(INTERMESSAGE_DELAY_MS);
  Send("Final transmission");
  delay(INTERMESSAGE_DELAY_MS);
}
