#include <util/crc16.h>

#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LASER_PIN 6

// Define the transmission rate.
// Rate: 1 / (8 * BIT_LENGTH * ISR_PERIOD_US / 1000000)  (bytes-per-second)
#define ISR_PERIOD_US 15
#define BIT_LENGTH 90

#define INTERMESSAGE_DELAY_MS 20000

#define HANDSHAKE_LENGTH 4000
#define MAX_DATA_LENGTH 500

#define LENGTH_BITS 16
#define CRC16_BITS 16

#define IDLE 0
#define HANDSHAKE 1
#define TRANSMITTING 2

// Make sure this is the same as in receiver.ino or nothing works!!
#define DATA_START_MAGIC_NUMBER 0x0F


#define MESSAGE_HEADER_LEN 7
uint8_t transmission_stage;
struct message {
  uint8_t magic_number;
  uint16_t data_length;
  uint16_t length_csum;
  uint16_t data_csum;
  uint8_t data[MAX_DATA_LENGTH];
} msg;
uint16_t cycles_remaining;
uint8_t msg_bit;
uint16_t msg_byte;
uint32_t msg_handshake_bits_remaining;
volatile bool transmission_complete;

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO CREATE BLINKS
////////////////////////////////////////////////////////////////////////////////
void TimerHandler() {
  if (!cycles_remaining) {
    switch(transmission_stage) {
      case HANDSHAKE:
        digitalWrite(LASER_PIN, !digitalRead(LASER_PIN));
        msg_handshake_bits_remaining--;
        if (msg_handshake_bits_remaining <= 0) {
          transmission_stage = TRANSMITTING;
        }
        break;
      case TRANSMITTING:
        digitalWrite(LASER_PIN, (((uint8_t*)(&msg))[msg_byte] >> (7 - msg_bit)) & 0x01);
        msg_bit++;
        if (msg_bit >= 8) {
          msg_bit = 0;
          msg_byte++;
        }
        if (msg_byte >= msg.data_length + MESSAGE_HEADER_LEN) {
          transmission_stage = IDLE;
        }
        break;
      default:
        Timer1.detachInterrupt();
        transmission_complete = true;
        digitalWrite(LASER_PIN, LOW);
        break;
    };
    cycles_remaining = BIT_LENGTH;
  }
  cycles_remaining--;
}

void Send(const char* text) {
  // Set up the message struct
  msg.magic_number = DATA_START_MAGIC_NUMBER;
  msg.data_length = strlen(text);
  msg.length_csum = _crc16_update(0, (msg.data_length >> 8) & 0xFF);
  msg.length_csum = _crc16_update(msg.length_csum, msg.data_length & 0xFF);
  strcpy((char*)msg.data, text);
  msg.data_csum = 0;
  for (uint16_t i = 0; i < msg.data_length; i++) {
    msg.data_csum = _crc16_update(msg.data_csum, msg.data[i]);
  }

  // Set up the state for a new message
  transmission_stage = HANDSHAKE;
  msg_handshake_bits_remaining = HANDSHAKE_LENGTH;
  msg_bit = msg_byte = 0;
  cycles_remaining = 0;
  transmission_complete = false;

  // Kick off the transmission
  Timer1.setPeriod(ISR_PERIOD_US);
  Timer1.attachInterrupt(TimerHandler);

  // Wait until the whole message was sent
  while (!transmission_complete) {
    delay(100);
  }
}

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  Timer1.initialize();
}

void loop() {
  Send("This is a test... Working?");
  delay(INTERMESSAGE_DELAY_MS);
  Send("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Duis malesuada sapien eu mollis facilisis. Pellentesque vitae sodales nunc. Vestibulum eleifend convallis placerat. Morbi nec enim vel enim sollicitudin faucibus eu et orci. Donec a ornare justo. Vivamus ullamcorper vulputate justo, ac posuere massa finibus ac. Donec a sapien odio. Interdum et malesuada fames ac ante ipsum primis in faucibus. Ut sollicitudin quis orci ut volutpat. ");
  delay(INTERMESSAGE_DELAY_MS);
}
