#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <LaserMessaging.h>
#include "FastLED.h"

LaserMessaging::LaserReceiver receiver;

static char morse_message[MAX_MESSAGE_LENGTH] = {'n', 'o', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e'};
char* curr_morse_char;
CRGB morse_color;

#define BAUD_RATE 115200

#define NUM_LEDS 10
CRGB leds[NUM_LEDS];
#define LED_DATA_PIN 8
#define LED_CLOCK_PIN 9

// EEPROM addresses/values for various values
#define MAGIC_NUMBER_ADDR 0
#define MAGIC_NUMBER 0xED
#define CONTENTS_ADDR 2

// How long the initial beacon blink is after a reboot
#define STARTUP_INDICATOR_TIME_MS 750

#define COLOR_OFF CRGB::Black
#define COLOR_STARTUP CRGB::Magenta
#define COLOR_HANDSHAKE CRGB::Yellow
#define COLOR_RECEIVING CRGB::Chartreuse
#define COLOR_SUCCESS CRGB::Green
#define COLOR_FAILURE CRGB::Red

#define NUM_STATUS_BLINKS 5
#define STATUS_BLINK_LENGTH_MS 300


#define MORSE_SPACING_UNIT_MS 300
#define MORSE_DOT_MS MORSE_SPACING_UNIT_MS
#define MORSE_DASH_MS (MORSE_SPACING_UNIT_MS * 3)
#define MORSE_INTRACHARACTER_PAUSE_MS MORSE_SPACING_UNIT_MS
#define MORSE_INTERCHARACTER_PAUSE_MS (MORSE_SPACING_UNIT_MS * 3)
#define MORSE_INTERWORD_PAUSE_MS (MORSE_SPACING_UNIT_MS * 7)
#define MORSE_INTERMESSAGE_PAUSE_MS 10000

static const PROGMEM char alpha[26][5] = {
    {'.', '-', 'X', 'X', 'X'}, //A
    {'-', '.', '.', '.', 'X'}, //B
    {'-', '.', '-', '.', 'X'}, //C
    {'-', '.', '.', 'X', 'X'}, //D
    {'.', 'X', 'X', 'X', 'X'}, //E
    {'.', '.', '-', '.', 'X'}, //F
    {'-', '-', '.', 'X', 'X'}, //G
    {'.', '.', '.', '.', 'X'}, //H
    {'.', '.', 'X', 'X', 'X'}, //I
    {'.', '-', '-', '-', 'X'}, //J
    {'-', '.', '-', 'X', 'X'}, //K
    {'.', '-', '.', '.', 'X'}, //L
    {'-', '-', 'X', 'X', 'X'}, //M
    {'-', '.', 'X', 'X', 'X'}, //N
    {'-', '-', '-', 'X', 'X'}, //O
    {'.', '-', '-', '.', 'X'}, //P
    {'-', '-', '.', '-', 'X'}, //Q
    {'.', '-', '.', 'X', 'X'}, //R
    {'.', '.', '.', 'X', 'X'}, //S
    {'-', 'X', 'X', 'X', 'X'}, //T
    {'.', '.', '-', 'X', 'X'}, //U
    {'.', '.', '.', '-', 'X'}, //V
    {'.', '-', '-', 'X', 'X'}, //W
    {'-', '.', '.', '-', 'X'}, //X
    {'-', '.', '-', '-', 'X'}, //Y
    {'-', '-', '.', '.', 'X'}  //Z
};

static const PROGMEM char num[10][5] = {
    {'-', '-', '-', '-', '-'}, //0
    {'.', '-', '-', '-', '-'}, //1
    {'.', '.', '-', '-', '-'}, //2
    {'.', '.', '.', '-', '-'}, //3
    {'.', '.', '.', '.', '-'}, //4
    {'.', '.', '.', '.', '.'}, //5
    {'-', '.', '.', '.', '.'}, //6
    {'-', '-', '.', '.', '.'}, //7
    {'-', '-', '-', '.', '.'}, //8
    {'-', '-', '-', '-', '.'}  //9
};

static const char* ConvertCharToMorse(char c) {
  if ('A' <= c && c <= 'Z') {
    return alpha[c - 'A'];
  } else if ('a' <= c && c <= 'z') {
    return alpha[c - 'a'];
  } else if ('0' <= c && c <= '9') {
    return num[c - '0'];
  }
  return NULL;
}

void saveNewMessageToEEPROM(char const *message) {
  // Write a magic number in a known location to let the system know this is a valid message
  EEPROM.write(MAGIC_NUMBER_ADDR, MAGIC_NUMBER);

  // Store the actual message charcter by character after that
  uint16_t length = strlen(message);
  for (uint16_t i = 0; i <= length; i++) {  // <= copies the null terminator over as well
    EEPROM.write(CONTENTS_ADDR + i, message[i]);
  }
}

// Check through the EEPROM to see if there's a stored message.  If so, read it into the
// supplied message buffer and return true.  Otherwise return false to indicate that no
// message can be found in eeprom.
bool restoreMessageFromEEPROM(char *message) {
  bool is_valid = (EEPROM.read(MAGIC_NUMBER_ADDR) == MAGIC_NUMBER);
  if (!is_valid) {
    return false;
  } else {
    uint16_t pos = 0;
    do {
      message[pos] = EEPROM.read(CONTENTS_ADDR + pos);
      pos++;
    } while (message[pos - 1] != 0x0);
    return true;
  }
}


static void setBeaconColor(CRGB color) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

static void blinkBeacon(CRGB color, uint16_t duration_ms) {
  setBeaconColor(color);
  ListenForMessagesWhileWaiting(duration_ms);
}

static void blinkBeaconWithoutListening(CRGB color, uint16_t duration_ms) {
  setBeaconColor(color);
  delay(duration_ms);
}

static void onHandshakeCallback() {
  setBeaconColor(COLOR_HANDSHAKE);
}

static void onReceivingCallback() {
  setBeaconColor(COLOR_RECEIVING);
}

static void ListenForMessagesWhileWaiting(uint16_t wait_time_ms) {
  uint8_t status = receiver.ListenForMessages(wait_time_ms);

  Serial.print(F(FWHT("STATUS:\t")));
  if (status == LaserMessaging::Status::TIMEOUT) {
    Serial.println(F("TIMEOUT"));
  } else if (status == LaserMessaging::Status::LOST_HANDSHAKE) {
    // Only blink very briefly so it can reaquire the handshake if possible
    Serial.println(F("LOST_HANDSHAKE"));
    blinkBeaconWithoutListening(COLOR_FAILURE, STATUS_BLINK_LENGTH_MS / 2);
    setBeaconColor(COLOR_OFF);
  } else {
    if (status == LaserMessaging::Status::SUCCESS) {
      Serial.println(F(FGRN("SUCCESS")));
      Serial.print(F("Message Received: \""));
      Serial.print(receiver.GetMessage());
      Serial.println(F("\""));
      saveNewMessageToEEPROM(receiver.GetMessage());
      restoreMessageFromEEPROM(morse_message);
      restartMorseMessage();
    } else {
      Serial.print(F(FRED("FAILED") " SOME KIND OF CHECKSUM ("));
      Serial.print(status);
      Serial.println(F(")"));
    }
    // Display to the transmitter if the message was successfully received or not by blinking an indicator color
    uint32_t color = (status == LaserMessaging::Status::SUCCESS) ? COLOR_SUCCESS : COLOR_FAILURE;
    for (uint8_t blink = 0; blink < NUM_STATUS_BLINKS; blink++) {
      blinkBeaconWithoutListening(COLOR_OFF, STATUS_BLINK_LENGTH_MS);
      blinkBeaconWithoutListening(color, STATUS_BLINK_LENGTH_MS);
    }
    delay(STATUS_BLINK_LENGTH_MS * 2);  // Make the last blink extra long for a "final" feel
    setBeaconColor(COLOR_OFF);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  // Set up the laser receiver code.
  receiver.Setup(onHandshakeCallback, onReceivingCallback);

  // Initialize the Beacon's LEDS, and blink them to make a reboot visible.
  FastLED.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);
  blinkBeacon(COLOR_STARTUP, STARTUP_INDICATOR_TIME_MS);
  setBeaconColor(COLOR_OFF);
}

void restartMorseMessage() {
  curr_morse_char = &morse_message[0];
  morse_color = CHSV(random8(),255,255);
}

void loop() {
  restoreMessageFromEEPROM(morse_message);

  // Get the Morse code message set up
  restartMorseMessage();

  while (true) {
    if (*curr_morse_char != 0x0) {
      Serial.print(*curr_morse_char);
    } else {
      Serial.println();
    }

    if (*curr_morse_char == 0x0) {
      restartMorseMessage();
      blinkBeacon(COLOR_OFF, MORSE_INTERMESSAGE_PAUSE_MS);
      continue;
    } else if (*curr_morse_char == ' ') {
      blinkBeacon(COLOR_OFF, MORSE_INTERWORD_PAUSE_MS);
    } else {
      const char *morse = ConvertCharToMorse(*curr_morse_char);
      while (pgm_read_byte(morse) != 'X') {
        blinkBeacon(morse_color, (pgm_read_byte(morse) == '.') ? MORSE_DOT_MS : MORSE_DASH_MS);
        if (pgm_read_byte(morse + 1) != 'X') {
          blinkBeacon(COLOR_OFF, MORSE_INTRACHARACTER_PAUSE_MS);
        }
        morse++;
      }

      char next_char = *(curr_morse_char + 1);
      if (next_char != 0x0 && next_char != ' ') {
        blinkBeacon(COLOR_OFF, MORSE_INTERCHARACTER_PAUSE_MS);
      }
    }

    curr_morse_char++;
  }
}
