#include <LaserMessaging.h>
#include "FastLED.h"

LaserMessaging::LaserReceiver *receiver;

#define BAUD_RATE 115200

#define NUM_LEDS 10
CRGB leds[NUM_LEDS];
#define LED_DATA_PIN 8
#define LED_CLOCK_PIN 9

#define STARTUP_INDICATOR_TIME_MS 750

#define COLOR_OFF CRGB::Black
#define COLOR_STARTUP CRGB::Magenta
#define COLOR_HANDSHAKE CRGB::Yellow
#define COLOR_RECEIVING CRGB::Chartreuse
#define COLOR_SUCCESS CRGB::Green
#define COLOR_FAILURE CRGB::Red

#define NUM_STATUS_BLINKS 5
#define STATUS_BLINK_LENGTH_MS 300

void setBeaconColor(CRGB color) {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void blinkBeacon(CRGB color, uint16_t duration_ms) {
  setBeaconColor(color);
  delay(duration_ms);
}

void onHandshakeCallback() {
  setBeaconColor(COLOR_HANDSHAKE);
}

void onReceivingCallback() {
  setBeaconColor(COLOR_RECEIVING);
}


void setup() {
  Serial.begin(BAUD_RATE);

  receiver = new LaserMessaging::LaserReceiver(onHandshakeCallback, onReceivingCallback);

  // Initialize the Beacon's LEDS, and blink them to make a reboot visible.
  FastLED.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);
  blinkBeacon(COLOR_STARTUP, STARTUP_INDICATOR_TIME_MS);
  setBeaconColor(COLOR_OFF);
}

void loop() {
  uint8_t status = receiver->ListenForMessages(1000);

  Serial.print(F(FWHT("STATUS:\t")));
  if (status == LaserMessaging::Status::TIMEOUT) {
    Serial.println(F("TIMEOUT"));
  } else if (status == LaserMessaging::Status::LOST_HANDSHAKE) {
    // Only blink very briefly so it can reaquire the handshake if possible
    Serial.println(F("LOST_HANDSHAKE"));
    blinkBeacon(COLOR_FAILURE, STATUS_BLINK_LENGTH_MS / 2);
    setBeaconColor(COLOR_OFF);
  } else {
    if (status == LaserMessaging::Status::SUCCESS) {
      Serial.println(F("SUCCESS"));
    } else {
      Serial.print(F("FAILED SOME KIND OF CHECKSUM ("));
      Serial.print(status);
      Serial.println(F(")"));
    }
    // Display to the transmitter if the message was successfully received or not by blinking an indicator color
    uint32_t color = (status == LaserMessaging::Status::SUCCESS) ? COLOR_SUCCESS : COLOR_FAILURE;
    for (uint8_t blink = 0; blink < NUM_STATUS_BLINKS; blink++) {
      blinkBeacon(COLOR_OFF, STATUS_BLINK_LENGTH_MS);
      blinkBeacon(color, STATUS_BLINK_LENGTH_MS);
    }
    delay(STATUS_BLINK_LENGTH_MS * 2);  // Make the last blink extra long for a "final" feel
    setBeaconColor(COLOR_OFF);
  }
}
