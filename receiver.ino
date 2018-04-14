#include "TimerOne.h"

#define LED_PIN 3
#define LIGHT_SENSOR_PIN 0

#define READING_PERIOD_US 1000
#define BUFFER_SIZE 100

////////////////////////////////////////////////////////////////////////////////
// CIRCUILAR BUFFER DEFINITIONS & SETUP
////////////////////////////////////////////////////////////////////////////////
int buf[BUFFER_SIZE];
volatile int hi = 0, lo = 0;
int BufferDepth() {
  return (hi >= lo) ? hi - lo : BUFFER_SIZE - (lo - hi);
}
void PushIntoBuffer(int value) {
  buf[hi] = value;
  hi = (hi + 1) % BUFFER_SIZE;
}
int PopFromBuffer() {
  int next_value = buf[lo];
  lo = (lo + 1) % BUFFER_SIZE;
  return next_value;
}


////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO HANDLE TAKING LIGHT SENSOR READINGS  (Timer1)
////////////////////////////////////////////////////////////////////////////////
void TakeReading() {
  digitalWrite(LED_PIN, HIGH);
  PushIntoBuffer(analogRead(LIGHT_SENSOR_PIN));
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Timer1.initialize();
  Timer1.setPeriod(READING_PERIOD_US);
  Timer1.attachInterrupt(TakeReading); 
}

int empty_cycles = 0;
void loop() {
  int buffer_depth = BufferDepth();
  if (buffer_depth) {
    int next_value = PopFromBuffer();
    Serial.print("\n\rv: ");
    Serial.print(next_value);
    Serial.print("\td:");
    Serial.print(buffer_depth);
    Serial.print("\tc:");
    Serial.print(empty_cycles);
    Serial.print("\n\r");
    empty_cycles = 0;

  } else {
    empty_cycles++;
  }
}
