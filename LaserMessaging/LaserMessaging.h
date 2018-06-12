#ifndef LASER_MESSAGING_H
#define LASER_MESSAGING_H

#include <util/crc16.h>
#include <CircularBuffer.h>
#include <LinkedList.h>
#include <TimerOne.h>

#include "colors.h"

#define RECV_BUFFER_SIZE 250

namespace LaserMessaging {

static const uint8_t kLightSensorPin = 0;
static const uint8_t kOverflowErrorLEDPin = 4;
static const uint8_t kReadingPeriodUS = 150;

// This shouldn't start with 0b01 or 0b10 since that pattern is found in the
// handshake, and could cause misalignment issues.  Start with 0b00 or 0b11
static const uint16_t kDataStartMagicNumber = 0x0F;
static const uint16_t kHandshakeMinPulses = 8;

static const uint8_t kHandshakeAnimationInterval = 23;


// The circular receiving buffer and the ISR to fill it at regular intervals
static CircularBuffer<uint16_t, RECV_BUFFER_SIZE> recv_buf_;
static void TakeReading() {
  bool success = recv_buf_.push(analogRead(kLightSensorPin));
  if (!success) {
    Serial.print(F("@@"));
    digitalWrite(kOverflowErrorLEDPin, HIGH);
  }
}

// Enums defining various return codes
enum Status { LOST_HANDSHAKE, BAD_LENGTH, BAD_CHECKSUM, TIMEOUT, SUCCESS };
enum HandshakeRejectionReason { NOT_ENOUGH_PULSES, HIGH_STD_DEV };

// The message receiver class.
class LaserReceiver {
  public:
    LaserReceiver(void (*onHandshakeCallback)(), void (*onReceivingCallback)());
    Status ListenForMessages(uint16_t timeout);

  private:
    // These functions offer high-level functionality to navigate the protocol
    int16_t DetectHandshake(uint16_t split);
    bool AlignBufferWithBitBoundaries(uint16_t bit_length, uint16_t split);
    bool WaitForMagicNumber(uint16_t bit_length, uint16_t split);
    bool ReadNextFullBit(uint16_t bit_length, uint16_t split);
    Status Receive(uint16_t bit_length, uint16_t split);
    uint8_t ReadNextFullByte(uint16_t bit_length, uint16_t split);
    uint16_t Read16BitInt(uint16_t bit_length, uint16_t split);
    uint16_t ReadInLength(uint16_t bit_length, uint16_t split);

    // These functions are used to controll when readings are being taken.
    void ClearBufferAndRestartCollection();
    void StopCollection();
    void RestartCollection();

    // These functions allow for manipulation of the receive buffer
    uint16_t BlockForNextReading();
    void PutReadingBack(uint16_t reading);

    // These functions are used to decode the incoming stream of data into individual bits
    uint16_t ComputeSplit() const;
    bool ConvertReading(uint16_t reading, uint16_t split) const;
    void Kmeans(CircularBuffer<uint16_t, RECV_BUFFER_SIZE>* buf, uint8_t k, uint16_t *means) const;
    void KmeansSelectStartingCentroids(uint8_t k, uint16_t* means) const;
    uint16_t KmeansDistance(uint16_t v1, uint16_t v2) const;

    void (*onHandshakeCallback_)();
    void (*onReceivingCallback_)();
};

};  // LaserMessaging namespace

#endif  // LASER_MESSAGING_H
