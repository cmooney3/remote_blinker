#include "LaserMessaging.h"

namespace LaserMessaging {

LaserReceiver::LaserReceiver(void (*onHandshakeCallback)(), void (*onReceivingCallback)()) :
        onHandshakeCallback_(onHandshakeCallback), onReceivingCallback_(onReceivingCallback) {
  // Configure the GPIO as output that's connected to the "overflow" error LED
  pinMode(kOverflowErrorLEDPin, OUTPUT);
  digitalWrite(kOverflowErrorLEDPin, LOW);

  // Set up the timer ISR to collect readings from light sensor
  Timer1.initialize();
}

Status LaserReceiver::ListenForMessages(uint16_t timeout) {
  // Listen for incoming messages for timeout milliseconds.
  // If an incoming message is found, this function will receive the entire message
  // no matter how long it takes.  However, if no message is detected before the
  // timeout it will return.
  // The return value is one of the STATUS_* macros and tells the calling function
  // what happened.
  Serial.println(F(FWHT("----  New ListenForMessages() call ----")));
  Serial.print(F(FYEL("Listenining: ")));
  uint32_t start_time = millis();
  ClearBufferAndRestartCollection();

  while (millis() - start_time < timeout) {
    if (recv_buf_.isFull()) {
      // Stop the timer from adding more readings while we work
      StopCollection();

      // Figure out what the "split" is for a 1 or a 0
      uint16_t split = ComputeSplit();

      // Using that "split", check to see if the log is full of equally-long
      // 0101010101 pattern.  This serves as the handshake and configures the
      // bit length.  That value is measured in the number of readings taken,
      // not directly as microseconds or something like that.
      int16_t bit_length = DetectHandshake(split);

      if (bit_length > 1) {
        // If a handshake was detected, begin listening for a message
        Serial.print(F("\n\r\n\r"));
        Serial.println(F(FMAG("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")));
        Serial.print(F(FMAG("X Handshake detected!")));
        Serial.print(F("\t(bit length: "));
        Serial.print(bit_length);
        Serial.print(F(" readings)\n\r"));
        Serial.println(F(FMAG("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")));

        // Run the callback now, since we've found a handshake and are now tracking it.
        if (onHandshakeCallback_) {
          onHandshakeCallback_();
        }
        return Receive((uint16_t)bit_length, split);
      } else {
        Serial.print(F(FYEL(".")));
      }
    }
  }

  Serial.print(F("\r\n"));
  return Status::TIMEOUT; // We timed out waiting for a message to arrive
}

const char* LaserReceiver::GetMessage() const {
  return message_;
}

void LaserReceiver::StopCollection() {
  Timer1.detachInterrupt();
}

void LaserReceiver::RestartCollection() {
  Timer1.attachInterrupt(TakeReading);
}

void LaserReceiver::ClearBufferAndRestartCollection() {
  recv_buf_.clear();
  Timer1.setPeriod(kReadingPeriodUS);
  Timer1.attachInterrupt(TakeReading);
}

uint16_t LaserReceiver::BlockForNextReading() {
  while (recv_buf_.isEmpty()) { delay(kReadingPeriodUS / 1000); };
  StopCollection();
  uint16_t reading = recv_buf_.shift();
  RestartCollection();
  return reading;
}

void LaserReceiver::PutReadingBack(uint16_t reading) {
  // TODO: Does this really work!? It seems like there might be other readings slipping into the buffer between.
  // Do you need to defer interrupts between the time you made this reading and when you return it?
  // Seems like maybe....
  StopCollection();
  recv_buf_.unshift(reading);
  RestartCollection();
}

bool LaserReceiver::ConvertReading(uint16_t reading, uint16_t split) const {
  return reading >= split;
}



uint16_t LaserReceiver::ComputeSplit() const {
  // Using Kmeans, find a reasonable "split" light level to determine
  // if the incoming bit is a 1 or a 0.
  uint16_t *means = (uint16_t *)malloc(2 * sizeof(uint16_t));
  Kmeans(&recv_buf_, 2, means);
  uint16_t split = (means[0] + means[1]) / 2;
  free(means);
  return split;
}


void LaserReceiver::Kmeans(CircularBuffer<uint16_t, RECV_BUFFER_SIZE>* buf,
                           uint8_t k, uint16_t *means) const {
  uint32_t *totals = (uint32_t *)malloc(k * sizeof(uint32_t));
  uint16_t *counts = (uint16_t *)malloc(k * sizeof(uint16_t));

  KmeansSelectStartingCentroids(k, means);

  bool same_means_as_last_time;
  do {
    // Recompute the means by going through all the values and seeing which
    // cluster they're closest to.  Then recompute the mean of each cluster
    // to hopefully move it to a more representative center point.
    for (uint8_t i = 0; i < k; i++) {
      totals[i] = counts[i] = 0;
    }
    for (uint16_t i = 0; i < buf->size(); i++) {
      // Determine which cluster this item is closest to
      uint8_t closest_cluster = 0xFF;
      uint16_t closest_cluster_distance = 0xFF;
      for (uint8_t cluster = 0; cluster < k; cluster++) {
        uint16_t distance = KmeansDistance((*buf)[i], means[cluster]);
        if (distance < closest_cluster_distance) {
          closest_cluster = cluster;
          closest_cluster_distance = distance;
        }
      }

      // Update the values for the closest cluster
      counts[closest_cluster]++;
      totals[closest_cluster] += (*buf)[i];
    }

    // Having finished with all the items, we now compute the new means
    same_means_as_last_time = true;
    for (uint8_t i = 0; i < k; i++) {
      uint16_t new_mean = totals[i] / counts[i];
      if (new_mean != means[i]) {
        same_means_as_last_time = false;
      }
      means[i] = new_mean;
    }
  } while(!same_means_as_last_time);

  free(totals);
  free(counts);
}

void LaserReceiver::KmeansSelectStartingCentroids(uint8_t k, uint16_t* means) const {
  // Select k random starting centroids as the initial means.  The subsequent
  // centroids are weighted to not be near the initial ones.
  means[0] = recv_buf_[random(recv_buf_.size())];
  for (uint8_t cluster = 1; cluster < k; cluster++) {
    uint32_t total_weights = 0;

    for (uint16_t i = 0; i < recv_buf_.size(); i++) {
      uint16_t d, closest_distance = 0xFF;
      for (uint8_t existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(recv_buf_[i], means[existing_cluster]);
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights += closest_distance * closest_distance;
    }

    uint32_t selection = random(total_weights);

    for (uint16_t i = 0; i < recv_buf_.size(); i++) {
      uint16_t d, closest_distance = 0xFF;
      for (uint8_t existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(recv_buf_[i], means[existing_cluster]);
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights -= closest_distance * closest_distance;
      if (total_weights <= selection) {
        means[cluster] = recv_buf_[i];
        break;
      }
    }
  }
}

uint16_t LaserReceiver::KmeansDistance(uint16_t v1, uint16_t v2) const {
  // Have to cast them a signed integers for the difference to work right
  return abs((int32_t)v1 - (int32_t)v2);
}

int16_t LaserReceiver::DetectHandshake(uint16_t split) {
  // First go through the working buffer and categorize each reading as a
  // zero or a one.  While doing this, the length of each pulse (all
  // contiguous ones or zeros) and store them in order, in a linked list.
  bool is_one = recv_buf_[0] >= split;
  uint16_t bit_length = 0;
  LinkedList<uint16_t> pulse_lengths;
  for (uint16_t i = 0; i < recv_buf_.size(); i++) {
    if (recv_buf_[i] >= split) {
      if (is_one) {
        bit_length++;
      } else {
        pulse_lengths.add(bit_length);
        bit_length = 1;
      }
      is_one = true;
    } else {
      if (!is_one) {
        bit_length++;
      } else {
        pulse_lengths.add(bit_length);
        bit_length = 1;
      }
      is_one = false;
    }
  }

  // First off, we remove the first and last values -- they're unlikely to
  // be aligned with the window we scanned, and will therefor not be a good
  // measure of the length.  Eg: they might be "cut off"
  if (pulse_lengths.size() <= kHandshakeMinPulses) {
    return -HandshakeRejectionReason::NOT_ENOUGH_PULSES;
  }
  pulse_lengths.pop();
  pulse_lengths.remove(pulse_lengths.size() - 1);

  // Next, compute some statistics on the pulse lengths to see if they seem
  // to be a handshake.  (all pulses are the same length)
  // Note some values are shifted to make them larger before dividing, thus
  // allowing us to avoid using floating point as long as we divide that value
  // out later.  bitwise shifts are used for efficiency's sake.
  // Start out with the mean
  uint32_t total = 0;
  for (uint16_t i = 0; i < pulse_lengths.size(); i++) {
    total += pulse_lengths.get(i);
  }
  double avg = (double)total / (double)pulse_lengths.size();
  // Next compute the std deviation
  double deviations = 0, d;
  for (uint16_t i = 0; i < pulse_lengths.size(); i++) {
    d = (double)pulse_lengths.get(i) - avg;
    deviations += d * d;
  }
  double std_dev = sqrt(deviations / (double)pulse_lengths.size());

  // A high standard deviation means that the pulses were or varying widths,
  // and therefor, not a handshake.  They must be very precisely timed or
  // else we just assume it's random flickering -- plus we use that timing
  // for our bit length later, so if it's not reliable here, we'll likely
  // have transmission errors later on either way.
  if (std_dev > avg * 0.25) {
    return -HandshakeRejectionReason::HIGH_STD_DEV;
  }

  return round(avg);
}

bool LaserReceiver::AlignBufferWithBitBoundaries(uint16_t bit_length, uint16_t split) {
  // Once we've identified a handshake and figured out what the split and bit
  // length are we need to make sure that the recieve buffer is aligned with
  // bit boundaries.  This essentially means we want the first value in the
  // buffer to be the first reading for either a 0 or a 1.  This is done by
  // consuming values until we see a boundary, and then putting that first
  // reading of the new bit back onto the buffer.
  Serial.print(F("Aligning readings with bit boundaries in handshake..."));
  // TODO: This seems somewhat naive to just assume the first edge I see is
  // well aligned.  Perhaps if I looked for several edges I could do better?
  uint16_t reading_number = 0, reading, converted_bit, value;
  bool is_first_value = true;
  do {
    // If we've been looking for a bit boundary for too long, give up
    if (reading_number++ > bit_length * 2) {
      return false;
    }

    reading = BlockForNextReading();
    converted_bit = (reading >= split);
    if (is_first_value) {
      value = converted_bit;
      is_first_value = false;
    }
  } while (converted_bit == value);

  // Unshift the last reading back on (it's the start of a new bit)
  PutReadingBack(reading);

  Serial.print(F(FGRN("\tDONE\n\r")));
  return true;
}

bool LaserReceiver::ReadNextFullBit(uint16_t bit_length, uint16_t split) {
  // Read in the right number of readings to cover one whole bit and see
  // what their average light level is.  Determine if it's a 1 or a 0 and
  // return that value.
  // Note this assumes the buffer is already aligned with the bit boundaries.
  uint16_t readings[bit_length];
  uint32_t total = 0;
  for (uint16_t i = 0; i < bit_length; i++) {
    readings[i] = BlockForNextReading();
    total += readings[i];
  }
  bool bit = ConvertReading((total / bit_length), split);

  // Try to "realign" the bit boundaries if there are some stray readings on
  // either end.  This combats drift by fixing it when we can pick up on it.
  // Obviously this only works on 1/0 or 0/1 boundaries, but when those
  // occur we can try to get back on track and if the data is reasonably
  // distributed, this should work for a long time.
  // First check if any of the first readings were of the wrong "bit" -- if
  // this is true, we should try to consume a few more readings to realign.
  uint16_t pos = 0; 
  while (ConvertReading(readings[pos], split) != bit) {
    pos++;
  }
  if (pos > 0) {
    // If any of the readings at the beginning of the bit don't agree with
    // the majority, we should pull an equal amount of additional readings.
    // If those readings don't match the majority, just put them back (we
    // probably weren't actually drifting) otherwise consume them to shift
    // our bit boundary slightly forwards.
    for (uint8_t i = 0; i < pos; i++) {
      int16_t new_reading = BlockForNextReading();
      // This this reading is a different value, put it back and stop
      if (ConvertReading(new_reading, split) != bit) {
        PutReadingBack(new_reading);
        break;
      }
    }
  } else {
    // Next, if there's no drift at the beginning of the bit, check if any
    // of the last readings should get "put back" because they really should
    // be part of the next bit.
    pos = bit_length - 1;
    while (ConvertReading(readings[pos], split) != bit) {
      PutReadingBack(readings[pos]);
      pos--;
    }
  }

  return bit;
}

uint8_t LaserReceiver::ReadNextFullByte(uint16_t bit_length, uint16_t split) {
  uint8_t val = 0;
  for (uint8_t i = 0; i < 8; i++) {
    val = (val << 1) | (uint8_t)ReadNextFullBit(bit_length, split);
  }
  return val;
}

uint16_t LaserReceiver::Read16BitInt(uint16_t bit_length, uint16_t split) {
  uint8_t byte0 = ReadNextFullByte(bit_length, split);
  uint8_t byte1 = ReadNextFullByte(bit_length, split);
  return (byte1 << 8) | byte0;
}

bool LaserReceiver::WaitForMagicNumber(uint16_t bit_length, uint16_t split) {
  // This function reads in (already confirmed) handshake byte by byte and
  // checks for the end of the handshake.  This is indicated by a "magic
  // number that tells the receiver that data will be coming next.
  Serial.println(F("Waiting for data to start..."));

  Serial.print(F("Tracking handshake: "));
  // Get a full byte to fill out the rolling value to start with;
  uint8_t rolling_value = ReadNextFullByte(bit_length, split);
  if (rolling_value == 0b01010101 || rolling_value == 0b10101010) {
    Serial.print(F(KGRN));
  } else {
    Serial.print(F(KRED));
  }
  for (int16_t i = 7; i >= 0; i--) {
    Serial.print((rolling_value & (0x01 << i)) >> i);
  }
  Serial.print(F(RST "\n\r"));
  if (rolling_value != 0b01010101 && rolling_value != 0b10101010) {
    Serial.print(F("\n\r"));
    Serial.println(F(FRED("Invalid bits after intital handshake detection.")));
    return false;
  }

  // Read in bits one at a time, until we see something that doesn't
  // Strictly alternate.  That means that bit must be part of the first byte
  // (and hopefully the Magic number)
  bool bit = (bool)(rolling_value & 0x01), last_bit;
  uint32_t count = 0;
  char sprites[] = {'|', '/', '-', '\\', '|', '/', '-', '\\'};
  uint8_t anim_frame = 0, num_sprites = sizeof(sprites) / sizeof(sprites[0]);
  do {
    last_bit = bit;
    bit = ReadNextFullBit(bit_length, split);
    rolling_value = (rolling_value << 1) | (uint8_t)bit;
    count++;

    if (count % kHandshakeAnimationInterval == 0) {
      Serial.print(F("\r"));
      Serial.print(sprites[anim_frame]);
      Serial.print(F("\t"));
      Serial.print(count);
      Serial.print(F(" "));
      anim_frame = (anim_frame + 1) % num_sprites;
    }
  } while (last_bit != bit);
  Serial.print(F("\n\rRead "));
  Serial.print(count);
  Serial.print(F(" bits of handshake.\n\r"));


  // Now read in up to 8 more bits, checking the rolling value against the
  // magic number.  If we never find it, then something's wrong.  All 8 of
  // may not be used, but we can't always know exactly where the Magic #
  // starts (basically we're aligning by byte now, instead of by bit)
  Serial.println(F("Attempting to align at the byte level w/ magic number..."));
  Serial.print(F("\tCandidates:"));
  for (uint8_t i = 0; i < 8; i++) {
    if (rolling_value == kDataStartMagicNumber) {
      // We've got a full magic number stored in the byte, so we're good.  We
      // now are fully byte-aligned and ready to receive the actual data.
      Serial.print(F(KGRN));
      Serial.print(F(" 0x"));
      Serial.print(rolling_value, HEX);
      Serial.print(F(RST));
      Serial.print(F("\n\r"));
      Serial.println(F(FGRN("\tSUCCESS, magic number found and we're now byte aligned.")));
      return true;
    } else {
      Serial.print(F(KYEL));
      Serial.print(F(" 0x"));
      Serial.print(rolling_value, HEX);
      Serial.print(F(RST));
    }
    bit = ReadNextFullBit(bit_length, split);
    rolling_value = (rolling_value << 1) | (uint8_t)bit;
  }
  Serial.print(F("\n\r"));

  // If that loop ends, that means we never found the magic number and somethig
  // is wrong.
  Serial.println(F(FRED("\tFAILURE, the handshake ended, but no magic number found!\n\r")));
  return false;
}

uint16_t LaserReceiver::ReadInLength(uint16_t bit_length, uint16_t split) {
  // First read in two bytes (16 bit length field)
  uint16_t message_len = Read16BitInt(bit_length, split);
  Serial.print(F("Data length: "));
  Serial.print(message_len);
  Serial.print(F("\n\r"));

  // Now read in the crc16 for the length, to make sure it's all good
  uint16_t crc16 = Read16BitInt(bit_length, split);
  Serial.print(F("Received length CRC: 0x"));
  Serial.print(crc16, HEX);
  Serial.print(F("\n\r"));

  // Confirm that the CRC is correct
  uint16_t computed_crc16 = 0;
  computed_crc16 = _crc16_update(computed_crc16, (message_len >> 8) & 0xFF);
  computed_crc16 = _crc16_update(computed_crc16, message_len & 0xFF);
  Serial.print(F("Computed length CRC: 0x"));
  Serial.print(computed_crc16, HEX);
  Serial.print(F("\n\r"));

  if (computed_crc16 == crc16) {
    Serial.println(F(FGRN("SUCCESS") " -- length checksums match"));
    return message_len;
  } else {
    Serial.println(F(FRED("ERROR") " -- length checksums don't match"));
    return -1;
  }
}

Status LaserReceiver::Receive(uint16_t bit_length, uint16_t split) {
  Status status;
  uint16_t message_crc16, computed_message_crc16;
  int16_t message_len;
  ClearBufferAndRestartCollection();

  // Make sure our readings are aligned with the transmitter.
  bool successfully_aligned = AlignBufferWithBitBoundaries(bit_length, split);

  // Now wait until the data actually starts to arrive
  if (!successfully_aligned || !WaitForMagicNumber(bit_length, split)) {
    status = Status::LOST_HANDSHAKE;
    goto abort;
  }

  // Run the "onReceiving" callback now, because we've just seen the start of the data!
  if (onReceivingCallback_) {
    onReceivingCallback_();
  }

  // Get the message length and confirm that it's checksum is correct.
  message_len = ReadInLength(bit_length, split);
  if (message_len < 0) {
    status = Status::BAD_LENGTH;
    goto abort;
  }

  // Read in the checksum for the message (CRC16) and confirm it's OK
  message_crc16 = Read16BitInt(bit_length, split);
  Serial.print(F("Received CRC16 for message = 0x"));
  Serial.print(message_crc16, HEX);
  Serial.print(F("\n\r"));

  // Collect the actual bits themselves.
  Serial.print(F(KGRN "Receiving "));
  Serial.print(message_len);
  Serial.print(F(" bytes:" RST));
  Serial.print(F(KCYN KBOLD " \""));
  for (uint16_t i = 0; i < message_len && i < MAX_MESSAGE_LENGTH; i++) {
    message_[i] = ReadNextFullByte(bit_length, split);
    Serial.print(message_[i]);
  }
  Serial.print(F("\"" RST "\n\r"));
  // Make sure there's a null terminator
  message_[(message_len < MAX_MESSAGE_LENGTH) ? message_len : MAX_MESSAGE_LENGTH] = 0x0;

  // Confirm that the message's checksum matches the one we received
  computed_message_crc16 = 0;
  for (int i = 0; i < message_len; i++) {
    computed_message_crc16 = _crc16_update(computed_message_crc16, message_[i]);
  }
  Serial.print(F("Computed CRC16 for message = 0x"));
  Serial.print(computed_message_crc16, HEX);
  Serial.print(F("\n\r"));
  if (computed_message_crc16 == message_crc16) {
    status = Status::SUCCESS;
    Serial.print(F(FGRN("SUCCESS")));
    Serial.print(F(" -- message checksums match"));
  } else {
    status = Status::BAD_CHECKSUM;
    Serial.print(F(FRED("FAILED")));
    Serial.print(F(" -- message checksums do not match!"));
  }
  Serial.print(F("\n\r"));

abort:
  Serial.print(F("\n\r"));
  StopCollection();

  return status;
}

};  // LaserMessaging namespace
