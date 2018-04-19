#include <util/crc16.h>

#include <CircularBuffer.h>
#include <limits.h>
#include <LinkedList.h>
#include <TimerOne.h>

////////////////////////////////////////////////////////////////////////////////
// BASIC CONFIGURATION PARAMETERS
////////////////////////////////////////////////////////////////////////////////
#define BAUD_RATE 115200

#define ERROR_LED_PIN 4
#define LIGHT_SENSOR_PIN 0

#define READING_PERIOD_US 1000
#define RECV_BUFFER_SIZE 600

#define HANDSHAKE_MIN_PULSES 15

// This shouldn't start with 0b01 or 0b10 since that pattern is found in the
// handshake, and could cause misalignment issues.  Start with 0b00 or 0b11
#define DATA_START_MAGIC_NUMBER 0x0F

////////////////////////////////////////////////////////////////////////////////
// ERROR CODES
////////////////////////////////////////////////////////////////////////////////
#define ENOTENOUGHPULSES 1
#define EHIGHSTDDEV 2

////////////////////////////////////////////////////////////////////////////////
// MACROS FOR TEXT FORMATTING (usefull while debugging, but not essential)
////////////////////////////////////////////////////////////////////////////////
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KBOLD "\x1B[1m"
#define KUNDL "\x1B[4m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) KBOLD x RST
#define UNDL(x) KUNDL x RST

////////////////////////////////////////////////////////////////////////////////
// CIRCUILAR BUFFERS SETUP
////////////////////////////////////////////////////////////////////////////////
CircularBuffer<uint16_t, RECV_BUFFER_SIZE> recv_buf;

////////////////////////////////////////////////////////////////////////////////
// K-MEANS TO DETECT BIMODAL LEVELS (ADJUSTING FOR LIGHT CONDITIONS)
////////////////////////////////////////////////////////////////////////////////
uint16_t KmeansDistance(uint16_t v1, uint16_t v2) {
  // Have to cast them a signed integers for the difference to work right
  return abs((int32_t)v1 - (int32_t)v2);
}

void KmeansSelectStartingCentroids(uint8_t k, uint16_t* means) {
  // Select k random starting centroids as the initial means.  The subsequent
  // centroids are weighted to not be near the initial ones.
  means[0] = recv_buf[random(recv_buf.size())];
  for (uint8_t cluster = 1; cluster < k; cluster++) {
    uint32_t total_weights = 0;

    for (uint16_t i = 0; i < recv_buf.size(); i++) {
      uint16_t d, closest_distance = 0xFF;
      for (uint8_t existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(recv_buf[i], means[existing_cluster]);
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights += closest_distance * closest_distance;
    }

    uint32_t selection = random(total_weights);

    for (uint16_t i = 0; i < recv_buf.size(); i++) {
      uint16_t d, closest_distance = 0xFF;
      for (uint8_t existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(recv_buf[i], means[existing_cluster]);
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights -= closest_distance * closest_distance;
      if (total_weights <= selection) {
        means[cluster] = recv_buf[i];
        break;
      }
    }
  }
}

void Kmeans(CircularBuffer<uint16_t, RECV_BUFFER_SIZE>* buf, uint8_t k, uint16_t *means) {
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

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO HANDLE TAKING LIGHT SENSOR READINGS  (Timer1)
////////////////////////////////////////////////////////////////////////////////
void TakeReading() {
  bool success = recv_buf.push(analogRead(LIGHT_SENSOR_PIN));
  if (!success) {
    Serial.print(F("@@"));
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
}

////////////////////////////////////////////////////////////////////////////////
// HIGH LEVEL FUNCTIONALITY
////////////////////////////////////////////////////////////////////////////////
uint16_t ComputeSplit() {
  // Using Kmeans, find a reasonable "split" light level to determine
  // if the incoming bit is a 1 or a 0.
  uint16_t *means = (uint16_t *)malloc(2 * sizeof(uint16_t));
  Kmeans(&recv_buf, 2, means);
  uint16_t split = (means[0] + means[1]) / 2;
  free(means);
  return split;
}

int16_t DetectHandshake(uint16_t split) {
  // First go through the working buffer and categorize each reading as a
  // zero or a one.  While doing this, the length of each pulse (all
  // contiguous ones or zeros) and store them in order, in a linked list.
  bool is_one = recv_buf[0] >= split;
  uint16_t bit_length = 0;
  LinkedList<uint16_t> pulse_lengths;
  for (uint16_t i = 0; i < recv_buf.size(); i++) {
    if (recv_buf[i] >= split) {
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
  if (pulse_lengths.size() <= HANDSHAKE_MIN_PULSES) {
    return -ENOTENOUGHPULSES;
  }
  pulse_lengths.remove(0);
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

  //Serial.print(F(KMAG));
  //for (int i = 0 ; i < pulse_lengths.size(); i++) {
  //  Serial.print(pulse_lengths.get(i));
  //  if (i != pulse_lengths.size() - 1) {
  //    Serial.print(", ");
  //  }
  //}
  //Serial.print("\n\r");
  //Serial.print("avg: ");
  //Serial.print(avg);
  //Serial.print("\n\r");
  //Serial.print("std dev: ");
  //Serial.print(std_dev);
  //Serial.print("\n\r");
  //Serial.println("---");
  //Serial.print(F(RST));

  // A high standard deviation means that the pulses were or varying widths,
  // and therefor, not a handshake.  They must be very precisely timed or
  // else we just assume it's random flickering -- plus we use that timing
  // for our bit length later, so if it's not reliable here, we'll likely
  // have transmission errors later on either way.
  if (std_dev > avg * 0.05) {
    return -EHIGHSTDDEV;
  }

  return round(avg);
}

void StopCollection() {
  Timer1.detachInterrupt();
}

void ClearBufferAndRestartCollection() {
  recv_buf.clear_no_memset();  // This was a hack added by me into their lib
  Timer1.setPeriod(READING_PERIOD_US);
  Timer1.attachInterrupt(TakeReading);
}

uint16_t BlockForNextReading() {
  uint16_t reading;
  while (recv_buf.isEmpty()) { delay(READING_PERIOD_US / 1000); };
  noInterrupts();
  reading = recv_buf.shift();
  interrupts();
  return reading;
}

void AlignBufferWithBitBoundaries(uint16_t split) {
  // Once we've identified a handshake and figured out what the split and bit
  // length are we need to make sure that the recieve buffer is aligned with
  // bit boundaries.  This essentially means we want the first value in the
  // buffer to be the first reading for either a 0 or a 1.  This is done by
  // consuming values until we see a boundary, and then putting that first
  // reading of the new bit back onto the buffer.
  Serial.print(F("Aligning readings with bit boundaries in handshake..."));
  // TODO: This seems somewhat naive to just assume the first edge I see is
  // well aligned.  Perhaps if I looked for several edges I could do better?
  uint16_t reading, converted_bit, value;
  bool is_first_value = true;
  do {
    reading = BlockForNextReading();
    converted_bit = (reading >= split);
    if (is_first_value) {
      value = converted_bit;
      is_first_value = false;
    }
  } while (converted_bit == value);

  // Unshift the last reading back on (it's the start of a new bit)
  noInterrupts();
  recv_buf.unshift(reading);
  interrupts();

  Serial.print(F(FGRN("\tDONE\n\r")));
}

// TODO:  Make this "realign" after some drift
uint8_t ReadNextFullBit(uint16_t bit_length, uint16_t split) {
  // Read in the right number of readings to cover one whole bit and see
  // what their average light level is.  Determine if it's a 1 or a 0 and
  // return that value.
  // Note this assumes the buffer is already aligned with the bit boundaries.
  uint16_t reading;
  uint32_t total = 0;
//  Serial.print(F("["));
  for (uint16_t sample = 0; sample < bit_length; sample++) {
    reading = BlockForNextReading();
//    Serial.print(reading);
    if (sample < bit_length - 1) {
//      Serial.print(F(" "));
    }
    total += reading;
  }
//  Serial.print(F("] = "));
  uint8_t bit = ((total / bit_length) >= split);
//  Serial.print(bit);
//  Serial.print(F("\n\r"));
  return bit;
}

uint8_t ReadNextFullByte(uint16_t bit_length, uint16_t split) {
  uint8_t val = 0;
  for (uint8_t i = 0; i < 8; i++) {
    val = (val << 1) | ReadNextFullBit(bit_length, split);
  }
  return val;
}

bool WaitForMagicNumber(uint16_t bit_length, uint16_t split) {
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
  Serial.print(F(RST));
  Serial.print(F(" "));
  if (rolling_value != 0b01010101 && rolling_value != 0b10101010) {
    Serial.print(F("\n\r"));
    Serial.println(F(FRED("Invalid bits after intital handshake detection.")));
    return false;
  }

  // Read in bits one at a time, until we see something that doesn't
  // Strictly alternate.  That means that bit must be part of the first byte
  // (and hopefully the Magic number)
  uint16_t bit = rolling_value & 0x01, last_bit;
  uint32_t count = 0;
  do {
    last_bit = bit;
    bit = ReadNextFullBit(bit_length, split);
    rolling_value = (rolling_value << 1) | bit;

    if (last_bit != bit) {
      Serial.print(F(KGRN));
    } else {
      Serial.print(F(KRED));
    }
    Serial.print(bit);
    Serial.print(F(RST));
    count++;
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
    if (rolling_value == DATA_START_MAGIC_NUMBER) {
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
    rolling_value = (rolling_value << 1) | bit;
  }
  Serial.print(F("\n\r"));

  // If that loop ends, that means we never found the magic number and somethig
  // is wrong.
  Serial.println(F(FRED("\tFAILURE, the handshake ended, but no magic number found!\n\r")));
  return false;
}

void Receive(uint16_t bit_length, uint16_t split) {
  uint8_t len[2], crc16[2], *data;
  uint16_t message_len, data_crc16, computed_data_crc16;
  ClearBufferAndRestartCollection();

  // Make sure our readings are aligned with the transmitter.
  AlignBufferWithBitBoundaries(split);

  // Now wait until the data actually starts to arrive
  if (!WaitForMagicNumber(bit_length, split)) {
    goto abort;
  }

  // Read off how long the message is (in bytes)
  len[0] = ReadNextFullByte(bit_length, split);
  len[1] = ReadNextFullByte(bit_length, split);
  message_len = (len[0] << 8) | len[1];
  Serial.print(F("Raw len data = [0x"));
  Serial.print(len[0], HEX);
  Serial.print(F(", 0x"));
  Serial.print(len[1], HEX);
  Serial.print(F("]\n\r"));
  Serial.print(F("Data length: "));
  Serial.print(message_len);
  Serial.print(F("\n\r"));

  // Collect the actual bits themselves.
  data = (uint8_t*)malloc(sizeof(uint8_t) * message_len);
  Serial.print(F(KGRN "Receiving "));
  Serial.print(message_len);
  Serial.print(F(" bytes:" RST));
  Serial.print(F(KCYN KBOLD " \""));
  for (uint16_t i = 0; i < message_len; i++) {
    data[i] = ReadNextFullByte(bit_length, split);
    Serial.print((char)data[i]);
  }
  Serial.print(F("\"" RST "\n\r"));

  // Read in the checksum for the data (CRC16) and confirm it's OK
  crc16[0] = ReadNextFullByte(bit_length, split);
  crc16[1] = ReadNextFullByte(bit_length, split);
  data_crc16 = (crc16[0] << 8) | crc16[1];
  Serial.print(F("Received CRC16 for data = [0x"));
  Serial.print(crc16[0], HEX);
  Serial.print(F(", 0x"));
  Serial.print(crc16[1], HEX);
  Serial.print(F("]  (eg: "));
  Serial.print(data_crc16);
  Serial.print(F(")\n\r"));
  computed_data_crc16 = 0;
  for (int i = 0; i < message_len; i++) {
    computed_data_crc16 = _crc16_update(computed_data_crc16, data[i]);
  }
  Serial.print(F("Computed CRC16 for data = "));
  Serial.print(computed_data_crc16);
  Serial.print(F("\n\r"));
  if (computed_data_crc16 == data_crc16) {
    Serial.print(F(FGRN("SUCCESS")));
    Serial.print(F(" -- the checksums match"));
  } else {
    Serial.print(F(FRED("FAILED")));
    Serial.print(F(" -- the checksums do not match!"));
  }
  Serial.print(F("\n\r"));

  free(data);

abort:
  Serial.print(F("\n\r"));
  StopCollection();
}

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);

  Timer1.initialize();
  ClearBufferAndRestartCollection();
}

void loop() {
  if (recv_buf.isFull()) {
    // Stop the timer from adding more readings while we work
    StopCollection();

    // Figure out what the "split" is for a 1 or a 0
    uint16_t split = ComputeSplit();

    // Using that "split", check to see if the log is full of equally-long
    // 0101010101 pattern.  This serves as the handshake and configures the
    // bit length.  That value is measured in the number of readings taken,
    // not directly as microseconds or something like that.
    int16_t bit_length = DetectHandshake(split);

    // If a handshake was detected, begin listening for a message
    if (bit_length <= 1) {
      Serial.print(F(FYEL(".")));
      //Serial.print(F("\treason: "));
      //if (bit_length == -ENOTENOUGHPULSES) {
      //    Serial.print(F("Not enough pulses detected."));
      //} else if (bit_length = -EHIGHSTDDEV) {
      //    Serial.print(F("Pulse lengths varied too much."));
      //} else {
      //    Serial.print(F("unknown ("));
      //    Serial.print(bit_length);
      //    Serial.print(F(")"));
      //}
      //Serial.print(F("\n\r"));
    } else {
      Serial.print(F("\n\r\n\r"));
      Serial.println(F(FMAG("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")));
      Serial.print(F(FMAG("X Handshake detected!")));
      Serial.print(F("\t(bit length: "));
      Serial.print(bit_length);
      Serial.print(F(") readings\n\r"));
      Serial.println(F(FMAG("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")));
      Receive((uint16_t)bit_length, split);

      Serial.print(F(FYEL("Listenining: ")));
    }

    ClearBufferAndRestartCollection();
  }
}
