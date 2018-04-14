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

#define HANDSHAKE_MIN_PULSES 4

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

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

////////////////////////////////////////////////////////////////////////////////
// CIRCUILAR BUFFERS SETUP
////////////////////////////////////////////////////////////////////////////////
CircularBuffer<int, RECV_BUFFER_SIZE> recv_buf;

////////////////////////////////////////////////////////////////////////////////
// K-MEANS TO DETECT BIMODAL LEVELS (ADJUSTING FOR LIGHT CONDITIONS)
////////////////////////////////////////////////////////////////////////////////
int KmeansDistance(int v1, int v2) {
  return abs(v1 - v2);
}

void KmeansSelectStartingCentroids(int k, int* means) {
  // Select k random starting centroids as the initial means.  The subsequent
  // centroids are weighted to not be near the initial ones.
  means[0] = recv_buf[random(recv_buf.size())];
  for (int cluster = 1; cluster < k; cluster++) {
    long total_weights = 0;

    for (int i = 0; i < recv_buf.size(); i++) {
      int closest_distance = INT_MAX;
      int d;
      for (int existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(recv_buf[i], means[existing_cluster]); 
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights += closest_distance * closest_distance;
    }

    int selection = random(total_weights);

    for (int i = 0; i < recv_buf.size(); i++) {
      int closest_distance = INT_MAX;
      int d;
      for (int existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
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

void Kmeans(CircularBuffer<int, RECV_BUFFER_SIZE>* buf, int k, int *means) {
  long *totals = (long *)malloc(k * sizeof(long));
  int *counts = (int *)malloc(k * sizeof(int));

  KmeansSelectStartingCentroids(k, means);

  int same_means_as_last_time;
  do {
    // Recompute the means by going through all the values and seeing which
    // cluster they're closest to.  Then recompute the mean of each cluster
    // to hopefully move it to a more representative center point.
    for (int i = 0; i < k; i++) {
      totals[i] = counts[i] = 0;
    }
    for (int i = 0; i < buf->size(); i++) {
      // Determine which cluster this item is closest to
      int closest_cluster = INT_MAX;
      int closest_cluster_distance = INT_MAX;
      for (int cluster = 0; cluster < k; cluster++) {
        int distance = KmeansDistance((*buf)[i], means[cluster]); 
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
    for (int i = 0; i < k; i++) {
      int new_mean = totals[i] / counts[i];
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
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
}

////////////////////////////////////////////////////////////////////////////////
// HIGH LEVEL FUNCTIONALITY
////////////////////////////////////////////////////////////////////////////////
int ComputeSplit() {
  // Using Kmeans, find a reasonable "split" light level to determine
  // if the incoming bit is a 1 or a 0.
  int *means = (int *)malloc(2 * sizeof(int));
  Kmeans(&recv_buf, 2, means);
  int split = (means[0] + means[1]) / 2;
  free(means);
  return split;
}

int DetectHandshake(int split) {
  // First go through the working buffer and categorize each reading as a
  // zero or a one.  While doing this, the length of each pulse (all
  // contiguous ones or zeros) and store them in order, in a linked list.
  bool is_one = recv_buf[0] >= split;
  int bit_length = 0;
  LinkedList<int> pulse_lengths;
  for (int i = 0; i < recv_buf.size(); i++) {
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
  long total = 0;
  for (int i = 0; i < pulse_lengths.size(); i++) {
    total += pulse_lengths.get(i);
  }
  double avg = (double)total / (double)pulse_lengths.size();
  // Next compute the std deviation
  double deviations = 0, d;
  for (int i = 0; i < pulse_lengths.size(); i++) {
    d = (double)pulse_lengths.get(i) - avg;
    deviations += d * d;
  }
  double std_dev = sqrt(deviations / (double)pulse_lengths.size());

  for (int i = 0 ; i < pulse_lengths.size(); i++) {
    Serial.print(pulse_lengths.get(i));
    if (i != pulse_lengths.size() - 1) {
      Serial.print(", ");
    }
  }
  Serial.print("\n\r");
  Serial.print("avg: ");
  Serial.print(avg);
  Serial.print("\n\r");
  Serial.print("std dev: ");
  Serial.print(std_dev);
  Serial.print("\n\r");
  Serial.println("---");

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

void Receive(int bit_length) {
  ClearBufferAndRestartCollection();

  Serial.println(F("Aligning readings with bit boundaries in handshake..."));
  // TODO: This seems somewhat naive to just assume the first edge I see is
  // well aligned.  Perhaps if I looked for several edges I could do better?
  int reading, value = -1;
  while (reading == value || value == -1) {
    noInterrupts();
    int reading = recv_buf.shift();
    interrupts();
    if (reading == -1) {
      value = reading;
    } else if (reading == value) {

    }
  }

  Serial.println(F("Waiting to receive data..."));
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
    int split = ComputeSplit();

    // Using that "split", check to see if the log is full of equally-long
    // 0101010101 pattern.  This serves as the handshake and configures the
    // bit length.  That value is measured in the number of readings taken,
    // not directly as microseconds or something like that.
    int bit_length = DetectHandshake(split);

    // If a handshake was detected, begin listening for a message
    if (bit_length <= 1) {
      Serial.print(F(FYEL("Handshake not detected.")));
      Serial.print(F("\treason: "));
      if (bit_length == -ENOTENOUGHPULSES) {
          Serial.print(F("Not enough pulses detected."));
      } else if (bit_length = -EHIGHSTDDEV) {
          Serial.print(F("Pulse lengths varied too much."));
      } else {
          Serial.print(F("unknown ("));
          Serial.print(bit_length);
          Serial.print(F(")"));
      }
      Serial.print(F("\n\r"));
    } else {
      Serial.print(F("Bit Length = "));
      Serial.print(bit_length);
      Serial.print(F("\n\r"));
      Serial.println(F(FGRN("Handshake detected!")));
      //Receive(bit_length);
    }

    ClearBufferAndRestartCollection();
  }
}
