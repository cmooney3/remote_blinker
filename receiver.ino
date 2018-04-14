#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define LED_PIN 3
#define LIGHT_SENSOR_PIN 0
#define MAX_LIGHT_READING 1024
#define MIN_LIGHT_READING 0

#define READING_PERIOD_US 1000000
#define RECV_BUFFER_SIZE 5
#define WORK_BUFFER_SIZE 15

////////////////////////////////////////////////////////////////////////////////
// CIRCUILAR BUFFERS SETUP
////////////////////////////////////////////////////////////////////////////////
CircularBuffer<int, RECV_BUFFER_SIZE> recv_buf;
CircularBuffer<int, WORK_BUFFER_SIZE> work_buf;

////////////////////////////////////////////////////////////////////////////////
// K-MEANS TO DETECT BIMODAL LEVELS (ADJUSTING FOR LIGHT CONDITIONS)
////////////////////////////////////////////////////////////////////////////////
int* KmeansOnBuffer(int k) {
  int *means = (int *)malloc(k * sizeof(int));
  int *totals = (int *)malloc(k * sizeof(int));
  int *counts = (int *)malloc(k * sizeof(int));

  // Evenly spread out the initial k guesses
  int segment_size = (MAX_LIGHT_READING - MIN_LIGHT_READING) / (k + 1);
  for (int i = 0; i < k; i++) {
    means[i] = MIN_LIGHT_READING + segment_size * (i + 1);
  }

  // Recompute the means by going through all the values and seeing which
  // cluster they're closest to.  Then recompute the mean of each cluster
  // to hopefully move it to a more representative center point.
  for (int i = 0; i < k; i++) {
    totals[i] = counts[i] = 0;
  }
  for (int offset = 0; offset < RECV_BUFFER_SIZE; offset++) {
    // Determine which cluster this item is closest to
    int closest_cluster = INT_MAX;
    int closest_cluster_distance = INT_MAX;
    for (int cluster = 0; cluster < k; cluster++) {
      int distance = abs(recv_buf[offset] - means[cluster]); 
      if (distance < closest_cluster_distance) {
        closest_cluster = cluster; 
        closest_cluster_distance = distance;
      }
    }

    // Update the values for the closest cluster
    counts[closest_cluster]++;
    totals[closest_cluster] += recv_buf[offset];
  }

  // Having finished with all the items, we now compute the new means
  for (int i = 0; i < k; i++) {
    means[i] = totals[i] / counts[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
// PERIODIC ISR TO HANDLE TAKING LIGHT SENSOR READINGS  (Timer1)
////////////////////////////////////////////////////////////////////////////////
void TakeReading() {
  digitalWrite(LED_PIN, HIGH);
  recv_buf.push(analogRead(LIGHT_SENSOR_PIN));
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

void loop() {
  if (!recv_buf.isEmpty()) {
    for (int i = 0; i < recv_buf.size(); i++) {
      Serial.print(recv_buf[i]);
      Serial.print("\t");
    }

    while (!recv_buf.isEmpty()) {
      noInterrupts();
      work_buf.push(recv_buf.shift());
      interrupts();
    }

    Serial.print("\n\r");
    for (int i = 0; i < WORK_BUFFER_SIZE; i++) {
      Serial.print(work_buf[i]);
      Serial.print("\t");
    }
    Serial.print("\n\r");
    Serial.println("----");
  }
}
