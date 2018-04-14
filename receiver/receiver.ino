#include <CircularBuffer.h>
#include <limits.h>
#include <TimerOne.h>

#define ERROR_LED_PIN 4
#define LIGHT_SENSOR_PIN 0

#define READING_PERIOD_US 1000
#define RECV_BUFFER_SIZE 100
#define WORK_BUFFER_SIZE 500

////////////////////////////////////////////////////////////////////////////////
// CIRCUILAR BUFFERS SETUP
////////////////////////////////////////////////////////////////////////////////
CircularBuffer<int, RECV_BUFFER_SIZE> recv_buf;
CircularBuffer<int, WORK_BUFFER_SIZE> work_buf;

////////////////////////////////////////////////////////////////////////////////
// K-MEANS TO DETECT BIMODAL LEVELS (ADJUSTING FOR LIGHT CONDITIONS)
////////////////////////////////////////////////////////////////////////////////
int KmeansDistance(int v1, int v2) {
  return abs(v1 - v2);
}

void KmeansSelectStartingCentroids(int k, int* means) {
  // Select k random starting centroids as the initial means.  The subsequent
  // centroids are weighted to not be near the initial ones.
  means[0] = work_buf[random(work_buf.size())];
  for (int cluster = 1; cluster < k; cluster++) {
    long total_weights = 0;

    for (int i = 0; i < work_buf.size(); i++) {
      int closest_distance = INT_MAX;
      int d;
      for (int existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(work_buf[i], means[existing_cluster]); 
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights += closest_distance * closest_distance;
    }

    int selection = random(total_weights);

    for (int i = 0; i < work_buf.size(); i++) {
      int closest_distance = INT_MAX;
      int d;
      for (int existing_cluster = 0; existing_cluster < cluster; existing_cluster++) {
        d = KmeansDistance(work_buf[i], means[existing_cluster]); 
        if (d < closest_distance) {
          closest_distance = d;
        }
      }
      total_weights -= closest_distance * closest_distance;
      if (total_weights <= selection) {
        means[cluster] = work_buf[i];
        break;
      }
    }
  }
}

void KmeansOnWorkBuffer(int k, int *means) {
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
    for (int i = 0; i < work_buf.size(); i++) {
      // Determine which cluster this item is closest to
      int closest_cluster = INT_MAX;
      int closest_cluster_distance = INT_MAX;
      for (int cluster = 0; cluster < k; cluster++) {
        int distance = KmeansDistance(work_buf[i], means[cluster]); 
        if (distance < closest_cluster_distance) {
          closest_cluster = cluster; 
          closest_cluster_distance = distance;
        }
      }

      // Update the values for the closest cluster
      counts[closest_cluster]++;
      totals[closest_cluster] += work_buf[i];
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

  Serial.print("Selected centroids: ");
  for (int i = 0; i < k; i++) {
    Serial.print(means[i]);
    Serial.print("\t");
  }
  Serial.print("\n\r");

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

void setup() {
  Serial.begin(115200);

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, LOW);

  Timer1.initialize();
  Timer1.setPeriod(READING_PERIOD_US);
  Timer1.attachInterrupt(TakeReading); 
}

void DrainReceiveBuffer() {
  while (!recv_buf.isEmpty()) {
    noInterrupts();
    work_buf.push(recv_buf.shift());
    interrupts();
  }
}

int ComputeSplitFromWorkBuffer() {
  int *means = (int *)malloc(2 * sizeof(int));
  KmeansOnWorkBuffer(2, means);
  int split = (means[0] + means[1]) / 2;
  free(means);
  return split;
}

void loop() {
  if (!recv_buf.isEmpty()) {
    DrainReceiveBuffer();

    if (work_buf.isFull()) {
      Serial.print("\n\r");
      for (int i = 0; i < WORK_BUFFER_SIZE; i++) {
        Serial.print(work_buf[i]);
        Serial.print("\t");
      }
      Serial.print("\n\r");

      int split = ComputeSplitFromWorkBuffer();
      Serial.print("split: ");
      Serial.print(split);
      Serial.print("\n\r");

      Serial.println("----");
      work_buf.clear();
    }
  }
}
