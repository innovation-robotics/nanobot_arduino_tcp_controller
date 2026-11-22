#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Servo servos[4];
const int servo_pins[4] = {3, 5, 6, 9};
const int offset[4] = {90, 150, 150, 177};

int current_angle[4];
int target_angle[4];

char line_buf[64];
size_t buf_idx = 0;
unsigned long last_step_ms = 0;
const unsigned long STEP_INTERVAL_MS = 15; // ~66 Hz step rate
const int MAX_STEP_DEG = 1;               // smaller = smoother/slower, larger = faster

void applyAngles() {
  for (int i = 0; i < 4; i++) {
    int clamped = constrain(current_angle[i], 0, 180);
    servos[i].write(clamped);
  }
}

int countChar(const char* str, char target) {
  int count = 0;
  for (size_t i = 0; str[i] != '\0'; i++) {
    if (str[i] == target) count++;
  }
  return count;
}

void parseCommand(const char* line) {
  if (line == nullptr || line[0] == '\0') return;

  char copy[64];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';

  int commas = countChar(copy, ',');

  // 3 comma-separated arm angles: "d1,d2,d3"
  if (commas == 2) {
    int joint_idx = 0;
    char* token = strtok(copy, ",");
    while (token != nullptr && joint_idx < 3) {
      int delta_deg = (int)lroundf(strtof(token, nullptr));
      int angle = offset[joint_idx] + delta_deg;
      target_angle[joint_idx] = constrain(angle, 0, 180);
      joint_idx++;
      token = strtok(nullptr, ",");
    }
  }
  // 1 gripper angle value: "g1"
  else if (commas == 0) {
    int delta_deg = (int)lroundf(strtof(copy, nullptr));
    int angle = offset[3] + delta_deg;
    target_angle[3] = constrain(angle, 0, 180);
  }
}

void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line_buf[buf_idx] = '\0';
      parseCommand(line_buf);
      buf_idx = 0;
    } else if (c != '\r' && buf_idx < sizeof(line_buf) - 1) {
      line_buf[buf_idx++] = c;
    } else if (buf_idx >= sizeof(line_buf) - 1) {
      buf_idx = 0; // reset on overflow
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    servos[i].attach(servo_pins[i]);
    current_angle[i] = constrain(offset[i], 0, 180);
    target_angle[i] = current_angle[i];
  }
  delay(1000);
  applyAngles();
}

void loop() {
  readSerialNonBlocking();

  unsigned long now = millis();
  if (now - last_step_ms >= STEP_INTERVAL_MS) {
    last_step_ms = now;
    bool changed = false;
    for (int i = 0; i < 4; i++) {
      int diff = target_angle[i] - current_angle[i];
      if (diff != 0) {
        int delta = constrain(diff, -MAX_STEP_DEG, MAX_STEP_DEG);
        current_angle[i] += delta;
        changed = true;
      }
    }
    if (changed) {
      applyAngles();
    }
  }
}