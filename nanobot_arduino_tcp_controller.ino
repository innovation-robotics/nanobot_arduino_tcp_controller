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

struct ParsedCommand {
  const char* elements[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
};

void applyAngles() {
  for (int i = 0; i < 4; i++) {
    int clamped = constrain(current_angle[i], 0, 180);
    servos[i].write(clamped);
  }
}

void applyAngleToken(const char* token, int servo_idx) {
  if (token == nullptr || token[0] == '\0') return;
  int delta_deg = (int)lroundf(strtod(token, nullptr));
  target_angle[servo_idx] = constrain(offset[servo_idx] + delta_deg, 0, 180);
}

ParsedCommand parseElements(char* buf, size_t buf_size, const char* line) {
  ParsedCommand cmd;
  if (line == nullptr || line[0] == '\0' || buf == nullptr || buf_size == 0) {
    return cmd;
  }

  strncpy(buf, line, buf_size - 1);
  buf[buf_size - 1] = '\0';

  char* tok = strtok(buf, ",");
  while (tok != nullptr && cmd.count < 4) {
    cmd.elements[cmd.count++] = tok;
    tok = strtok(nullptr, ",");
  }

  return cmd;
}

void parseCommand(const char* line) {
  char copy[64];
  ParsedCommand cmd = parseElements(copy, sizeof(copy), line);

  if (cmd.count < 2 || cmd.elements[0] == nullptr) return;

  if (strcmp(cmd.elements[0], "a") == 0) {
    int angle_count = cmd.count - 1;
    if (angle_count > 3) angle_count = 3;
    for (int i = 0; i < angle_count; i++) {
      applyAngleToken(cmd.elements[1 + i], i);
    }
  } else if (strcmp(cmd.elements[0], "g") == 0) {
    applyAngleToken(cmd.elements[1], 3);
  }
}

void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line_buf[buf_idx] = '\0';
      parseCommand(line_buf);
      // Serial.println(line_buf);
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