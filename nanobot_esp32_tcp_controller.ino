#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>

// =============================================================================
// NETWORK & PORT CONFIGURATION
// =============================================================================
const char* WIFI_SSID = "DIR-806A-C4FA";
const char* WIFI_PASS = "0108083659";

const uint16_t ARM_TCP_PORT     = 8890; // PC -> ESP32 -> Arduino: "a1,a2,a3\n" (3 arm angles)
const uint16_t GRIPPER_TCP_PORT = 8891; // PC -> ESP32 -> Arduino: "g1\n"       (1 gripper angle)
const uint16_t BASE_TCP_PORT    = 8892; // PC <-> ESP32: in "v,w\n", out "left_w,right_w\n"

// =============================================================================
// HARDWARE PIN & KINEMATICS CONFIGURATION
// =============================================================================
#define L298N_IN1 12
#define L298N_IN2 13
#define L298N_IN3 14
#define L298N_IN4 27

#define L298N_ENA 2  // Left Motor PWM
#define L298N_ENB 4  // Right Motor PWM

#define ARDUINO_RX_PIN 16
#define ARDUINO_TX_PIN 17

#define RIGHT_ENCA_PIN 22
#define RIGHT_ENCB_PIN 23

#define LEFT_ENCA_PIN  34
#define LEFT_ENCB_PIN  25

HardwareSerial ArduinoSerial(2);
SemaphoreHandle_t serialMutex;

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

const float WHEEL_SEPARATION = 0.233;
const float WHEEL_RADIUS     = 0.0325;
const float TICKS_PER_REV    = 717.0;

// =============================================================================
// REUSABLE SINGLE-CLIENT LINE-BUFFERED TCP SERVER
// =============================================================================
struct TcpLineServer {
  WiFiServer server;
  WiFiClient client;
  const char* name;
  char rx_buf[128];
  size_t rx_len = 0;

  TcpLineServer(uint16_t port, const char* label)
    : server(port), name(label) {}

  void begin() {
    server.begin();
    server.setNoDelay(true);
    Serial.printf("[%s] TCP server listening\n", name);
  }

  void acceptClients() {
    if (server.hasClient()) {
      if (!client || !client.connected()) {
        if (client) client.stop();
        client = server.available();
        client.setNoDelay(true);
        rx_len = 0;
        Serial.printf("[%s] Client connected\n", name);
      } else {
        WiFiClient extra = server.available();
        extra.stop();
      }
    }
  }

  template <typename Callback>
  void pollLines(Callback onLine) {
    acceptClients();
    if (!client || !client.connected()) return;

    while (client.available() > 0) {
      char c = (char)client.read();
      if (c == '\n') {
        rx_buf[rx_len] = '\0';
        if (rx_len > 0) {
          onLine(rx_buf);
        }
        rx_len = 0;
      } else if (c != '\r') {
        if (rx_len < sizeof(rx_buf) - 1) {
          rx_buf[rx_len++] = c;
        } else {
          rx_len = 0;
        }
      }
    }
  }

  void sendLine(const String& msg) {
    if (client && client.connected()) {
      client.print(msg);
      if (!msg.endsWith("\n")) {
        client.print('\n');
      }
    }
  }
};

TcpLineServer armTcp(ARM_TCP_PORT, "ARM");
TcpLineServer gripperTcp(GRIPPER_TCP_PORT, "GRIPPER");
TcpLineServer baseTcp(BASE_TCP_PORT, "BASE");

// =============================================================================
// PARSING HELPER FOR BASE KINEMATICS ("v,w")
// =============================================================================
size_t parseCsvFloats(const char* line, float* out, size_t max_count) {
  char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  size_t count = 0;
  char* token = strtok(buf, ",");
  while (token != nullptr && count < max_count) {
    out[count++] = strtof(token, nullptr);
    token = strtok(nullptr, ",");
  }
  return count;
}

// =============================================================================
// ENCODERS, MOTORS & PID
// =============================================================================
volatile long left_wheel_ticks = 0;
volatile long right_wheel_ticks = 0;

unsigned long last_encoder_time = 0;
const unsigned long CALC_INTERVAL = 100; // 10 Hz

unsigned long last_pid_time = 0;
const unsigned long PID_INTERVAL = 100;

unsigned long last_base_tx_time = 0;
const unsigned long BASE_TX_INTERVAL = 50; // 20 Hz wheel feedback to PC

struct PIDState {
  float prev_error = 0.0;
  float integral = 0.0;
};

PIDState left_pid;
PIDState right_pid;

float Kv = 10.9303718369895;
int Bv = 152.718267291324;
float Kp = 0.0;
float Ki = 10.0;
float Kd = 0.0;

int MIN_PWM = 150;
int PWM_MAX = 220;

volatile float target_left_rad_s = 0.0;
volatile float target_right_rad_s = 0.0;
volatile float latest_left_rad_s = 0.0;
volatile float latest_right_rad_s = 0.0;

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENCB_PIN) == LOW) left_wheel_ticks++;
  else left_wheel_ticks--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENCB_PIN) == LOW) right_wheel_ticks++;
  else right_wheel_ticks--;
}

void setupEncoders() {
  pinMode(LEFT_ENCA_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCB_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCA_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCB_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA_PIN), rightEncoderISR, RISING);
}

void stopMotors() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
  ledcWrite(L298N_ENA, 0);
  ledcWrite(L298N_ENB, 0);
}

void setupMotors() {
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);

  ledcAttach(L298N_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENB, PWM_FREQ, PWM_RES);
  stopMotors();
}

void setMotorChannel(int speed, int pwm_pin, int in_a, int in_b) {
  int pwm_val = constrain(abs(speed), 0, 255);
  ledcWrite(pwm_pin, pwm_val);

  if (speed > 0) {
    digitalWrite(in_a, HIGH);
    digitalWrite(in_b, LOW);
  } else if (speed < 0) {
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, HIGH);
  } else {
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, LOW);
  }
}

void setLeftMotor(int speed)  { setMotorChannel(speed, L298N_ENA, L298N_IN1, L298N_IN2); }
void setRightMotor(int speed) { setMotorChannel(speed, L298N_ENB, L298N_IN3, L298N_IN4); }

int computePID(float target_rad_s, float actual_rad_s, PIDState &pid, float dt_sec) {
  if (fabsf(target_rad_s) < 0.05f) {
    pid.prev_error = 0.0f;
    return 0;
  }

  float feed_forward_term = Kv * target_rad_s;
  float error = target_rad_s - actual_rad_s;
  float p_term = Kp * error;

  pid.integral = constrain(pid.integral + (Ki * error * dt_sec), -255.0f, 255.0f);
  float i_term = pid.integral;

  float d_term = (dt_sec > 0.0f) ? (Kd * ((error - pid.prev_error) / dt_sec)) : 0.0f;
  pid.prev_error = error;

  float total_output = feed_forward_term + p_term + i_term + d_term;
  int target_sign = (target_rad_s >= 0.0f) ? 1 : -1;
  int unified_output = constrain((int)(Bv + total_output * target_sign), MIN_PWM, PWM_MAX);

  return target_sign * unified_output;
}

void calculateWheelVelocities(float &left_rad_s, float &right_rad_s) {
  unsigned long now = millis();
  unsigned long dt_ms = now - last_encoder_time;

  if (dt_ms >= CALC_INTERVAL) {
    noInterrupts();
    long cur_left = left_wheel_ticks;
    long cur_right = right_wheel_ticks;
    left_wheel_ticks = 0;
    right_wheel_ticks = 0;
    interrupts();

    float dt_sec = dt_ms / 1000.0f;
    left_rad_s  = (((float)cur_left / TICKS_PER_REV) * 2.0f * PI) / dt_sec;
    right_rad_s = ((((float)cur_right / TICKS_PER_REV) * 2.0f * PI) / dt_sec) * -1.0f;
    last_encoder_time = now;
  }
}

// =============================================================================
// DIRECT ARDUINO SERIAL FORWARDER & BASE COMMAND HANDLER
// =============================================================================
void forwardRawLineToArduino(const char* line) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    ArduinoSerial.println(line);
    xSemaphoreGive(serialMutex);
  }
}

void onArmLine(const char* line) {
  // Direct pass-through of "a1,a2,a3" to Arduino
  forwardRawLineToArduino(line);
}

void onGripperLine(const char* line) {
  // Direct pass-through of "g1" to Arduino
  forwardRawLineToArduino(line);
}

void onBaseLine(const char* line) {
  // Expects 2 comma-separated velocities: "linear_x,angular_z"
  float vals[2] = {0.0f, 0.0f};
  if (parseCsvFloats(line, vals, 2) < 2) return;

  float linear = vals[0];
  float angular = vals[1];

  float left_m_s  = linear - (angular * WHEEL_SEPARATION / 2.0f);
  float right_m_s = linear + (angular * WHEEL_SEPARATION / 2.0f);

  target_left_rad_s  = left_m_s / WHEEL_RADIUS;
  target_right_rad_s = right_m_s / WHEEL_RADIUS;
}

// =============================================================================
// SETUP & LOOP
// =============================================================================
void setup() {
  serialMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);

  setupMotors();
  setupEncoders();

  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.print("Wi-Fi connected! ESP32 IP: ");
  Serial.println(WiFi.localIP());

  armTcp.begin();
  gripperTcp.begin();
  baseTcp.begin();
}

void loop() {
  // 1. Service all 3 TCP servers without blocking
  armTcp.pollLines(onArmLine);
  gripperTcp.pollLines(onGripperLine);
  baseTcp.pollLines(onBaseLine);

  // 2. Run wheel velocity calculation + PID motor control
  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL) {
    float dt_sec = (now - last_pid_time) / 1000.0f;
    last_pid_time = now;

    float actual_left = latest_left_rad_s;
    float actual_right = latest_right_rad_s;
    calculateWheelVelocities(actual_left, actual_right);

    int left_pwm  = computePID(target_left_rad_s, actual_left, left_pid, dt_sec);
    int right_pwm = computePID(target_right_rad_s, actual_right, right_pid, dt_sec);

    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);

    latest_left_rad_s = actual_left;
    latest_right_rad_s = actual_right;
  }

  // 3. Stream 2 comma-separated wheel velocities back to PC over Base TCP at 20 Hz
  if (now - last_base_tx_time >= BASE_TX_INTERVAL) {
    last_base_tx_time = now;
    baseTcp.sendLine(
      String(latest_left_rad_s, 4) + "," + String(latest_right_rad_s, 4)
    );
  }
}