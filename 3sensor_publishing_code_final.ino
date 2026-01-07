#include <micro_ros_arduino.h>
#include <stdlib.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

// ================= HARDWARE =================
#include <HardwareSerial.h>
#include <Wire.h>
#include "SparkFun_AS7265X.h"

#define LED_PIN 13

// ---------- NPK (RS485) ----------
#define RE 18
#define DE 19
HardwareSerial mod(2); // RX=16, TX=17

const byte nitro[] = {0x01,0x03,0x00,0x1e,0x00,0x01,0xe4,0x0c};
const byte phos[]  = {0x01,0x03,0x00,0x1f,0x00,0x01,0xb5,0xcc};
const byte pota[]  = {0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xc0};

byte values[11];

// ---------- GAS SENSOR (I2C) ----------
#define I2C_ADDR 0x08
#define CMD_GET_CONCENTRATION 0x04
#define CMD_GET_VOLTAGE 0x05

// ---------- SPECTRAL ----------
AS7265X sensor;

// ================= micro-ROS =================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t pub_npk;
rcl_publisher_t pub_gas;
rcl_publisher_t pub_spectral;

rcl_timer_t timer;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray msg_npk;
std_msgs__msg__Float32MultiArray msg_gas;
std_msgs__msg__Float32MultiArray msg_spectral;

// ---------- error handler ----------
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// =======================================================
// ================= SENSOR FUNCTIONS =====================
// =======================================================

byte requestData(const byte cmd[]) {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);

  mod.write(cmd, 8);
  mod.flush();

  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  long t = millis();
  while (mod.available() < 7) {
    if (millis() - t > 2000) return 255;
    delay(10);
  }

  for (int i = 0; i < 7; i++) values[i] = mod.read();
  return values[4];
}

byte nitrogen()    { return requestData(nitro); }
byte phosphorous() { return requestData(phos); }
byte potassium()   { return requestData(pota); }

float readGasPPM(int gasIndex) {
  Wire1.beginTransmission(I2C_ADDR);
  Wire1.write(CMD_GET_CONCENTRATION);
  Wire1.write(gasIndex);
  if (Wire1.endTransmission() != 0) return -1;

  delay(50);
  if (Wire1.requestFrom(I2C_ADDR, 2) != 2) return -1;

  uint16_t raw = (Wire1.read() << 8) | Wire1.read();
  return (float)raw;
}

// =======================================================
// ================= TIMER CALLBACK =======================
// =======================================================

void timer_callback(rcl_timer_t *timer, int64_t last_call) {
  (void)timer;
  (void)last_call;

  // ---------- NPK ----------
  static float npk_data[3];
  npk_data[0] = nitrogen();
  npk_data[1] = phosphorous();
  npk_data[2] = potassium();

  msg_npk.data.data = npk_data;
  msg_npk.data.size = 3;
  msg_npk.data.capacity = 3;
  rcl_publish(&pub_npk, &msg_npk, NULL);

  // ---------- GAS ----------
  static float gas_data[8];
  for (int i = 1; i <= 8; i++)
    gas_data[i-1] = readGasPPM(i);

  msg_gas.data.data = gas_data;
  msg_gas.data.size = 8;
  msg_gas.data.capacity = 8;
  rcl_publish(&pub_gas, &msg_gas, NULL);

  // ---------- SPECTRAL ----------
  sensor.takeMeasurements();
  static float spec_data[18] = {
    sensor.getA(), sensor.getB(), sensor.getC(), sensor.getD(),
    sensor.getE(), sensor.getF(), sensor.getG(), sensor.getH(),
    sensor.getR(), sensor.getI(), sensor.getS(), sensor.getJ(),
    sensor.getT(), sensor.getU(), sensor.getV(), sensor.getW(),
    sensor.getK(), sensor.getL()
  };

  msg_spectral.data.data = spec_data;
  msg_spectral.data.size = 18;
  msg_spectral.data.capacity = 18;
  rcl_publish(&pub_spectral, &msg_spectral, NULL);
}

// =======================================================
// ===================== SETUP ============================
// =======================================================

void setup() {

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  // 1️⃣ Transport FIRST
  set_microros_transports();

  // 2️⃣ Wait for agent
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);

  // 3️⃣ Hardware init
  mod.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  digitalWrite(RE, LOW);
  digitalWrite(DE, LOW);

  Wire1.begin(25, 26);

  if (!sensor.begin()) error_loop();
  Wire.setClock(400000);
  sensor.disableIndicator();

  // 4️⃣ micro-ROS init (ONCE)
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_sensor_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &pub_npk, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "npk"));

  RCCHECK(rclc_publisher_init_default(
    &pub_gas, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "gas"));

  RCCHECK(rclc_publisher_init_default(
    &pub_spectral, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "spectral"));

  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(1000),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

// =======================================================
// ===================== LOOP =============================
// =======================================================

void loop() {

  if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
