#include <Wire.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define I2C_ADDR 0x08   // Default I2C address

#define CMD_GET_CONCENTRATION 0x04
#define CMD_GET_VOLTAGE       0x05

// List of gases (adjust depending on your sensor cartridge)
const char* gasNames[] = {
  "None", "CO", "NO2", "NH3", "C3H8", "C4H10", "CH4", "H2", "C2H5OH"
};

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Buffer for message data
char msg_buffer[512];

void setup() {
  // Init micro-ROS transport
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "gas_sensor_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gas");

  // Initialize message memory
  msg.data.data = NULL;
  msg.data.size = 0;
  msg.data.capacity = 0;

  // Initialize I2C
  Wire.begin(21, 22); // SDA, SCL (ESP32 default pins)
}

void loop() {
  // Build message string
  String data = "Gas Concentrations (ppm):\n";
  for (int gas = 1; gas <= 8; gas++) {
    float ppm = readGasPPM(gas);
    if (ppm >= 0) {
      data += String(gasNames[gas]) + ": " + String(ppm, 3) + " ppm\n";
    } else {
      data += String(gasNames[gas]) + ": Error\n";
    }
  }

  // Copy to buffer and publish
  data.toCharArray(msg_buffer, sizeof(msg_buffer));
  msg.data.data = msg_buffer;
  msg.data.size = strlen(msg_buffer);
  msg.data.capacity = sizeof(msg_buffer);

  rcl_publish(&publisher, &msg, NULL);

  delay(2000); // 2 sec interval
}

// ---------------- Helper functions ----------------

float readChannelVoltage(int channel) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(CMD_GET_VOLTAGE);
  Wire.write(channel);
  if (Wire.endTransmission() != 0) return -1; // error

  delay(50);
  if (Wire.requestFrom(I2C_ADDR, 2) != 2) return -1;

  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 1000.0;  // mV → V
}

float readGasPPM(int gasIndex) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(CMD_GET_CONCENTRATION);
  Wire.write(gasIndex);
  if (Wire.endTransmission() != 0) return -1;

  delay(50);
  if (Wire.requestFrom(I2C_ADDR, 2) != 2) return -1;

  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (float)raw;  // already in ppm
}
