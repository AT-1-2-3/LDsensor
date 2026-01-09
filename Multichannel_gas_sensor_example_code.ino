#include <Wire.h>

#define I2C_ADDR 0x08   // Default I2C address

#define CMD_GET_CONCENTRATION 0x04
#define CMD_GET_VOLTAGE       0x05

// List of gases (adjust depending on your sensor cartridge)
const char* gasNames[] = {
  "None", "CO", "NO2", "NH3", "C3H8", "C4H10", "CH4", "H2", "C2H5OH"
};

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL
  delay(1000);
  Serial.println("Multichannel Gas Sensor v2.0 (Voltages + PPM)");
}

void loop() {
  // ---- Print electrode voltages ----
  Serial.println("Electrode Voltages:");
  for (int ch = 1; ch <= 4; ch++) {
    float voltage = readChannelVoltage(ch);
    if (voltage >= 0) {
      Serial.printf("  CH%d: %.3f V (%.1f mV)\n", ch, voltage, voltage * 1000);
    } else {
      Serial.printf("  CH%d: Error reading\n", ch);
    }
  }

  // ---- Print gas concentrations ----
  Serial.println("Gas Concentrations (ppm):");
  for (int gas = 1; gas <= 8; gas++) {
    float ppm = readGasPPM(gas);
    if (ppm >= 0) {
      Serial.printf("  %s: %.3f ppm\n", gasNames[gas], ppm);
    } else {
      Serial.printf("  %s: Error reading\n", gasNames[gas]);
    }
  }

  Serial.println("------------------------------\n");
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
  return raw / 1000.0;  // mV → Vy

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
