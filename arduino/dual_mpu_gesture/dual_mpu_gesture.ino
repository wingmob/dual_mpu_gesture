#include <Wire.h>

namespace {
constexpr uint8_t MPU1_ADDR = 0x68;
constexpr uint8_t MPU2_ADDR = 0x69;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_LSB_PER_G = 8192.0f;
constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr uint32_t BAUD_RATE = 230400;
constexpr uint16_t SAMPLE_PERIOD_MS = 10;
constexpr uint16_t CALIBRATION_SAMPLES = 500;

struct RawMpuFrame {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct GyroBias {
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

GyroBias g_bias1;
GyroBias g_bias2;
uint32_t g_lastSampleMs = 0;

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readRegisters(uint8_t addr, uint8_t startReg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t readCount = Wire.requestFrom(addr, length, true);
  if (readCount != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool readRawFrame(uint8_t addr, RawMpuFrame& frame) {
  uint8_t buffer[14];
  if (!readRegisters(addr, REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
    return false;
  }

  frame.ax = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
  frame.ay = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
  frame.az = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];
  frame.gx = (static_cast<int16_t>(buffer[8]) << 8) | buffer[9];
  frame.gy = (static_cast<int16_t>(buffer[10]) << 8) | buffer[11];
  frame.gz = (static_cast<int16_t>(buffer[12]) << 8) | buffer[13];
  return true;
}

bool initMpu(uint8_t addr) {
  writeRegister(addr, REG_PWR_MGMT_1, 0x00);
  delay(50);
  writeRegister(addr, REG_SMPLRT_DIV, 0x09);
  writeRegister(addr, REG_CONFIG, 0x03);
  writeRegister(addr, REG_GYRO_CONFIG, 0x08);
  writeRegister(addr, REG_ACCEL_CONFIG, 0x08);

  RawMpuFrame verify;
  return readRawFrame(addr, verify);
}

GyroBias calibrateGyroBias(uint8_t addr) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  RawMpuFrame frame;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readRawFrame(addr, frame)) {
      sumX += frame.gx;
      sumY += frame.gy;
      sumZ += frame.gz;
    }
    delay(3);
  }

  GyroBias bias;
  bias.gx = static_cast<float>(sumX) / CALIBRATION_SAMPLES;
  bias.gy = static_cast<float>(sumY) / CALIBRATION_SAMPLES;
  bias.gz = static_cast<float>(sumZ) / CALIBRATION_SAMPLES;
  return bias;
}

void emitHeader() {
  Serial.println(F("# dual_mpu_gesture"));
  Serial.print(F("# baud="));
  Serial.println(BAUD_RATE);
  Serial.print(F("# sample_rate_hz="));
  Serial.println(100);
  Serial.print(F("# accel_lsb_per_g="));
  Serial.println(ACCEL_LSB_PER_G, 1);
  Serial.print(F("# gyro_lsb_per_dps="));
  Serial.println(GYRO_LSB_PER_DPS, 1);
  Serial.println(F("ts_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2"));
}

void recalibrate() {
  Serial.println(F("# recalibration_started"));
  delay(1000);
  g_bias1 = calibrateGyroBias(MPU1_ADDR);
  g_bias2 = calibrateGyroBias(MPU2_ADDR);
  Serial.println(F("# recalibration_done"));
}

int16_t applyBias(int16_t raw, float bias) {
  return static_cast<int16_t>(raw - bias);
}
}  // namespace

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(BAUD_RATE);
  delay(1000);

  const bool ok1 = initMpu(MPU1_ADDR);
  const bool ok2 = initMpu(MPU2_ADDR);
  Serial.print(F("# mpu1="));
  Serial.println(ok1 ? F("ok") : F("fail"));
  Serial.print(F("# mpu2="));
  Serial.println(ok2 ? F("ok") : F("fail"));

  Serial.println(F("# keep the arm still for gyro calibration"));
  delay(1500);
  g_bias1 = calibrateGyroBias(MPU1_ADDR);
  g_bias2 = calibrateGyroBias(MPU2_ADDR);
  emitHeader();
  g_lastSampleMs = millis();
}

void loop() {
  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C') {
      recalibrate();
      emitHeader();
    }
  }

  const uint32_t now = millis();
  if (now - g_lastSampleMs < SAMPLE_PERIOD_MS) {
    return;
  }
  g_lastSampleMs = now;

  RawMpuFrame frame1;
  RawMpuFrame frame2;
  const bool ok1 = readRawFrame(MPU1_ADDR, frame1);
  const bool ok2 = readRawFrame(MPU2_ADDR, frame2);
  if (!ok1 || !ok2) {
    Serial.println(F("# read_error"));
    delay(10);
    return;
  }

  Serial.print(now);
  Serial.print(',');
  Serial.print(frame1.ax);
  Serial.print(',');
  Serial.print(frame1.ay);
  Serial.print(',');
  Serial.print(frame1.az);
  Serial.print(',');
  Serial.print(applyBias(frame1.gx, g_bias1.gx));
  Serial.print(',');
  Serial.print(applyBias(frame1.gy, g_bias1.gy));
  Serial.print(',');
  Serial.print(applyBias(frame1.gz, g_bias1.gz));
  Serial.print(',');
  Serial.print(frame2.ax);
  Serial.print(',');
  Serial.print(frame2.ay);
  Serial.print(',');
  Serial.print(frame2.az);
  Serial.print(',');
  Serial.print(applyBias(frame2.gx, g_bias2.gx));
  Serial.print(',');
  Serial.print(applyBias(frame2.gy, g_bias2.gy));
  Serial.print(',');
  Serial.println(applyBias(frame2.gz, g_bias2.gz));
}
