#include <Wire.h>

namespace {
constexpr uint8_t MPU1_ADDR = 0x68;
constexpr uint8_t MPU2_ADDR = 0x69;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_WHO_AM_I = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float ACCEL_LSB_PER_G = 8192.0f;
constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr uint32_t BAUD_RATE = 230400;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
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

struct BusScanResult {
  uint8_t foundCount = 0;
  bool found68 = false;
  bool found69 = false;
};

GyroBias g_bias1;
GyroBias g_bias2;
uint32_t g_lastSampleMs = 0;

void printHexAddress(uint8_t addr) {
  Serial.print(F("0x"));
  if (addr < 0x10) {
    Serial.print('0');
  }
  Serial.print(addr, HEX);
}

void printHexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
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

bool readRegister(uint8_t addr, uint8_t reg, uint8_t& value) {
  return readRegisters(addr, reg, &value, 1);
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
  if (!writeRegister(addr, REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);
  if (!writeRegister(addr, REG_SMPLRT_DIV, 0x09)) {
    return false;
  }
  if (!writeRegister(addr, REG_CONFIG, 0x03)) {
    return false;
  }
  if (!writeRegister(addr, REG_GYRO_CONFIG, 0x08)) {
    return false;
  }
  if (!writeRegister(addr, REG_ACCEL_CONFIG, 0x08)) {
    return false;
  }

  RawMpuFrame verify;
  return readRawFrame(addr, verify);
}

BusScanResult scanI2CBus() {
  BusScanResult result;
  Serial.println(F("# i2c_scan_begin"));
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t error = Wire.endTransmission(true);
    if (error == 0) {
      ++result.foundCount;
      result.found68 = result.found68 || (addr == MPU1_ADDR);
      result.found69 = result.found69 || (addr == MPU2_ADDR);
      Serial.print(F("# i2c_found="));
      printHexAddress(addr);
      if (addr == MPU1_ADDR) {
        Serial.print(F(" expected_mpu1"));
      } else if (addr == MPU2_ADDR) {
        Serial.print(F(" expected_mpu2"));
      }
      Serial.println();
    } else if (error == 4) {
      Serial.print(F("# i2c_unknown_error="));
      printHexAddress(addr);
      Serial.println();
    }
  }

  Serial.print(F("# i2c_scan_end count="));
  Serial.println(result.foundCount);
  Serial.print(F("# expected_0x68="));
  Serial.println(result.found68 ? F("found") : F("missing"));
  Serial.print(F("# expected_0x69="));
  Serial.println(result.found69 ? F("found") : F("missing"));
  return result;
}

void probeWhoAmI(uint8_t addr) {
  uint8_t whoAmI = 0;
  Serial.print(F("# whoami_addr="));
  printHexAddress(addr);
  if (!readRegister(addr, REG_WHO_AM_I, whoAmI)) {
    Serial.println(F(" result=read_fail"));
    return;
  }

  Serial.print(F(" result="));
  printHexByte(whoAmI);
  if (whoAmI == MPU6050_WHO_AM_I) {
    Serial.println(F(" status=mpu6050_signature"));
    return;
  }

  Serial.println(F(" status=unexpected_device"));
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
  Wire.setClock(I2C_CLOCK_HZ);
  Serial.begin(BAUD_RATE);
  delay(1000);

  Serial.println(F("# startup_begin"));
  Serial.print(F("# i2c_clock_hz="));
  Serial.println(I2C_CLOCK_HZ);
  const BusScanResult scanResult = scanI2CBus();
  probeWhoAmI(MPU1_ADDR);
  probeWhoAmI(MPU2_ADDR);
  if (scanResult.found68 && !scanResult.found69) {
    Serial.println(F("# hint_only_0x68=second MPU is probably still at 0x68"));
    Serial.println(F("# hint_check_ad0=tie MPU2 AD0/ADO firmly to VCC; floating is not enough"));
    Serial.println(F("# hint_same_addr=two MPU6050 boards cannot share the same I2C address"));
  }

  const bool ok1 = initMpu(MPU1_ADDR);
  const bool ok2 = initMpu(MPU2_ADDR);
  Serial.print(F("# init_addr="));
  printHexAddress(MPU1_ADDR);
  Serial.print(F(" result="));
  Serial.println(ok1 ? F("ok") : F("fail"));
  Serial.print(F("# init_addr="));
  printHexAddress(MPU2_ADDR);
  Serial.print(F(" result="));
  Serial.println(ok2 ? F("ok") : F("fail"));

  if (!ok1 || !ok2) {
    Serial.println(F("# startup_halted"));
    if (!ok1) {
      Serial.println(F("# check_0x68 AD0->GND SDA->20 SCL->21 VCC/GND"));
    }
    if (!ok2) {
      Serial.println(F("# check_0x69 AD0->VCC SDA->20 SCL->21 VCC/GND"));
    }
    while (true) {
      delay(1000);
    }
  }

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
