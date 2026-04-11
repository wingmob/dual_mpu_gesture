#include <Wire.h>

namespace {
constexpr uint8_t MPU_ADDR_LOW = 0x68;
constexpr uint8_t MPU_ADDR_HIGH = 0x69;
constexpr uint8_t MPU6050_WHO_AM_I = 0x68;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;

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

uint8_t g_mpuAddr = 0;
GyroBias g_bias;
uint32_t g_lastSampleMs = 0;

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

  uint8_t readCount = Wire.requestFrom(
      static_cast<uint8_t>(addr),
      static_cast<uint8_t>(length),
      static_cast<uint8_t>(true));
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
    uint8_t error = Wire.endTransmission(true);
    if (error == 0) {
      ++result.foundCount;
      result.found68 = result.found68 || (addr == MPU_ADDR_LOW);
      result.found69 = result.found69 || (addr == MPU_ADDR_HIGH);
      Serial.print(F("# i2c_found="));
      printHexByte(addr);
      if (addr == MPU_ADDR_LOW || addr == MPU_ADDR_HIGH) {
        Serial.print(F(" expected_mpu"));
      }
      Serial.println();
    } else if (error == 4) {
      Serial.print(F("# i2c_unknown_error="));
      printHexByte(addr);
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

uint8_t chooseMpuAddress(const BusScanResult& scanResult) {
  if (scanResult.found68 && !scanResult.found69) {
    return MPU_ADDR_LOW;
  }
  if (scanResult.found69 && !scanResult.found68) {
    return MPU_ADDR_HIGH;
  }
  return 0;
}

void probeWhoAmI(uint8_t addr) {
  uint8_t whoAmI = 0;
  Serial.print(F("# whoami_addr="));
  printHexByte(addr);
  if (!readRegister(addr, REG_WHO_AM_I, whoAmI)) {
    Serial.println(F(" result=read_fail"));
    return;
  }

  Serial.print(F(" result="));
  printHexByte(whoAmI);
  if (whoAmI == MPU6050_WHO_AM_I) {
    Serial.println(F(" status=mpu6050_signature"));
  } else {
    Serial.println(F(" status=unexpected_device"));
  }
}

GyroBias calibrateGyroBias(uint8_t addr) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  uint16_t validSamples = 0;
  RawMpuFrame frame;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readRawFrame(addr, frame)) {
      sumX += frame.gx;
      sumY += frame.gy;
      sumZ += frame.gz;
      ++validSamples;
    }
    delay(3);
  }

  GyroBias bias;
  if (validSamples == 0) {
    return bias;
  }

  bias.gx = static_cast<float>(sumX) / validSamples;
  bias.gy = static_cast<float>(sumY) / validSamples;
  bias.gz = static_cast<float>(sumZ) / validSamples;
  return bias;
}

int16_t applyBias(int16_t raw, float bias) {
  return static_cast<int16_t>(raw - bias);
}

void emitHeader() {
  Serial.println(F("# single_mpu_gesture"));
  Serial.print(F("# addr="));
  printHexByte(g_mpuAddr);
  Serial.println();
  Serial.print(F("# baud="));
  Serial.println(BAUD_RATE);
  Serial.print(F("# sample_rate_hz="));
  Serial.println(100);
  Serial.print(F("# accel_lsb_per_g="));
  Serial.println(ACCEL_LSB_PER_G, 1);
  Serial.print(F("# gyro_lsb_per_dps="));
  Serial.println(GYRO_LSB_PER_DPS, 1);
  Serial.println(F("ts_ms,ax,ay,az,gx,gy,gz"));
}

void recalibrate() {
  Serial.println(F("# recalibration_started"));
  delay(1000);
  g_bias = calibrateGyroBias(g_mpuAddr);
  Serial.println(F("# recalibration_done"));
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
  if (scanResult.found68) {
    probeWhoAmI(MPU_ADDR_LOW);
  }
  if (scanResult.found69) {
    probeWhoAmI(MPU_ADDR_HIGH);
  }

  g_mpuAddr = chooseMpuAddress(scanResult);
  if (g_mpuAddr == 0) {
    Serial.println(F("# startup_halted"));
    if (!scanResult.found68 && !scanResult.found69) {
      Serial.println(F("# no_mpu_found=check VCC GND SDA->20 SCL->21 AD0"));
    } else {
      Serial.println(F("# ambiguous_i2c=disconnect extra devices and keep only one MPU6050"));
    }
    while (true) {
      delay(1000);
    }
  }

  Serial.print(F("# selected_addr="));
  printHexByte(g_mpuAddr);
  Serial.println();

  const bool ok = initMpu(g_mpuAddr);
  Serial.print(F("# init_addr="));
  printHexByte(g_mpuAddr);
  Serial.print(F(" result="));
  Serial.println(ok ? F("ok") : F("fail"));
  if (!ok) {
    Serial.println(F("# startup_halted"));
    Serial.println(F("# init_failed=check sensor wiring or replace the module"));
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("# keep the hand still for gyro calibration"));
  delay(1500);
  g_bias = calibrateGyroBias(g_mpuAddr);
  emitHeader();
  g_lastSampleMs = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C') {
      recalibrate();
      emitHeader();
    }
  }

  uint32_t now = millis();
  if (now - g_lastSampleMs < SAMPLE_PERIOD_MS) {
    return;
  }
  g_lastSampleMs = now;

  RawMpuFrame frame;
  if (!readRawFrame(g_mpuAddr, frame)) {
    Serial.println(F("# read_error"));
    delay(10);
    return;
  }

  Serial.print(now);
  Serial.print(',');
  Serial.print(frame.ax);
  Serial.print(',');
  Serial.print(frame.ay);
  Serial.print(',');
  Serial.print(frame.az);
  Serial.print(',');
  Serial.print(applyBias(frame.gx, g_bias.gx));
  Serial.print(',');
  Serial.print(applyBias(frame.gy, g_bias.gy));
  Serial.print(',');
  Serial.println(applyBias(frame.gz, g_bias.gz));
}
