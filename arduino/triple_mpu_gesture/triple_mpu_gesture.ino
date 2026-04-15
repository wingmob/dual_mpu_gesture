#include <Wire.h>

namespace {
constexpr uint8_t HAND_ADDR = 0x68;
constexpr uint8_t THUMB_ADDR = 0x69;
constexpr uint8_t INDEX_ADDR = 0x68;

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
constexpr uint16_t CALIBRATION_SAMPLES = 400;

constexpr uint8_t SOFT_SDA_PIN = 16;
constexpr uint8_t SOFT_SCL_PIN = 17;
constexpr uint16_t SOFT_I2C_DELAY_US = 5;
constexpr uint32_t SOFT_I2C_TIMEOUT_US = 300;

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

struct ScanResult {
  uint8_t hardwareFoundCount = 0;
  bool foundHand = false;
  bool foundThumb = false;
  bool foundIndex = false;
};

GyroBias g_handBias;
GyroBias g_thumbBias;
GyroBias g_indexBias;
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

bool writeRegisterHardware(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readRegistersHardware(uint8_t addr, uint8_t startReg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t readCount = Wire.requestFrom(
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

bool readRegisterHardware(uint8_t addr, uint8_t reg, uint8_t& value) {
  return readRegistersHardware(addr, reg, &value, 1);
}

bool readRawFrameHardware(uint8_t addr, RawMpuFrame& frame) {
  uint8_t buffer[14];
  if (!readRegistersHardware(addr, REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
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

bool initMpuHardware(uint8_t addr) {
  if (!writeRegisterHardware(addr, REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);
  if (!writeRegisterHardware(addr, REG_SMPLRT_DIV, 0x09)) {
    return false;
  }
  if (!writeRegisterHardware(addr, REG_CONFIG, 0x03)) {
    return false;
  }
  if (!writeRegisterHardware(addr, REG_GYRO_CONFIG, 0x08)) {
    return false;
  }
  if (!writeRegisterHardware(addr, REG_ACCEL_CONFIG, 0x08)) {
    return false;
  }

  RawMpuFrame verify;
  return readRawFrameHardware(addr, verify);
}

void softReleaseLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

void softDriveLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

bool softReadLine(uint8_t pin) {
  return digitalRead(pin) == HIGH;
}

bool softWaitLineHigh(uint8_t pin) {
  const uint32_t startedAt = micros();
  while (!softReadLine(pin)) {
    if (micros() - startedAt > SOFT_I2C_TIMEOUT_US) {
      return false;
    }
  }
  return true;
}

void softI2CBegin() {
  softReleaseLine(SOFT_SDA_PIN);
  softReleaseLine(SOFT_SCL_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
}

bool softClockHigh() {
  softReleaseLine(SOFT_SCL_PIN);
  if (!softWaitLineHigh(SOFT_SCL_PIN)) {
    return false;
  }
  delayMicroseconds(SOFT_I2C_DELAY_US);
  return true;
}

void softClockLow() {
  softDriveLow(SOFT_SCL_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
}

bool softStart() {
  softReleaseLine(SOFT_SDA_PIN);
  softReleaseLine(SOFT_SCL_PIN);
  if (!softWaitLineHigh(SOFT_SDA_PIN) || !softWaitLineHigh(SOFT_SCL_PIN)) {
    return false;
  }
  delayMicroseconds(SOFT_I2C_DELAY_US);
  softDriveLow(SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
  softDriveLow(SOFT_SCL_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
  return true;
}

bool softStop() {
  softDriveLow(SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
  if (!softClockHigh()) {
    return false;
  }
  softReleaseLine(SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
  return softWaitLineHigh(SOFT_SDA_PIN);
}

bool softWriteBit(bool high) {
  if (high) {
    softReleaseLine(SOFT_SDA_PIN);
  } else {
    softDriveLow(SOFT_SDA_PIN);
  }
  delayMicroseconds(SOFT_I2C_DELAY_US);
  if (!softClockHigh()) {
    return false;
  }
  softClockLow();
  return true;
}

bool softReadBit(bool& bit) {
  softReleaseLine(SOFT_SDA_PIN);
  delayMicroseconds(SOFT_I2C_DELAY_US);
  if (!softClockHigh()) {
    return false;
  }
  bit = softReadLine(SOFT_SDA_PIN);
  softClockLow();
  return true;
}

bool softWriteByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if (!softWriteBit((value & mask) != 0)) {
      return false;
    }
  }

  bool ackBit = true;
  if (!softReadBit(ackBit)) {
    return false;
  }
  return !ackBit;
}

bool softReadByte(uint8_t& value, bool sendAck) {
  value = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    bool bit = false;
    if (!softReadBit(bit)) {
      return false;
    }
    value = static_cast<uint8_t>((value << 1) | (bit ? 1 : 0));
  }
  return softWriteBit(!sendAck);
}

bool writeRegisterSoft(uint8_t addr, uint8_t reg, uint8_t value) {
  if (!softStart()) {
    return false;
  }
  if (!softWriteByte(static_cast<uint8_t>(addr << 1))) {
    softStop();
    return false;
  }
  if (!softWriteByte(reg)) {
    softStop();
    return false;
  }
  if (!softWriteByte(value)) {
    softStop();
    return false;
  }
  return softStop();
}

bool readRegistersSoft(uint8_t addr, uint8_t startReg, uint8_t* buffer, uint8_t length) {
  if (!softStart()) {
    return false;
  }
  if (!softWriteByte(static_cast<uint8_t>(addr << 1))) {
    softStop();
    return false;
  }
  if (!softWriteByte(startReg)) {
    softStop();
    return false;
  }
  if (!softStart()) {
    softStop();
    return false;
  }
  if (!softWriteByte(static_cast<uint8_t>((addr << 1) | 0x01))) {
    softStop();
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    if (!softReadByte(buffer[i], i + 1 < length)) {
      softStop();
      return false;
    }
  }
  return softStop();
}

bool readRegisterSoft(uint8_t addr, uint8_t reg, uint8_t& value) {
  return readRegistersSoft(addr, reg, &value, 1);
}

bool readRawFrameSoft(uint8_t addr, RawMpuFrame& frame) {
  uint8_t buffer[14];
  if (!readRegistersSoft(addr, REG_ACCEL_XOUT_H, buffer, sizeof(buffer))) {
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

bool initMpuSoft(uint8_t addr) {
  if (!writeRegisterSoft(addr, REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(50);
  if (!writeRegisterSoft(addr, REG_SMPLRT_DIV, 0x09)) {
    return false;
  }
  if (!writeRegisterSoft(addr, REG_CONFIG, 0x03)) {
    return false;
  }
  if (!writeRegisterSoft(addr, REG_GYRO_CONFIG, 0x08)) {
    return false;
  }
  if (!writeRegisterSoft(addr, REG_ACCEL_CONFIG, 0x08)) {
    return false;
  }

  RawMpuFrame verify;
  return readRawFrameSoft(addr, verify);
}

bool probeSoftAddress(uint8_t addr) {
  if (!softStart()) {
    return false;
  }
  const bool ack = softWriteByte(static_cast<uint8_t>(addr << 1));
  softStop();
  return ack;
}

ScanResult scanBuses() {
  ScanResult result;

  Serial.println(F("# hw_i2c_scan_begin"));
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t error = Wire.endTransmission(true);
    if (error == 0) {
      ++result.hardwareFoundCount;
      result.foundHand = result.foundHand || (addr == HAND_ADDR);
      result.foundThumb = result.foundThumb || (addr == THUMB_ADDR);
      Serial.print(F("# hw_i2c_found="));
      printHexAddress(addr);
      if (addr == HAND_ADDR) {
        Serial.print(F(" expected_hand"));
      } else if (addr == THUMB_ADDR) {
        Serial.print(F(" expected_thumb"));
      }
      Serial.println();
    } else if (error == 4) {
      Serial.print(F("# hw_i2c_unknown_error="));
      printHexAddress(addr);
      Serial.println();
    }
  }
  Serial.print(F("# hw_i2c_scan_end count="));
  Serial.println(result.hardwareFoundCount);

  Serial.println(F("# soft_i2c_scan_begin"));
  result.foundIndex = probeSoftAddress(INDEX_ADDR);
  if (result.foundIndex) {
    Serial.print(F("# soft_i2c_found="));
    printHexAddress(INDEX_ADDR);
    Serial.println(F(" expected_index"));
  } else {
    Serial.print(F("# soft_i2c_missing="));
    printHexAddress(INDEX_ADDR);
    Serial.println(F(" expected_index"));
  }
  Serial.println(F("# soft_i2c_scan_end count=1"));

  Serial.print(F("# expected_hand_0x68="));
  Serial.println(result.foundHand ? F("found") : F("missing"));
  Serial.print(F("# expected_thumb_0x69="));
  Serial.println(result.foundThumb ? F("found") : F("missing"));
  Serial.print(F("# expected_index_soft_0x68="));
  Serial.println(result.foundIndex ? F("found") : F("missing"));
  return result;
}

void probeWhoAmIHardware(const __FlashStringHelper* label, uint8_t addr) {
  uint8_t whoAmI = 0;
  Serial.print(F("# whoami_sensor="));
  Serial.print(label);
  Serial.print(F(" addr="));
  printHexAddress(addr);
  if (!readRegisterHardware(addr, REG_WHO_AM_I, whoAmI)) {
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

void probeWhoAmISoft(const __FlashStringHelper* label, uint8_t addr) {
  uint8_t whoAmI = 0;
  Serial.print(F("# whoami_sensor="));
  Serial.print(label);
  Serial.print(F(" addr="));
  printHexAddress(addr);
  if (!readRegisterSoft(addr, REG_WHO_AM_I, whoAmI)) {
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

GyroBias calibrateGyroBiasHardware(uint8_t addr) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  RawMpuFrame frame;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readRawFrameHardware(addr, frame)) {
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

GyroBias calibrateGyroBiasSoft(uint8_t addr) {
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  RawMpuFrame frame;

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    if (readRawFrameSoft(addr, frame)) {
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
  Serial.println(F("# triple_mpu_gesture"));
  Serial.print(F("# baud="));
  Serial.println(BAUD_RATE);
  Serial.print(F("# sample_rate_hz="));
  Serial.println(100);
  Serial.print(F("# accel_lsb_per_g="));
  Serial.println(ACCEL_LSB_PER_G, 1);
  Serial.print(F("# gyro_lsb_per_dps="));
  Serial.println(GYRO_LSB_PER_DPS, 1);
  Serial.println(F("# sensor_1=hand_back"));
  Serial.println(F("# sensor_2=thumb"));
  Serial.println(F("# sensor_3=index"));
  Serial.print(F("# index_soft_i2c_pins="));
  Serial.print(SOFT_SDA_PIN);
  Serial.print(',');
  Serial.println(SOFT_SCL_PIN);
  Serial.println(F("ts_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3"));
}

void recalibrate() {
  Serial.println(F("# recalibration_started"));
  delay(1000);
  g_handBias = calibrateGyroBiasHardware(HAND_ADDR);
  g_thumbBias = calibrateGyroBiasHardware(THUMB_ADDR);
  g_indexBias = calibrateGyroBiasSoft(INDEX_ADDR);
  Serial.println(F("# recalibration_done"));
}

int16_t applyBias(int16_t raw, float bias) {
  return static_cast<int16_t>(raw - bias);
}
}  // namespace

void setup() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  softI2CBegin();
  Serial.begin(BAUD_RATE);
  delay(1000);

  Serial.println(F("# startup_begin"));
  Serial.print(F("# hw_i2c_clock_hz="));
  Serial.println(I2C_CLOCK_HZ);
  Serial.print(F("# soft_i2c_index_pins="));
  Serial.print(SOFT_SDA_PIN);
  Serial.print(',');
  Serial.println(SOFT_SCL_PIN);

  const ScanResult scanResult = scanBuses();
  probeWhoAmIHardware(F("hand"), HAND_ADDR);
  probeWhoAmIHardware(F("thumb"), THUMB_ADDR);
  probeWhoAmISoft(F("index"), INDEX_ADDR);

  if (scanResult.foundHand && !scanResult.foundThumb) {
    Serial.println(F("# hint_only_0x68=thumb MPU is probably still at 0x68"));
    Serial.println(F("# hint_check_thumb_ad0=tie thumb MPU AD0/ADO firmly to VCC; floating is not enough"));
  }

  const bool okHand = initMpuHardware(HAND_ADDR);
  const bool okThumb = initMpuHardware(THUMB_ADDR);
  const bool okIndex = initMpuSoft(INDEX_ADDR);

  Serial.print(F("# init_sensor=hand addr="));
  printHexAddress(HAND_ADDR);
  Serial.print(F(" result="));
  Serial.println(okHand ? F("ok") : F("fail"));

  Serial.print(F("# init_sensor=thumb addr="));
  printHexAddress(THUMB_ADDR);
  Serial.print(F(" result="));
  Serial.println(okThumb ? F("ok") : F("fail"));

  Serial.print(F("# init_sensor=index addr="));
  printHexAddress(INDEX_ADDR);
  Serial.print(F(" result="));
  Serial.println(okIndex ? F("ok") : F("fail"));

  if (!okHand || !okThumb || !okIndex) {
    Serial.println(F("# startup_halted"));
    if (!okHand) {
      Serial.println(F("# check_hand=AD0->GND SDA->20 SCL->21 VCC/GND"));
    }
    if (!okThumb) {
      Serial.println(F("# check_thumb=AD0->VCC SDA->20 SCL->21 VCC/GND"));
    }
    if (!okIndex) {
      Serial.println(F("# check_index=AD0->GND SDA->16 SCL->17 VCC/GND"));
      Serial.println(F("# hint_index_soft_bus=the third MPU is on a dedicated software I2C bus"));
    }
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("# keep all three sensors still for gyro calibration"));
  delay(1500);
  g_handBias = calibrateGyroBiasHardware(HAND_ADDR);
  g_thumbBias = calibrateGyroBiasHardware(THUMB_ADDR);
  g_indexBias = calibrateGyroBiasSoft(INDEX_ADDR);
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

  RawMpuFrame handFrame;
  RawMpuFrame thumbFrame;
  RawMpuFrame indexFrame;
  const bool okHand = readRawFrameHardware(HAND_ADDR, handFrame);
  const bool okThumb = readRawFrameHardware(THUMB_ADDR, thumbFrame);
  const bool okIndex = readRawFrameSoft(INDEX_ADDR, indexFrame);
  if (!okHand || !okThumb || !okIndex) {
    Serial.print(F("# read_error hand="));
    Serial.print(okHand ? 1 : 0);
    Serial.print(F(" thumb="));
    Serial.print(okThumb ? 1 : 0);
    Serial.print(F(" index="));
    Serial.println(okIndex ? 1 : 0);
    delay(10);
    return;
  }

  Serial.print(now);
  Serial.print(',');
  Serial.print(handFrame.ax);
  Serial.print(',');
  Serial.print(handFrame.ay);
  Serial.print(',');
  Serial.print(handFrame.az);
  Serial.print(',');
  Serial.print(applyBias(handFrame.gx, g_handBias.gx));
  Serial.print(',');
  Serial.print(applyBias(handFrame.gy, g_handBias.gy));
  Serial.print(',');
  Serial.print(applyBias(handFrame.gz, g_handBias.gz));
  Serial.print(',');
  Serial.print(thumbFrame.ax);
  Serial.print(',');
  Serial.print(thumbFrame.ay);
  Serial.print(',');
  Serial.print(thumbFrame.az);
  Serial.print(',');
  Serial.print(applyBias(thumbFrame.gx, g_thumbBias.gx));
  Serial.print(',');
  Serial.print(applyBias(thumbFrame.gy, g_thumbBias.gy));
  Serial.print(',');
  Serial.print(applyBias(thumbFrame.gz, g_thumbBias.gz));
  Serial.print(',');
  Serial.print(indexFrame.ax);
  Serial.print(',');
  Serial.print(indexFrame.ay);
  Serial.print(',');
  Serial.print(indexFrame.az);
  Serial.print(',');
  Serial.print(applyBias(indexFrame.gx, g_indexBias.gx));
  Serial.print(',');
  Serial.print(applyBias(indexFrame.gy, g_indexBias.gy));
  Serial.print(',');
  Serial.println(applyBias(indexFrame.gz, g_indexBias.gz));
}
