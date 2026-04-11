#include <Wire.h>

namespace {
constexpr uint32_t BAUD_RATE = 230400;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint8_t REG_WHO_AM_I = 0x75;

void printHexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t readCount = Wire.requestFrom(addr, static_cast<uint8_t>(1), static_cast<uint8_t>(true));
  if (readCount != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

const __FlashStringHelper *guessChip(uint8_t whoAmI) {
  switch (whoAmI) {
    case 0x68:
      return F("mpu6050_or_mpu6000");
    case 0x69:
      return F("mpu9250_or_mpu9255");
    case 0x70:
      return F("mpu6500_or_compatible");
    case 0x71:
      return F("mpu9250_or_icm_variant");
    case 0x12:
      return F("icm20602_or_compatible");
    default:
      return F("unknown");
  }
}

void probeAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  const uint8_t ack = Wire.endTransmission(true);
  if (ack != 0) {
    return;
  }

  Serial.print(F("# i2c_found="));
  printHexByte(addr);

  uint8_t whoAmI = 0;
  if (readRegister(addr, REG_WHO_AM_I, whoAmI)) {
    Serial.print(F(" whoami="));
    printHexByte(whoAmI);
    Serial.print(F(" guess="));
    Serial.print(guessChip(whoAmI));
  } else {
    Serial.print(F(" whoami=read_fail"));
  }

  if (addr == 0x68) {
    Serial.print(F(" role_hint=mpu1_candidate"));
  } else if (addr == 0x69) {
    Serial.print(F(" role_hint=mpu2_candidate"));
  }
  Serial.println();
}

void runProbe() {
  Serial.println(F("# probe_begin"));
  Serial.print(F("# i2c_clock_hz="));
  Serial.println(I2C_CLOCK_HZ);

  uint8_t foundCount = 0;
  bool found68 = false;
  bool found69 = false;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    const uint8_t ack = Wire.endTransmission(true);
    if (ack == 0) {
      ++foundCount;
      found68 = found68 || (addr == 0x68);
      found69 = found69 || (addr == 0x69);
      probeAddress(addr);
    } else if (ack == 4) {
      Serial.print(F("# i2c_unknown_error="));
      printHexByte(addr);
      Serial.println();
    }
  }

  Serial.print(F("# found_count="));
  Serial.println(foundCount);
  Serial.print(F("# expected_0x68="));
  Serial.println(found68 ? F("found") : F("missing"));
  Serial.print(F("# expected_0x69="));
  Serial.println(found69 ? F("found") : F("missing"));

  uint8_t whoAmI = 0;
  Serial.print(F("# probe_target=0x68 whoami="));
  if (readRegister(0x68, REG_WHO_AM_I, whoAmI)) {
    printHexByte(whoAmI);
  } else {
    Serial.print(F("read_fail"));
  }
  Serial.println();

  Serial.print(F("# probe_target=0x69 whoami="));
  if (readRegister(0x69, REG_WHO_AM_I, whoAmI)) {
    printHexByte(whoAmI);
  } else {
    Serial.print(F("read_fail"));
  }
  Serial.println();

  Serial.println(F("# probe_done"));
  Serial.println(F("# hint=rescan_send_s"));
}
}  // namespace

void setup() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Serial.begin(BAUD_RATE);
  delay(1000);

  Serial.println(F("# mpu_whoami_probe"));
  Serial.print(F("# baud="));
  Serial.println(BAUD_RATE);
  runProbe();
}

void loop() {
  if (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 's' || command == 'S' || command == '\n' || command == '\r') {
      runProbe();
    }
  }
}
