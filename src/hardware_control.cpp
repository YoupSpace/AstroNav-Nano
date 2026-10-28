#include "hardware_control.h"

#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <cmath>

// Defined in main.cpp.
extern BMP581 baro;
extern FlightState flightState;
extern bool missionStarted;
extern void updateStatusLed();
extern void appendMissionSample();

static constexpr float ACCEL_LSB_PER_G = 1024.0f;
// GYRO_CONFIG0 = 0x06 selects GYRO_UI_FS_SEL = 0 = +-4000 dps: 32768 / 4000 = 8.2 LSB per dps.
// (Same encoding as ACCEL_CONFIG0 = 0x06 -> +-32 g -> 1024 LSB per g above.)
static constexpr float GYRO_LSB_PER_DPS = 8.2f;
static constexpr float ADC_VREF = 3.3f;
static constexpr float ADC_COUNTS = 4095.0f;
static constexpr float VIN_DIVIDER_TOP_OHMS = 20000.0f;
static constexpr float VIN_DIVIDER_BOTTOM_OHMS = 10000.0f;
static constexpr float VIN_DIVIDER_RATIO =
  (VIN_DIVIDER_TOP_OHMS + VIN_DIVIDER_BOTTOM_OHMS) / VIN_DIVIDER_BOTTOM_OHMS;
static constexpr uint32_t CALIBRATION_TIMEOUT_MS = 6000;
static constexpr uint8_t CALIBRATION_GOOD_SAMPLES = 20;
static constexpr float STATIONARY_ACCEL_TOLERANCE_G = 0.08f;
static constexpr float STATIONARY_GYRO_TOLERANCE_DPS = 20.0f;
static constexpr uint32_t PYRO_PULSE_MS = 1000;

bool testCoreTick() {
  uint32_t start = millis();
  delay(2);
  return millis() > start;
}

bool testHeap() {
  const size_t testSize = 256;
  uint8_t *buffer = static_cast<uint8_t *>(malloc(testSize));
  if (buffer == nullptr) {
    return false;
  }

  for (size_t i = 0; i < testSize; i++) {
    buffer[i] = static_cast<uint8_t>(i ^ 0x5A);
  }

  bool ok = buffer[0] == 0x5A && buffer[1] == 0x5B && buffer[255] == static_cast<uint8_t>(255 ^ 0x5A);
  free(buffer);
  return ok;
}

bool readInputVoltage(float &voltage) {
  const int samples = 8;
  uint32_t total = 0;

  for (int i = 0; i < samples; i++) {
    total += static_cast<uint32_t>(analogRead(PIN_VIN_SENSE));
    delay(1);
  }

  float averageCounts = static_cast<float>(total) / static_cast<float>(samples);
  float senseVoltage = averageCounts * ADC_VREF / ADC_COUNTS;
  voltage = senseVoltage * VIN_DIVIDER_RATIO;
  return voltage > 0.1f;
}

bool readBaroSample(float &temperature, float &pressureHpa) {
  bmp5_sensor_data data = {0};
  if (baro.getSensorData(&data) != BMP5_OK) {
    return false;
  }

  temperature = data.temperature;
  pressureHpa = data.pressure / 100.0f;
  return pressureHpa > 300.0f && pressureHpa < 1200.0f;
}

ImuSample decodeSample(const uint8_t *rawData) {
  ImuSample sample;
  sample.ax = static_cast<int16_t>((rawData[1] << 8) | rawData[0]);
  sample.ay = static_cast<int16_t>((rawData[3] << 8) | rawData[2]);
  sample.az = static_cast<int16_t>((rawData[5] << 8) | rawData[4]);
  sample.gx = static_cast<int16_t>((rawData[7] << 8) | rawData[6]);
  sample.gy = static_cast<int16_t>((rawData[9] << 8) | rawData[8]);
  sample.gz = static_cast<int16_t>((rawData[11] << 8) | rawData[10]);
  return sample;
}

bool readImuFrame(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t rawData[12] = {0};

  for (int attempt = 0; attempt < 4; attempt++) {
    // Set the SPI mode before selecting the chip: the barometer uses mode 0 and the IMU
    // mode 3, so switching with CS already low would give the IMU a stray clock edge.
    SPI.beginTransaction(spiSettings);
    digitalWrite(PIN_CS_IMU, LOW);
    SPI.transfer(ICM_REG_ACCEL_DATA_X1_UI | 0x80);
    for (int i = 0; i < 12; i++) {
      rawData[i] = SPI.transfer(0x00);
    }
    digitalWrite(PIN_CS_IMU, HIGH);
    SPI.endTransaction();

    bool allZero = true;
    bool allFF = true;
    for (int i = 0; i < 12; i++) {
      if (rawData[i] != 0x00) {
        allZero = false;
      }
      if (rawData[i] != 0xFF) {
        allFF = false;
      }
    }

    if (!allZero && !allFF) {
      ImuSample sample = decodeSample(rawData);
      ax = static_cast<float>(sample.ax) / ACCEL_LSB_PER_G;
      ay = static_cast<float>(sample.ay) / ACCEL_LSB_PER_G;
      az = static_cast<float>(sample.az) / ACCEL_LSB_PER_G;
      gx = static_cast<float>(sample.gx) / GYRO_LSB_PER_DPS;
      gy = static_cast<float>(sample.gy) / GYRO_LSB_PER_DPS;
      gz = static_cast<float>(sample.gz) / GYRO_LSB_PER_DPS;
      return true;
    }

    delay(2);
  }

  return false;
}

bool verifyImuStream() {
  const int samples = 8;
  int validSamples = 0;
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  for (int i = 0; i < samples; i++) {
    if (readImuFrame(ax, ay, az, gx, gy, gz)) {
      if (isfinite(ax) && isfinite(ay) && isfinite(az) && isfinite(gx) && isfinite(gy) && isfinite(gz)) {
        validSamples++;
      }
    }
    delay(2);
  }

  return validSamples >= 6;
}

bool configureImu() {
  bool transportOk = false;
  uint8_t whoAmI = 0;

  for (size_t i = 0; i < 2; i++) {
    uint8_t value = readRegister(PIN_CS_IMU, 0x72, (i == 0)
      ? SPISettings(500000, MSBFIRST, SPI_MODE3)
      : SPISettings(500000, MSBFIRST, SPI_MODE0));
    if (value != 0x00 && value != 0xFF) {
      spiSettings = (i == 0)
        ? SPISettings(500000, MSBFIRST, SPI_MODE3)
        : SPISettings(500000, MSBFIRST, SPI_MODE0);
      whoAmI = value;
      transportOk = true;
      break;
    }
  }

  boardHealth.spiOk = transportOk;
  boardHealth.imuWhoAmIOk = transportOk && (whoAmI == ICM45686_EXPECTED_ID);

  if (!boardHealth.imuWhoAmIOk) {
    return false;
  }

  // Match the proven test-tool sequence: switch mode, wait briefly, then write config.
  if (!transportOk || whoAmI != ICM45686_EXPECTED_ID) {
    return false;
  }

  writeRegister(PIN_CS_IMU, ICM_REG_REG_MISC2, 0x02, spiSettings);
  delay(5);
  writeRegister(PIN_CS_IMU, ICM_REG_PWR_MGMT0, 0x0F, spiSettings);
  writeRegister(PIN_CS_IMU, ICM_REG_ACCEL_CONFIG0, 0x06, spiSettings);
  writeRegister(PIN_CS_IMU, ICM_REG_GYRO_CONFIG0, 0x06, spiSettings);

  boardHealth.imuConfigOk =
    readRegister(PIN_CS_IMU, ICM_REG_PWR_MGMT0, spiSettings) == 0x0F &&
    readRegister(PIN_CS_IMU, ICM_REG_ACCEL_CONFIG0, spiSettings) == 0x06 &&
    readRegister(PIN_CS_IMU, ICM_REG_GYRO_CONFIG0, spiSettings) == 0x06;

  return boardHealth.imuConfigOk;
}

bool calibratePadOrientation() {
  uint32_t deadline = millis() + CALIBRATION_TIMEOUT_MS;
  uint8_t stableCount = 0;
  float accelSumX = 0.0f;
  float accelSumY = 0.0f;
  float accelSumZ = 0.0f;
  float gyroSumX = 0.0f;
  float gyroSumY = 0.0f;
  float gyroSumZ = 0.0f;
  float temperatureSum = 0.0f;
  float pressureSum = 0.0f;

  while (millis() < deadline) {
    updateStatusLed();

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float temp = 0.0f;
    float pressure = 0.0f;

    bool imuOk = readImuFrame(ax, ay, az, gx, gy, gz);
    bool baroOk = readBaroSample(temp, pressure);
    if (!imuOk || !baroOk) {
      stableCount = 0;
      accelSumX = accelSumY = accelSumZ = 0.0f;
      gyroSumX = gyroSumY = gyroSumZ = 0.0f;
      temperatureSum = 0.0f;
      pressureSum = 0.0f;
      delay(40);
      continue;
    }

    float accelMag = magnitude3(ax, ay, az);
    float gyroMag = magnitude3(gx, gy, gz);
    if (fabsf(accelMag - 1.0f) <= STATIONARY_ACCEL_TOLERANCE_G &&
        gyroMag <= STATIONARY_GYRO_TOLERANCE_DPS) {
      accelSumX += ax;
      accelSumY += ay;
      accelSumZ += az;
      gyroSumX += gx;
      gyroSumY += gy;
      gyroSumZ += gz;
      temperatureSum += temp;
      pressureSum += pressure;
      stableCount++;

      if (stableCount >= CALIBRATION_GOOD_SAMPLES) {
        float invCount = 1.0f / static_cast<float>(stableCount);
        gravityAxisX = accelSumX * invCount;
        gravityAxisY = accelSumY * invCount;
        gravityAxisZ = accelSumZ * invCount;
        float gravityMag = magnitude3(gravityAxisX, gravityAxisY, gravityAxisZ);
        if (gravityMag > 0.01f) {
          gravityAxisX /= gravityMag;
          gravityAxisY /= gravityMag;
          gravityAxisZ /= gravityMag;
        } else {
          gravityAxisX = 0.0f;
          gravityAxisY = 0.0f;
          gravityAxisZ = 1.0f;
        }

        gyroBiasX = gyroSumX * invCount;
        gyroBiasY = gyroSumY * invCount;
        gyroBiasZ = gyroSumZ * invCount;
        filteredTemperature = temperatureSum * invCount;
        filteredPressure = pressureSum * invCount;
        baselinePressureHpa = filteredPressure;
        filteredAltitude = 0.0f;
        currentAltitude = 0.0f;
        currentVerticalVelocity = 0.0f;
        return true;
      }
    } else {
      stableCount = 0;
      accelSumX = accelSumY = accelSumZ = 0.0f;
      gyroSumX = gyroSumY = gyroSumZ = 0.0f;
      temperatureSum = 0.0f;
      pressureSum = 0.0f;
    }

    delay(40);
  }

  return false;
}

void firePyro() {
  if (pyroLatched) {
    return;
  }

  if (!missionStarted || (flightState != FlightState::Boost && flightState != FlightState::Coast &&
                          flightState != FlightState::Descent)) {
    Serial.println("[PYRO] Blocked: confirmed liftoff and active flight phase required.");
    return;
  }

  pyroLatched = true;
  pyroPulseUntilMs = millis() + PYRO_PULSE_MS;
  digitalWrite(PIN_PYRO, HIGH);
  appendMissionSample();
}

// Ground igniter test: same pulse as a real deployment, but it does not latch the flight
// pyro, so the board can still deploy on the next flight after the igniter is replaced.
// The caller (PYROTEST) only allows this in Idle before launch.
void firePyroTestPulse() {
  if (missionStarted || flightState != FlightState::Idle) {
    Serial.println("[PYRO] Test blocked: only allowed on the ground in idle.");
    return;
  }
  pyroPulseUntilMs = millis() + PYRO_PULSE_MS;
  digitalWrite(PIN_PYRO, HIGH);
}

void updatePyroOutput() {
  if (digitalRead(PIN_PYRO) == HIGH && static_cast<int32_t>(millis() - pyroPulseUntilMs) >= 0) {
    digitalWrite(PIN_PYRO, LOW);
  }
}

void recordFaultAndSafeStop() {
  setFlightState(FlightState::Fault);
  digitalWrite(PIN_PYRO, LOW);
}

uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings) {
  uint8_t value = 0;
  SPI.beginTransaction(settings);   // mode first, then CS (see readImuFrame)
  digitalWrite(csPin, LOW);
  SPI.transfer(regAddr | 0x80);
  value = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return value;
}

uint8_t readRegister(uint8_t csPin, uint8_t regAddr) {
  return readRegister(csPin, regAddr, spiSettings);
}

void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings) {
  SPI.beginTransaction(settings);   // mode first, then CS (see readImuFrame)
  digitalWrite(csPin, LOW);
  SPI.transfer(regAddr & 0x7F);
  SPI.transfer(value);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value) {
  writeRegister(csPin, regAddr, value, spiSettings);
}
