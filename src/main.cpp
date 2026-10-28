#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <FatFS.h>
#include <FatFSUSB.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#include "hardware_control.h"
#include "otp_memory.h"
#include "usb_info_files.h"
#include "build_info.h"

#ifndef AUTO_VERSION
#define AUTO_VERSION "dev"
#endif

class Assets {
 public:
  const char *Firmware_Version = AUTO_VERSION;
  uint16_t TotalFlights = 0;
  String Device_Key = "0";
};

Assets assets;

namespace OldCodeTemporary {
String generateRandomKey(uint8_t len) {
  uint32_t seed = micros() ^ (millis() << 16) ^ static_cast<uint32_t>(analogRead(PIN_VIN_SENSE));
  randomSeed(seed);

  const char chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  String key = "";
  for (uint8_t i = 0; i < len; i++) {
    key += chars[random(0, sizeof(chars) - 1)];
  }
  return key;
}
}

bool isPlaceholderSerial(const char *serialNumber) {
  return !serialNumber || serialNumber[0] == '\0' || strcmp(serialNumber, "AN-0001") == 0;
}

/*
 * AstroNav flight firmware for RP2350
 * - Startup health check with LED status
 * - Launch and apogee detection
 * - Safe pyro pulse and lockout
 * - RAM mission log flushed to flash after landing
 * - USB mass-storage mode when powered from USB
 */

// Sensor scaling, ADC and calibration constants live in hardware_control.cpp.
static constexpr float LIPO_1S_LOW_VOLTAGE = 3.5f;
static constexpr float LIPO_2S_LOW_VOLTAGE = 7.0f;

static constexpr uint32_t SAMPLE_PERIOD_MS = 50;
static constexpr uint32_t LED_PERIOD_MS = 25;
static constexpr uint32_t BOOT_PULSE_MS = 2000;
static constexpr float LAUNCH_THRESHOLD_G = 1.35f;
static constexpr uint8_t LAUNCH_CONFIRM_SAMPLES = 3;
static constexpr uint8_t BOOST_DECAY_CONFIRM_SAMPLES = 3;
static constexpr uint32_t LAUNCH_MIN_TIME_MS = 400;
static constexpr float APOGEE_DROP_M = 0.25f;
static constexpr float APOGEE_VELOCITY_THRESHOLD_MPS = -0.25f;
// Near apogee the rocket is in free fall (~0 g). On the ground, carried, or swinging
// in a tree it reads 0.7-1.3 g, so this keeps apogee from triggering there.
static constexpr float APOGEE_ACCEL_MAX_G = 0.50f;
static constexpr uint8_t APOGEE_CONFIRM_SAMPLES = 3;
static constexpr uint8_t APOGEE_CONFIRM_SAMPLES_NEAR_EXPECTED = 2;
static constexpr float APOGEE_MIN_DROP_M = 0.30f;            // above baro noise
static constexpr float APOGEE_PEAK_HOLD_BAND_M = 0.30f;
static constexpr float APOGEE_PEAK_HOLD_MAX_SPEED_MPS = 0.50f;
static constexpr float APOGEE_BACKUP_DROP_MIN_M = 0.5f;
static constexpr float APOGEE_BACKUP_DROP_MAX_M = 5.0f;
// Without the IMU's free-fall check, the barometer must show a real descent.
static constexpr float APOGEE_BARO_ONLY_DESCENT_MPS = -1.0f;
// Fallback when the IMU never reads free fall (tumbling, spinning, board far from the CG):
// a clear barometer descent still declares apogee, a little late but before it is too fast.
static constexpr float APOGEE_FALLBACK_DESCENT_MPS = -3.0f;
static constexpr float APOGEE_FALLBACK_MIN_DROP_M = 2.0f;
static constexpr uint8_t APOGEE_FALLBACK_CONFIRM_SAMPLES = 3;
// The fallback is only allowed after a real powered climb. Walking, stairs or a lift stay
// far below this, so carrying a falsely triggered board downhill cannot fire the pyro.
static constexpr float APOGEE_FALLBACK_MIN_CLIMB_MPS = 10.0f;
// A "launch" that comes to rest without climbing this high was a bump while handling the
// rocket: return to Idle so the real launch is still detected and the pyro stays available.
static constexpr float FALSE_LAUNCH_MAX_CLIMB_M = 3.0f;
// If the pyro was blocked only by a momentary barometer glitch, it may still fire
// shortly after apogee; later than this the parachute timing would be wrong.
static constexpr uint32_t PYRO_RETRY_WINDOW_MS = 1000;
static constexpr float LANDING_ALTITUDE_TOLERANCE_M = 2.0f;
static constexpr float LANDING_VELOCITY_TOLERANCE_MPS = 0.25f;
static constexpr uint8_t LANDING_CONFIRM_SAMPLES = 20;
// Without a detected apogee, require a longer quiet period so the near-zero
// velocity around apogee cannot be mistaken for a landing.
static constexpr uint8_t LANDING_CONFIRM_SAMPLES_WITHOUT_APOGEE = 60;
// A single sensor cannot be cross-checked, so it must stay calm for longer.
static constexpr uint8_t LANDING_CONFIRM_SAMPLES_SINGLE_IMU = 100;
static constexpr uint32_t LANDING_TIMEOUT_MIN_MS = 300000;
static constexpr uint32_t LANDING_TIMEOUT_MAX_MS = 1800000;
static constexpr float LANDING_SLOWEST_DESCENT_MPS = 2.0f;
// "Calm" = resting still; "minimal" = small rocking (wind) but clearly not flying or swinging.
static constexpr float LANDING_IMU_CALM_ACCEL_G = 0.08f;
static constexpr float LANDING_IMU_CALM_ACCEL_SPAN_G = 0.05f;
static constexpr float LANDING_IMU_CALM_GYRO_DPS = 5.0f;
static constexpr float LANDING_IMU_MINIMAL_ACCEL_G = 0.20f;
static constexpr float LANDING_IMU_MINIMAL_ACCEL_SPAN_G = 0.25f;
static constexpr float LANDING_IMU_MINIMAL_GYRO_DPS = 30.0f;
static constexpr float LANDING_BARO_CALM_SPAN_M = 0.6f;
static constexpr float LANDING_BARO_MINIMAL_SPAN_M = 1.0f;   // < 1 m/s: slower than any realistic parachute descent
static constexpr uint8_t SENSOR_HISTORY_SAMPLES = 20;   // 1 s at SAMPLE_PERIOD_MS
static constexpr uint8_t VELOCITY_WINDOW_SAMPLES = 8;   // 0.4 s slope, ~0.2 s lag
static constexpr float ALTITUDE_FILTER_ALPHA = 0.5f;
static constexpr float BARO_SPIKE_NOISE_MARGIN_M = 2.0f;
// The allowed jump already grows with the time since the last accepted sample, so real
// changes are accepted on their own; this only re-anchors after 1 s of disagreement.
static constexpr uint8_t BARO_SPIKE_MAX_CONSECUTIVE_REJECTS = 20;
// Without fresh barometer data, extrapolate at most 0.5 s and then hold the altitude.
static constexpr uint8_t BARO_MAX_EXTRAPOLATION_SAMPLES = 10;
static constexpr uint32_t PYRO_TEST_CONFIRM_WINDOW_MS = 5000;
static constexpr uint32_t APOGEE_FIRE_MAX_MS = 500;
static constexpr uint8_t APOGEE_HOLD_SAMPLES = 8;
static constexpr uint32_t SENSOR_FAILURE_WINDOW_MS = 1000;
static constexpr uint8_t SENSOR_FAILURE_LIMIT = 5;
// RAM flight log: 6144 samples x 44 bytes = 270 KB = 307 s at SAMPLE_PERIOD_MS.
// Adjustable; see "Flight Log Storage" in SOFTWARE_DESIGN.md before changing it.
static constexpr uint16_t FLIGHT_LOG_CAPACITY = 6144;
// Long flights: start moving the oldest samples to flash at 75% full, 20 rows per sample tick.
static constexpr uint16_t LOG_STREAM_START_SAMPLES = (FLIGHT_LOG_CAPACITY / 4) * 3;
static constexpr uint16_t LOG_STREAM_BATCH_SAMPLES = 20;
static constexpr size_t LOG_ROW_MAX_BYTES = 256;
static constexpr uint64_t LOG_STORAGE_RESERVE_BYTES = 16384;   // keeps room to rewrite Settings.ini
static constexpr uint8_t LOG_MAX_OPEN_ATTEMPTS = 3;
static constexpr uint8_t PRELAUNCH_SAMPLE_CAPACITY = 20;
static constexpr uint32_t PROFILE_MAGIC = 0x50455246;   // "FRFP"
static constexpr uint32_t PROFILE_VERSION = 1;
static constexpr const char *DEVICE_INFO_FILE = "/Settings.ini";
static constexpr const char *HOWTO_FILE = "/Howto.txt";
static constexpr const char *WEBSITE_FILE = "/Website.url";
static constexpr const char *AUTORUN_FILE = "/autorun.inf";
static constexpr const char *USB_VOLUME_LABEL = "AstroNav Nano";
static constexpr const char *LEGACY_SETTINGS_FILE = "/SETTINGS.INI";
static constexpr const char *LEGACY_PROFILE_FILE = "/PROFILE.INI";
static constexpr const char *LEGACY_DEBUG_FILE = "/DEBUG.INI";
static constexpr const char *LEGACY_HOWTO_FILE = "/HOWTO.INI";
static constexpr const char *LEGACY_HOWTO_TEXT_FILE = "/HOWTO.TXT";
static constexpr const char *LEGACY_WEBSITE_FILE = "/WEBSITE.URL";
static constexpr const char *LEGACY_MOREHELP_FILE = "/MoreHelp.url";
static constexpr const char *LEGACY_AUTORUN_FILE = "/AUTORUN.INF";
static constexpr const char *SECTION_FILES = "files";
static constexpr const char *SECTION_SETTINGS = "settings";
static constexpr const char *SECTION_PROFILE = "profile";
static constexpr const char *SECTION_DEBUG = "debug";

enum class PowerMode {
  Unknown,
  USB,
  LiPo1S,
  LiPo2S
};

// One distinct colour per state; boot (white), USB (blue) and fault (red) are never reused.
enum class LedProfile {
  Boot,          // white, breathing
  Calibrating,   // yellow, blinking
  Idle,          // green, breathing: ready for launch
  Usb,           // blue, breathing
  Boost,         // orange, solid
  Coast,         // purple, solid
  Descent,       // cyan, solid
  Landed,        // magenta, blinking
  Fault          // red, fast blinking
};

struct FlightSample {
  uint32_t counter = 0;
  uint32_t ms = 0;
  int16_t axMg = 0;
  int16_t ayMg = 0;
  int16_t azMg = 0;
  int16_t gxDps10 = 0;
  int16_t gyDps10 = 0;
  int16_t gzDps10 = 0;
  int16_t tempCd10 = 0;
  uint16_t pressureHd10 = 0;
  int16_t batteryVoltageMv = 0;
  // 32-bit so high and fast flights do not saturate (16-bit cm would cap at 327 m and 327 m/s).
  // The apogee-drop CSV columns are derived from these when the log is written.
  int32_t altitudeCm = 0;
  int32_t velocityCms = 0;
  int16_t rollD10 = 0;
  int16_t pitchD10 = 0;
  uint8_t healthBits = 0;
  uint8_t state = 0;
  uint8_t pyro = 0;
  uint8_t baroRejected = 0;
};

struct FlightSettings {
  float estimatedHeightM = 120.0f;
  float estimatedSpeedMps = 35.0f;
  float heightMarginM = 20.0f;
  float speedMarginMps = 8.0f;
  float launchThresholdG = 1.35f;
  bool firePyroAtApogee = true;
  float minPyroFireHeightM = 5.0f;
};

struct DeviceProfile {
  uint32_t magic = PROFILE_MAGIC;
  uint32_t version = PROFILE_VERSION;
  char serialNumber[20] = "";
  char flashStamp[24] = "";
  uint32_t flightCount = 0;
  uint32_t logCount = 0;
  float maxFlightAltitudeM = 0.0f;
  float maxFlightSpeedMps = 0.0f;
  float lastFlightAltitudeM = 0.0f;
  float lastFlightSpeedMps = 0.0f;
  uint32_t checksum = 0;
};

Adafruit_NeoPixel led(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
BMP581 baro;
SPISettings spiSettings(500000, MSBFIRST, SPI_MODE3);

BoardHealth boardHealth;
PowerMode powerMode = PowerMode::Unknown;
PowerMode detectedBatteryMode = PowerMode::Unknown;
FlightState flightState = FlightState::Booting;

bool imuInitialized = false;
bool storageReady = false;
bool usbDriveReady = false;
bool usbFlightOverride = false;
volatile bool usbExitRequested = false;   // set by the USB eject callback
volatile bool usbMediaPresent = false;   // shared with the USB task callbacks
bool settingsNeedPersist = false;
bool missionLogFlushed = false;
bool pyroLatched = false;
bool missionStarted = false;
bool flightRecordsPendingSave = false;
bool lowVoltageFault = false;
bool sensorFault = false;

uint32_t bootMs = 0;
uint32_t bootPulseUntilMs = 0;
uint32_t nextSampleMs = 0;
uint32_t nextLedMs = 0;
uint32_t launchMs = 0;
uint32_t pyroPulseUntilMs = 0;
uint32_t lastLogMs = 0;
uint32_t lastUsbSettingsPollMs = 0;
char serialCommandBuffer[48] = {0};
uint8_t serialCommandLength = 0;
bool pyroTestPendingConfirm = false;
uint32_t pyroTestConfirmUntilMs = 0;
// GYROTEST: integrates the bias-corrected gyro into a total angle per axis, so the gyro
// scale can be checked by turning the board a known angle (speed does not matter).
static constexpr uint32_t GYRO_TEST_DURATION_MS = 10000;
bool gyroTestActive = false;
uint32_t gyroTestEndMs = 0;
uint32_t gyroTestLastSampleMs = 0;
float gyroTestAngleX = 0.0f;
float gyroTestAngleY = 0.0f;
float gyroTestAngleZ = 0.0f;

float currentVinVoltage = 0.0f;
float currentAx = 0.0f;
float currentAy = 0.0f;
float currentAz = 0.0f;
float currentGx = 0.0f;
float currentGy = 0.0f;
float currentGz = 0.0f;
float currentTemperature = 0.0f;
float currentPressure = 0.0f;
float currentAltitude = 0.0f;
float currentVerticalVelocity = 0.0f;
float currentRollDeg = 0.0f;
float currentPitchDeg = 0.0f;
float baselinePressureHpa = 0.0f;
float filteredTemperature = 0.0f;
float filteredPressure = 0.0f;
float filteredAltitude = 0.0f;
float peakAltitude = 0.0f;
float gravityAxisX = 0.0f;
float gravityAxisY = 0.0f;
float gravityAxisZ = 1.0f;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

uint8_t launchConfirmCount = 0;
uint8_t boostDecayConfirmCount = 0;
uint8_t apogeeConfirmCount = 0;
uint8_t apogeeFallbackConfirmCount = 0;
uint8_t landingConfirmCount = 0;
uint32_t flightCountBeforeLaunch = 0;
uint32_t falseLaunchCount = 0;
uint8_t apogeePeakHoldCount = 0;
uint32_t apogeePeakHoldStartMs = 0;
uint8_t imuFailureCount = 0;
uint8_t baroFailureCount = 0;
uint32_t imuFailureWindowStartMs = 0;
uint32_t baroFailureWindowStartMs = 0;
uint32_t logCounter = 0;
float peakFlightSpeedMps = 0.0f;
float peakClimbRateMps = 0.0f;   // fastest climb, from accepted barometer samples only
float peakAccelMagG = 0.0f;
bool apogeeDetected = false;
float detectedApogeeAltitudeM = 0.0f;
uint32_t detectedApogeeMs = 0;
const char *apogeeTrigger = "none";
const char *landingTrigger = "none";

float altitudeHistory[SENSOR_HISTORY_SAMPLES] = {0};
float accelMagHistory[SENSOR_HISTORY_SAMPLES] = {0};
uint8_t altitudeHistoryIndex = 0;
uint8_t altitudeHistoryCount = 0;
uint8_t accelHistoryIndex = 0;
uint8_t accelHistoryCount = 0;
float lastAcceptedRawAltitude = 0.0f;
uint8_t baroConsecutiveRejects = 0;
bool baroSampleFresh = false;
bool baroSampleRejected = false;
uint32_t baroRejectedSamples = 0;
float flightMaxPlausibleSpeedMps = 100.0f;
float flightExpectedApogeeLowM = 100.0f;
float flightExpectedApogeeHighM = 140.0f;
float flightMinPyroFireHeightM = 5.0f;
float launchAltitudeM = 0.0f;
uint32_t pyroRetryUntilMs = 0;
const char *pyroBlockedReason = "none";
uint32_t sampleGaps = 0;

struct MissionLogSummary {
  float maxAltitudeM = 0.0f;
  float maxSpeedMps = 0.0f;
  float maxAccelG = 0.0f;
  float maxBatteryV = 0.0f;
  float lastAltitudeM = 0.0f;
};
MissionLogSummary logSummary;
bool missionLogStreaming = false;
uint32_t missionLogRowsWritten = 0;
uint32_t missionLogRowsDropped = 0;   // did not fit on flash
uint32_t missionLogRamDropped = 0;    // RAM buffer full before the descent
uint8_t missionLogOpenAttempts = 0;
const char *lastLogStatus = "none";

float flightLaunchThresholdG = 1.35f;
float flightApogeeDropM = 0.25f;
bool pyroFireAtApogeeAllowed = true;

FlightSettings flightSettings;
DeviceProfile deviceProfile;

FlightSample missionLog[FLIGHT_LOG_CAPACITY];
FlightSample prelaunchSamples[PRELAUNCH_SAMPLE_CAPACITY];
uint16_t missionLogCount = 0;
uint8_t prelaunchSampleCount = 0;
uint32_t currentFlightNumber = 0;
uint32_t nextFlightNumber = 1;

char missionLogPath[48] = {0};
char lastFaultReason[96] = "none";

void setStatusLED(uint8_t r, uint8_t g, uint8_t b);
void setLedProfile(LedProfile profile);
void updateStatusLed();
bool setFlightState(FlightState nextState);
float clampFloat(float value, float minimum, float maximum);
bool testCoreTick();
bool testHeap();
bool readInputVoltage(float &voltage);
bool readBaroSample(float &temperature, float &pressureHpa);
bool readImuFrame(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
bool verifyImuStream();
bool configureImu();
bool calibratePadOrientation();
void resetAltitudeEstimator();
void appendMissionSample();
void flushMissionLogToFlash();
void streamMissionLogIfNeeded();
void resetMissionLogWriter();
uint64_t logStorageAvailableBytes();
void firePyro();
void updatePyroOutput();
void updateFlightStateFromSamples();
void recordFaultAndSafeStop();
void startMissionLog();
void prepareNextFlightNumber();
void finalizeFlightPersonalRecords();
void ensureLogsDirectory();
void handleSerialCommand(const char *command);
void enterFlightModeFromUsb();
void onUsbStorageUnplug(uint32_t cbData);
void ensureUsbInfoFiles();
bool configureUsbVolumeLabel();
bool shouldExitUsbModeFromUsbFiles();
bool loadFlightSettings();
bool loadOrCreateDeviceProfile();
void persistDeviceProfile();
void syncAssetsFromDeviceProfile();
void buildFaultReason(char *buffer, size_t bufferSize);
uint8_t healthBits();
bool systemHealthy();
uint32_t scanFlightLogs(uint32_t &flightLogCount);

uint8_t readRegister(uint8_t csPin, uint8_t regAddr, SPISettings settings);
uint8_t readRegister(uint8_t csPin, uint8_t regAddr);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value, SPISettings settings);
void writeRegister(uint8_t csPin, uint8_t regAddr, uint8_t value);
ImuSample decodeSample(const uint8_t *rawData);

// Shown at boot so an unexpected reboot (crash, watchdog, brownout) is visible in the log.
const char *resetReasonText() {
  switch (rp2040.getResetReason()) {
    case RP2040::PWRON_RESET: return "power_on";
    case RP2040::RUN_PIN_RESET: return "run_pin";
    case RP2040::SOFT_RESET: return "software";
    case RP2040::WDT_RESET: return "watchdog";
    case RP2040::DEBUG_RESET: return "debugger";
    case RP2040::GLITCH_RESET: return "glitch";
    case RP2040::BROWNOUT_RESET: return "brownout";
    case RP2040::UNKNOWN_RESET:
    default: return "unknown";
  }
}

const char *boolText(bool value) {
  return value ? "OK" : "FAIL";
}

const char *stateText(FlightState state) {
  switch (state) {
    case FlightState::Booting: return "boot";
    case FlightState::Calibrating: return "calibrating";
    case FlightState::Idle: return "idle";
    case FlightState::UsbMode: return "storage";
    case FlightState::Armed: return "armed";
    case FlightState::Boost: return "boost";
    case FlightState::Coast: return "coast";
    case FlightState::Descent: return "descent";
    case FlightState::Landed: return "landed";
    case FlightState::Fault:
    default: return "fault";
  }
}

bool setFlightState(FlightState nextState) {
  if (flightState == nextState) {
    return true;
  }
  // Boost/Coast -> Idle is only used by abortFalseLaunch(), which runs before any apogee
  // (so before the pyro can have fired) and after the barometer showed no real climb.

  bool allowed = false;
  switch (flightState) {
    case FlightState::Booting:
      allowed = (nextState == FlightState::Calibrating || nextState == FlightState::UsbMode || nextState == FlightState::Fault);
      break;
    case FlightState::Calibrating:
      allowed = (nextState == FlightState::Idle || nextState == FlightState::Fault);
      break;
    case FlightState::Idle:
      allowed = (nextState == FlightState::Boost || nextState == FlightState::UsbMode || nextState == FlightState::Fault);
      break;
    case FlightState::UsbMode:
      allowed = (nextState == FlightState::Calibrating || nextState == FlightState::Fault);
      break;
    case FlightState::Armed:
      allowed = (nextState == FlightState::Boost || nextState == FlightState::Fault);
      break;
    case FlightState::Boost:
      allowed = (nextState == FlightState::Coast || nextState == FlightState::Descent ||
                 nextState == FlightState::Landed || nextState == FlightState::Idle ||
                 nextState == FlightState::Fault);
      break;
    case FlightState::Coast:
      allowed = (nextState == FlightState::Descent || nextState == FlightState::Landed ||
                 nextState == FlightState::Idle || nextState == FlightState::Fault);
      break;
    case FlightState::Descent:
      allowed = (nextState == FlightState::Landed || nextState == FlightState::Fault);
      break;
    case FlightState::Landed:
      allowed = (nextState == FlightState::Fault);
      break;
    case FlightState::Fault:
    default:
      allowed = (nextState == FlightState::Fault);
      break;
  }

  if (!allowed) {
    boardHealth.warning = true;
    strncpy(lastFaultReason, "stateTransition", sizeof(lastFaultReason) - 1);
    lastFaultReason[sizeof(lastFaultReason) - 1] = '\0';
    return false;
  }

  flightState = nextState;
  if (missionStarted) {
    appendMissionSample();
  }
  return true;
}

const char *powerModeText(PowerMode mode) {
  switch (mode) {
    case PowerMode::USB: return "USB";
    case PowerMode::LiPo1S: return "1S LiPo";
    case PowerMode::LiPo2S: return "2S LiPo";
    case PowerMode::Unknown:
    default: return "Unknown";
  }
}

const char *runtimeModeText() {
  if (powerMode == PowerMode::USB && !usbFlightOverride) {
    return "usb-storage";
  }

  return "flight";
}

void buildFlashStamp(char *buffer, size_t bufferSize) {
  static const char *months[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  int day = 1;
  int year = 2026;
  int monthIndex = 7;
  char monthText[4] = {0};

  if (sscanf(__DATE__, "%3s %d %d", monthText, &day, &year) == 3) {
    for (int i = 0; i < 12; i++) {
      if (strcmp(monthText, months[i]) == 0) {
        monthIndex = i;
        break;
      }
    }
  }

  snprintf(buffer, bufferSize, "%s %02d-%02d-%04d",
           __TIME__,
           day,
           monthIndex + 1,
           year);
}

uint32_t checksumBytes(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

void applyFlightSettings() {
  // Same ranges as the Settings.ini validation in loadFlightSettings().
  float heightMargin = clampFloat(flightSettings.heightMarginM, 0.0f, 100000.0f);
  float speedMargin = clampFloat(flightSettings.speedMarginMps, 0.0f, 3000.0f);

  flightLaunchThresholdG = clampFloat(flightSettings.launchThresholdG, 1.15f, 4.0f);
  pyroFireAtApogeeAllowed = flightSettings.firePyroAtApogee;

  // The estimates are a baseline only: they tune how fast apogee is confirmed and the
  // backup drop threshold, but never prevent apogee on a much lower or higher flight.
  // Backup trigger: a drop of ~1% of the expected apogee, bounded so it stays early.
  flightApogeeDropM = clampFloat((flightSettings.estimatedHeightM + heightMargin) * 0.01f,
                                 APOGEE_BACKUP_DROP_MIN_M, APOGEE_BACKUP_DROP_MAX_M);
  flightExpectedApogeeLowM = flightSettings.estimatedHeightM - heightMargin;
  flightExpectedApogeeHighM = flightSettings.estimatedHeightM + heightMargin;
  flightMinPyroFireHeightM = clampFloat(flightSettings.minPyroFireHeightM, 0.0f, 10000.0f);

  // Twice the expected top speed: anything faster between two samples is a sensor glitch.
  flightMaxPlausibleSpeedMps = clampFloat((flightSettings.estimatedSpeedMps + speedMargin) * 2.0f, 50.0f, 12000.0f);
}

void makeDefaultFlightSettings() {
  flightSettings.estimatedHeightM = 120.0f;
  flightSettings.estimatedSpeedMps = 35.0f;
  flightSettings.heightMarginM = 20.0f;
  flightSettings.speedMarginMps = 8.0f;
  flightSettings.launchThresholdG = 1.35f;
  flightSettings.firePyroAtApogee = true;
  flightSettings.minPyroFireHeightM = 5.0f;
  applyFlightSettings();
}

void buildFaultReason(char *buffer, size_t bufferSize) {
  buffer[0] = '\0';

  auto appendReason = [&](const char *label) {
    if (!label || !label[0]) {
      return;
    }
    size_t currentLength = strlen(buffer);
    size_t labelLength = strlen(label);
    if (currentLength + labelLength + 2 >= bufferSize) {
      return;
    }
    if (currentLength > 0) {
      strncat(buffer, ",", bufferSize - currentLength - 1);
    }
    strncat(buffer, label, bufferSize - strlen(buffer) - 1);
  };

  if (!boardHealth.coreTickOk) appendReason("coreTick");
  if (!boardHealth.heapOk) appendReason("heap");
  if (!boardHealth.spiOk) appendReason("spi");
  if (!boardHealth.imuWhoAmIOk) appendReason("imuWhoAmI");
  if (!boardHealth.imuConfigOk) appendReason("imuConfig");
  if (!boardHealth.imuStreamOk) appendReason("imuStream");
  if (!boardHealth.baroOk) appendReason("baro");
  if (!boardHealth.flashFsOk) appendReason("flashFs");
  if (!boardHealth.usbStorageOk && powerMode == PowerMode::USB) appendReason("usbStorage");
  if (!boardHealth.vinOk) appendReason("vin");
  if (powerMode == PowerMode::Unknown) appendReason("powerMode");

  if (buffer[0] == '\0') {
    strncpy(buffer, "none", bufferSize - 1);
    buffer[bufferSize - 1] = '\0';
  }
}

void saveFlightSettings();

bool parseBoolValue(const char *text) {
  if (!text) {
    return false;
  }
  while (*text == ' ' || *text == '\t') {
    text++;
  }
  return (strcasecmp(text, "TRUE") == 0) || (strcasecmp(text, "1") == 0) ||
         (strcasecmp(text, "YES") == 0) || (strcasecmp(text, "ON") == 0);
}

// Accepts only a complete, finite number ("12.5"); rejects "", "abc", "12m", "nan", "inf".
bool parseFloatSetting(const char *text, float &out) {
  if (!text || text[0] == '\0') {
    return false;
  }
  char *end = nullptr;
  float value = strtof(text, &end);
  if (end == text) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    end++;
  }
  if (*end != '\0' || !isfinite(value)) {
    return false;
  }
  out = value;
  return true;
}

bool parseStrictBoolValue(const char *text, bool &out) {
  if (!text) {
    return false;
  }
  while (*text == ' ' || *text == '\t') {
    text++;
  }
  if (parseBoolValue(text)) {
    out = true;
    return true;
  }
  if (strcasecmp(text, "FALSE") == 0 || strcasecmp(text, "0") == 0 ||
      strcasecmp(text, "NO") == 0 || strcasecmp(text, "OFF") == 0) {
    out = false;
    return true;
  }
  return false;
}

bool readSettingLine(const char *line, const char *key, char *valueBuffer, size_t valueBufferSize) {
  size_t keyLength = strlen(key);
  if (strncmp(line, key, keyLength) != 0 || line[keyLength] != '=') {
    return false;
  }

  const char *valueStart = line + keyLength + 1;
  while (*valueStart == ' ' || *valueStart == '\t') {
    valueStart++;
  }

  strncpy(valueBuffer, valueStart, valueBufferSize - 1);
  valueBuffer[valueBufferSize - 1] = '\0';

  size_t valueLength = strlen(valueBuffer);
  while (valueLength > 0 && (valueBuffer[valueLength - 1] == '\r' || valueBuffer[valueLength - 1] == '\n' || valueBuffer[valueLength - 1] == ' ' || valueBuffer[valueLength - 1] == '\t')) {
    valueBuffer[valueLength - 1] = '\0';
    valueLength--;
  }

  return true;
}

bool readProfileLine(const char *line, const char *key, char *valueBuffer, size_t valueBufferSize) {
  return readSettingLine(line, key, valueBuffer, valueBufferSize);
}

void writeProfileFieldLine(File &file, const char *key, const char *value) {
  file.printf("%s=%s\n", key, value ? value : "");
}

bool parseIniSectionHeader(const char *line, char *sectionBuffer, size_t sectionBufferSize) {
  if (!line || line[0] != '[') {
    return false;
  }

  const char *sectionStart = line + 1;
  const char *sectionEnd = strchr(sectionStart, ']');
  if (!sectionEnd || sectionEnd <= sectionStart) {
    return false;
  }

  size_t length = static_cast<size_t>(sectionEnd - sectionStart);
  if (length >= sectionBufferSize) {
    length = sectionBufferSize - 1;
  }

  for (size_t i = 0; i < length; i++) {
    sectionBuffer[i] = static_cast<char>(tolower(static_cast<unsigned char>(sectionStart[i])));
  }
  sectionBuffer[length] = '\0';
  return true;
}

bool loadFlightSettings() {
  const char *sourcePath = nullptr;
  bool legacyFormat = false;

  if (storageReady && FatFS.exists(DEVICE_INFO_FILE)) {
    sourcePath = DEVICE_INFO_FILE;
  } else if (storageReady && FatFS.exists(LEGACY_SETTINGS_FILE)) {
    sourcePath = LEGACY_SETTINGS_FILE;
    legacyFormat = true;
  }

  if (!sourcePath) {
    makeDefaultFlightSettings();
    return false;
  }

  File settingsFile = FatFS.open(sourcePath, "r");
  if (!settingsFile) {
    makeDefaultFlightSettings();
    return false;
  }

  makeDefaultFlightSettings();
  bool settingsDirty = false;
  // Valid ranges: a value outside them (or not a number, NaN, inf) keeps the default and
  // the file is rewritten, so a typo can never disable launch, apogee or pyro safety.
  struct FloatSettingField {
    const char *key;
    float *target;
    float minimum;
    float maximum;
    bool loaded;
  };
  FloatSettingField floatFields[] = {
    {"ESTIMATED_HEIGHT_M", &flightSettings.estimatedHeightM, 0.0f, 100000.0f, false},
    {"ESTIMATED_SPEED_MPS", &flightSettings.estimatedSpeedMps, 0.0f, 3000.0f, false},
    {"HEIGHT_MARGIN_M", &flightSettings.heightMarginM, 0.0f, 100000.0f, false},
    {"SPEED_MARGIN_MPS", &flightSettings.speedMarginMps, 0.0f, 3000.0f, false},
    {"LAUNCH_THRESHOLD_G", &flightSettings.launchThresholdG, 1.15f, 4.0f, false},
    {"MIN_PYRO_FIRE_HEIGHT_M", &flightSettings.minPyroFireHeightM, 0.0f, 10000.0f, false},
  };
  bool firePyroAtApogeeLoaded = false;
  char currentSection[32] = {0};
  char line[128];

  while (settingsFile.available()) {
    size_t length = settingsFile.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';

    if (line[0] == '#' || line[0] == ';' || line[0] == '[' || line[0] == '\0') {
      if (line[0] == '[') {
        parseIniSectionHeader(line, currentSection, sizeof(currentSection));
      }
      continue;
    }

    if (!legacyFormat && strcmp(currentSection, SECTION_SETTINGS) != 0) {
      continue;
    }

    char value[64];
    bool matched = false;
    for (FloatSettingField &field : floatFields) {
      if (!readSettingLine(line, field.key, value, sizeof(value))) {
        continue;
      }
      matched = true;
      field.loaded = true;
      float parsed = 0.0f;
      if (parseFloatSetting(value, parsed) && parsed >= field.minimum && parsed <= field.maximum) {
        *field.target = parsed;
      } else {
        Serial.printf("[SETTINGS] Invalid %s='%s' (allowed %.2f..%.2f). Using default %.2f.\n",
                      field.key, value, field.minimum, field.maximum, *field.target);
        settingsDirty = true;
      }
      break;
    }

    if (!matched && readSettingLine(line, "FIRE_PYRO_APOGEE", value, sizeof(value))) {
      firePyroAtApogeeLoaded = true;
      bool parsed = false;
      if (parseStrictBoolValue(value, parsed)) {
        flightSettings.firePyroAtApogee = parsed;
      } else {
        // Unrecognized text: choose the option that cannot fire the pyro, and show it in the file.
        Serial.printf("[SETTINGS] Invalid FIRE_PYRO_APOGEE='%s'. Using FALSE.\n", value);
        flightSettings.firePyroAtApogee = false;
        settingsDirty = true;
      }
    }
  }

  settingsFile.close();
  applyFlightSettings();
  for (const FloatSettingField &field : floatFields) {
    if (!field.loaded) {
      settingsDirty = true;
    }
  }
  if (!firePyroAtApogeeLoaded) {
    settingsDirty = true;
  }

  settingsNeedPersist = settingsDirty;
  return true;
}

void saveFlightSettings() {
  if (!storageReady) {
    return;
  }

  persistDeviceProfile();
}

void makeDefaultDeviceProfile() {
  memset(&deviceProfile, 0, sizeof(deviceProfile));
  deviceProfile.magic = PROFILE_MAGIC;
  deviceProfile.version = PROFILE_VERSION;
  deviceProfile.serialNumber[0] = '\0';
  assets.Device_Key = "0";
  buildFlashStamp(deviceProfile.flashStamp, sizeof(deviceProfile.flashStamp));
  assets.TotalFlights = 0;
}

void ensureDeviceSerial() {
  if (ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES) {
    snprintf(deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber), "RECOVERY");
    assets.Device_Key = deviceProfile.serialNumber;
    return;
  }

  uint32_t otpSerialNumber = 0;
  if (astroNavOtpGetSerialNumber(otpSerialNumber)) {
    formatAstroNavDisplaySerial(otpSerialNumber, deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber));
    assets.Device_Key = deviceProfile.serialNumber;
    return;
  }

  if (!isPlaceholderSerial(deviceProfile.serialNumber)) {
    assets.Device_Key = deviceProfile.serialNumber;
    return;
  }

  assets.Device_Key = OldCodeTemporary::generateRandomKey(10);
  snprintf(deviceProfile.serialNumber, sizeof(deviceProfile.serialNumber), "%s", assets.Device_Key.c_str());
}

// Checksum over the profile exactly as it is written to Settings.ini (same text and
// number formatting), so a saved profile always verifies when it is read back.
uint32_t computeProfileChecksum(const char *flashStampText) {
  char stamp[sizeof(deviceProfile.flashStamp)] = {0};
  snprintf(stamp, sizeof(stamp), "%s", flashStampText ? flashStampText : "");
  char canonical[160];
  int length = snprintf(canonical, sizeof(canonical), "%08lX|%s|%s|%lu|%lu|%.1f|%.1f|%.1f|%.1f",
                        static_cast<unsigned long>(deviceProfile.magic),
                        deviceProfile.serialNumber,
                        stamp,
                        static_cast<unsigned long>(deviceProfile.flightCount),
                        static_cast<unsigned long>(deviceProfile.logCount),
                        deviceProfile.maxFlightAltitudeM,
                        deviceProfile.maxFlightSpeedMps,
                        deviceProfile.lastFlightAltitudeM,
                        deviceProfile.lastFlightSpeedMps);
  if (length < 0) {
    length = 0;
  } else if (static_cast<size_t>(length) >= sizeof(canonical)) {
    length = sizeof(canonical) - 1;
  }
  return checksumBytes(reinterpret_cast<const uint8_t *>(canonical), static_cast<size_t>(length));
}

void updateDeviceProfileChecksum() {
  deviceProfile.checksum = computeProfileChecksum(deviceProfile.flashStamp);
}

void syncAssetsFromDeviceProfile() {
  assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
  assets.Device_Key = deviceProfile.serialNumber;
}

bool loadOrCreateDeviceProfile() {
  if (ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  bool legacyFormat = false;
  const char *sourcePath = nullptr;

  if (storageReady && FatFS.exists(DEVICE_INFO_FILE)) {
    sourcePath = DEVICE_INFO_FILE;
  } else if (storageReady && FatFS.exists(LEGACY_PROFILE_FILE)) {
    sourcePath = LEGACY_PROFILE_FILE;
    legacyFormat = true;
  }

  if (!sourcePath) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  File profileFile = FatFS.open(sourcePath, "r");
  if (!profileFile) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  makeDefaultDeviceProfile();
  bool anyValue = false;
  bool checksumPresent = false;
  uint32_t expectedChecksum = 0;
  char currentSection[32] = {0};
  char line[160];

  while (profileFile.available()) {
    size_t length = profileFile.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';

    if (line[0] == '#' || line[0] == ';' || line[0] == '[' || line[0] == '\0') {
      if (line[0] == '[') {
        parseIniSectionHeader(line, currentSection, sizeof(currentSection));
      }
      continue;
    }

    if (!legacyFormat && strcmp(currentSection, SECTION_PROFILE) != 0) {
      continue;
    }

    char value[96];
    if (readProfileLine(line, "MAGIC", value, sizeof(value))) {
      deviceProfile.magic = static_cast<uint32_t>(strtoul(value, nullptr, 16));
      anyValue = true;
    } else if (readProfileLine(line, "SERIAL_NUMBER", value, sizeof(value))) {
      strncpy(deviceProfile.serialNumber, value, sizeof(deviceProfile.serialNumber) - 1);
      deviceProfile.serialNumber[sizeof(deviceProfile.serialNumber) - 1] = '\0';
      anyValue = true;
      if (!isPlaceholderSerial(deviceProfile.serialNumber)) {
        assets.Device_Key = deviceProfile.serialNumber;
      }
    } else if (readProfileLine(line, "FLASH_STAMP", value, sizeof(value))) {
      strncpy(deviceProfile.flashStamp, value, sizeof(deviceProfile.flashStamp) - 1);
      deviceProfile.flashStamp[sizeof(deviceProfile.flashStamp) - 1] = '\0';
      anyValue = true;
    } else if (readProfileLine(line, "FLIGHT_COUNT", value, sizeof(value))) {
      deviceProfile.flightCount = static_cast<uint32_t>(strtoul(value, nullptr, 10));
      anyValue = true;
      assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
    } else if (readProfileLine(line, "LOG_COUNT", value, sizeof(value))) {
      deviceProfile.logCount = static_cast<uint32_t>(strtoul(value, nullptr, 10));
      anyValue = true;
    } else if (readProfileLine(line, "MAX_FLIGHT_ALTITUDE_M", value, sizeof(value))) {
      deviceProfile.maxFlightAltitudeM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "MAX_FLIGHT_SPEED_MPS", value, sizeof(value))) {
      deviceProfile.maxFlightSpeedMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "LAST_FLIGHT_ALTITUDE_M", value, sizeof(value))) {
      deviceProfile.lastFlightAltitudeM = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "LAST_FLIGHT_SPEED_MPS", value, sizeof(value))) {
      deviceProfile.lastFlightSpeedMps = static_cast<float>(atof(value));
      anyValue = true;
    } else if (readProfileLine(line, "CHECKSUM", value, sizeof(value))) {
      expectedChecksum = static_cast<uint32_t>(strtoul(value, nullptr, 16));
      checksumPresent = true;
    }
  }

  profileFile.close();

  deviceProfile.checksum = 0;
  if (deviceProfile.magic != PROFILE_MAGIC || deviceProfile.version != PROFILE_VERSION) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  uint32_t computedChecksum = computeProfileChecksum(deviceProfile.flashStamp);
  if (!anyValue || !checksumPresent || computedChecksum != expectedChecksum) {
    makeDefaultDeviceProfile();
    updateDeviceProfileChecksum();
    return false;
  }

  deviceProfile.checksum = expectedChecksum;
  ensureDeviceSerial();
  syncAssetsFromDeviceProfile();
  return true;
}

void persistDeviceProfile() {
  // Never write while the drive is exported to the computer: the host and the firmware
  // must not use the flash at the same time.
  if (!storageReady || usbDriveReady || ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES) {
    return;
  }

  ensureDeviceSerial();
  Serial.println("[BOOT] stage=persist_otp_read_begin");
  AstroNav_OTP_Data otpData = {};
  bool otpReadable = readAstroNavOtpData(otpData);
  Serial.printf("[BOOT] stage=persist_otp_read_complete ok=%s\n", boolText(otpReadable));
  const AstroNavOtpStatus &otpStatus = getAstroNavOtpStatus();
  char firstFlashStamp[32] = {0};
  snprintf(firstFlashStamp, sizeof(firstFlashStamp), "%s", deviceProfile.flashStamp);
  if (otpReadable && otpStatus.has_magic_header) {
    int firstHour = 0;
    int firstMinute = 0;
    int firstDay = 0;
    int firstMonth = 0;
    int firstYear = 0;
    if (sscanf(otpData.production_date, "%2d-%2d-%2d-%2d-%4d", &firstHour, &firstMinute, &firstDay, &firstMonth, &firstYear) == 5) {
      snprintf(
        firstFlashStamp,
        sizeof(firstFlashStamp),
        "%02d:%02d:%02u %02d-%02d-%04d",
        firstHour,
        firstMinute,
        static_cast<unsigned int>(otpData.reserved[0]),
        firstDay,
        firstMonth,
        firstYear
      );
    }
  }

  // FLASH_STAMP is written as firstFlashStamp, so the checksum must cover that value.
  deviceProfile.checksum = computeProfileChecksum(firstFlashStamp);
  Serial.println("[BOOT] stage=persist_file_open_begin");
  File infoFile = FatFS.open(DEVICE_INFO_FILE, "w");
  Serial.printf("[BOOT] stage=persist_file_open_complete ok=%s\n", boolText(static_cast<bool>(infoFile)));
  if (!infoFile) {
    boardHealth.warning = true;
    return;
  }

  infoFile.println("; AstroNav Nano combined device settings");
  infoFile.println("[files]");
  infoFile.println("SETTINGS_FILE=Settings.ini");
  infoFile.println("HOWTO_FILE=Howto.txt");
  infoFile.println("WEBSITE_FILE=Website.url");
  infoFile.println("LOG_DIRECTORY=/logs");
  infoFile.println("LOG_PATTERN=Flight_*.csv");
  infoFile.println();
  UsbInfoFiles::writeSettingsSection(infoFile,
                                    flightSettings.estimatedHeightM,
                                    flightSettings.estimatedSpeedMps,
                                    flightSettings.heightMarginM,
                                    flightSettings.speedMarginMps,
                                    flightSettings.launchThresholdG,
                                    flightSettings.firePyroAtApogee,
                                    flightSettings.minPyroFireHeightM);
  infoFile.println();
  infoFile.println("[profile]");
  writeProfileFieldLine(infoFile, "MAGIC", "50455246");
  infoFile.printf("VERSION=%s\n", assets.Firmware_Version);
  infoFile.printf("CURRENT_FIRMWARE_VERSION=%s\n", assets.Firmware_Version);
  writeProfileFieldLine(infoFile, "SERIAL_NUMBER", deviceProfile.serialNumber);
  writeProfileFieldLine(infoFile, "FLASH_STAMP", firstFlashStamp);
  writeProfileFieldLine(infoFile, "LAST_FLASH_STAMP", deviceProfile.flashStamp);
  infoFile.printf("FLIGHT_COUNT=%lu\n", static_cast<unsigned long>(deviceProfile.flightCount));
  infoFile.printf("LOG_COUNT=%lu\n", static_cast<unsigned long>(deviceProfile.logCount));
  // Same "%.1f" formatting as computeProfileChecksum, so the values round-trip exactly.
  infoFile.printf("MAX_FLIGHT_ALTITUDE_M=%.1f\n", deviceProfile.maxFlightAltitudeM);
  infoFile.printf("MAX_FLIGHT_SPEED_MPS=%.1f\n", deviceProfile.maxFlightSpeedMps);
  infoFile.printf("LAST_FLIGHT_ALTITUDE_M=%.1f\n", deviceProfile.lastFlightAltitudeM);
  infoFile.printf("LAST_FLIGHT_SPEED_MPS=%.1f\n", deviceProfile.lastFlightSpeedMps);
  infoFile.printf("CHECKSUM=%08lX\n", static_cast<unsigned long>(deviceProfile.checksum));
  infoFile.println();
  infoFile.println("[otp]");
  infoFile.printf("OTP_READ_OK=%s\n", otpReadable ? "TRUE" : "FALSE");
  infoFile.printf("OTP_PAYLOAD_VALID=%s\n", otpStatus.payload_valid ? "TRUE" : "FALSE");
  infoFile.printf("OTP_HARDWARE_MATCHES_BUILD=%s\n", otpStatus.hardware_matches_build ? "TRUE" : "FALSE");
  infoFile.printf("OTP_BLANK=%s\n", otpStatus.blank ? "TRUE" : "FALSE");
  infoFile.printf("OTP_WRITE_BLOCKED=%s\n", otpStatus.write_blocked ? "TRUE" : "FALSE");
  infoFile.printf("OTP_MAGIC_VALID=%s\n", otpStatus.has_magic_header ? "TRUE" : "FALSE");
  infoFile.printf("OTP_PROGRAMMED_THIS_BOOT=%s\n", astroNavOtpWasProgrammedThisBoot() ? "TRUE" : "FALSE");
  infoFile.printf("OTP_PROFILE_MATCHES_CURRENT_BUILD=%s\n", otpStatus.signature_matches_build ? "TRUE" : "FALSE");
  infoFile.printf("OTP_OFFICIAL_SIGNATURE=%s\n", otpStatus.official_signature ? "TRUE" : "FALSE");
  if (otpReadable) {
    char otpDisplaySerial[16] = {0};
    char otpInitialFirmwareVersion[28] = {0};
    formatAstroNavDisplaySerial(otpData.serial_number, otpDisplaySerial, sizeof(otpDisplaySerial));
    formatAstroNavOtpInitialFirmwareVersion(otpData, otpInitialFirmwareVersion, sizeof(otpInitialFirmwareVersion));
    infoFile.printf("OTP_MAGIC_HEADER=%08lX\n", static_cast<unsigned long>(otpData.magic_header));
    infoFile.printf("OTP_MANUFACTURER_ID=%s\n", otpData.manufacturer_id);
    infoFile.printf("OTP_PRODUCT_ID=%s\n", otpData.product_id);
    infoFile.printf("OTP_HARDWARE_MAJOR=%u\n", static_cast<unsigned int>(otpData.hardware_major));
    infoFile.printf("OTP_HARDWARE_MINOR=%u\n", static_cast<unsigned int>(otpData.hardware_minor));
    infoFile.printf("OTP_PRODUCTION_DATE=%s\n", otpData.production_date);
    infoFile.printf("OTP_PRODUCTION_SECOND=%u\n", static_cast<unsigned int>(otpData.reserved[0]));
    infoFile.printf("OTP_PRODUCTION_TIMESTAMP=%s-%02u\n", otpData.production_date, static_cast<unsigned int>(otpData.reserved[0]));
    infoFile.printf("OTP_FIRST_FIRMWARE_VERSION=%s\n", otpInitialFirmwareVersion);
    infoFile.printf("OTP_SERIAL_NUMBER=%s\n", otpDisplaySerial);
    infoFile.printf("OTP_INITIAL_FIRMWARE=%u.%u.%u\n",
                    static_cast<unsigned int>(otpData.initial_firmware[0]),
                    static_cast<unsigned int>(otpData.initial_firmware[1]),
                    static_cast<unsigned int>(otpData.initial_firmware[2]));
    infoFile.printf("OTP_WARRANTY_SIGNATURE=%08lX\n", static_cast<unsigned long>(otpData.warranty_signature));
    infoFile.printf("CURRENT_FIRMWARE_VERSION=%s\n", assets.Firmware_Version);
    infoFile.printf("CURRENT_FLASH_STAMP=%s\n", deviceProfile.flashStamp);
    infoFile.printf("OTP_FIRST_FIRMWARE_MATCHES_CURRENT=%s\n",
                    strcmp(otpInitialFirmwareVersion, assets.Firmware_Version) == 0 ? "TRUE" : "FALSE");
  }
  Serial.println("[BOOT] stage=persist_profile_write_complete");
  infoFile.println();
  UsbInfoFiles::writeDebugSection(infoFile,
                                  lastFaultReason,
                                  boardHealth.coreTickOk,
                                  boardHealth.heapOk,
                                  boardHealth.spiOk,
                                  boardHealth.imuWhoAmIOk,
                                  boardHealth.imuConfigOk,
                                  boardHealth.imuStreamOk,
                                  boardHealth.baroOk,
                                  boardHealth.flashFsOk,
                                  boardHealth.usbStorageOk,
                                  boardHealth.vinOk,
                                  boardHealth.warning,
                                  boardHealth.critical,
                                  currentVinVoltage,
                                  healthBits(),
                                  runtimeModeText(),
                                  stateText(flightState),
                                  powerModeText(powerMode));
  Serial.println("[BOOT] stage=persist_debug_write_complete");
  infoFile.println();
  infoFile.println("[storage]");
  FSInfo storageInfo = {};
  if (FatFS.info(storageInfo)) {
    uint64_t freeBytes = (storageInfo.totalBytes > storageInfo.usedBytes) ? (storageInfo.totalBytes - storageInfo.usedBytes) : 0;
    infoFile.printf("STORAGE_TOTAL_KB=%lu\n", static_cast<unsigned long>(storageInfo.totalBytes / 1024));
    infoFile.printf("STORAGE_FREE_KB=%lu\n", static_cast<unsigned long>(freeBytes / 1024));
    // ~140 bytes per CSV row at 20 rows per second.
    infoFile.printf("ESTIMATED_LOG_SECONDS_FREE=%lu\n", static_cast<unsigned long>(logStorageAvailableBytes() / (140u * 20u)));
  }
  infoFile.printf("LAST_LOG_STATUS=%s\n", lastLogStatus);
  infoFile.close();
  Serial.println("[BOOT] stage=persist_close_complete");
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

PowerMode classifyPowerSource(float voltage) {
  if (voltage >= 4.4f && voltage <= 5.5f) {
    return PowerMode::USB;
  }
  if (voltage >= 3.0f && voltage <= 4.35f) {
    return PowerMode::LiPo1S;
  }
  if (voltage >= 6.0f && voltage <= 8.4f) {
    return PowerMode::LiPo2S;
  }
  return PowerMode::Unknown;
}

bool systemHealthy() {
  return boardHealth.coreTickOk && boardHealth.heapOk && boardHealth.spiOk &&
         boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk && boardHealth.imuStreamOk &&
         boardHealth.baroOk && !boardHealth.critical;
}

bool isLiPoVoltageTooLow() {
  if (detectedBatteryMode == PowerMode::LiPo1S) {
    return currentVinVoltage <= LIPO_1S_LOW_VOLTAGE;
  }
  if (detectedBatteryMode == PowerMode::LiPo2S) {
    return currentVinVoltage <= LIPO_2S_LOW_VOLTAGE;
  }
  return false;
}

void recordSensorResult(bool success, bool &sensorOk, uint8_t &failureCount, uint32_t &windowStartMs) {
  uint32_t now = millis();
  if (success) {
    sensorOk = true;
    failureCount = 0;
    windowStartMs = now;
    return;
  }

  sensorOk = false;
  if (failureCount == 0 || now - windowStartMs > SENSOR_FAILURE_WINDOW_MS) {
    failureCount = 1;
    windowStartMs = now;
  } else if (failureCount < 255) {
    failureCount++;
  }

  boardHealth.warning = true;
  if (failureCount >= SENSOR_FAILURE_LIMIT) {
    sensorFault = true;
  }
}

float magnitude3(float x, float y, float z) {
  return sqrtf((x * x) + (y * y) + (z * z));
}

float altitudeFromPressure(float pressureHpa) {
  if (baselinePressureHpa <= 0.0f || pressureHpa <= 0.0f) {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(pressureHpa / baselinePressureHpa, 0.190294957f));
}

uint8_t healthBits() {
  uint8_t bits = 0;
  bits |= boardHealth.coreTickOk ? (1 << 0) : 0;
  bits |= boardHealth.heapOk ? (1 << 1) : 0;
  bits |= boardHealth.spiOk ? (1 << 2) : 0;
  bits |= boardHealth.imuWhoAmIOk ? (1 << 3) : 0;
  bits |= boardHealth.imuConfigOk ? (1 << 4) : 0;
  bits |= boardHealth.imuStreamOk ? (1 << 5) : 0;
  bits |= boardHealth.baroOk ? (1 << 6) : 0;
  bits |= boardHealth.flashFsOk ? (1 << 7) : 0;
  return bits;
}

// Picks the next free flight number on the ground, so launch needs no flash access.
void prepareNextFlightNumber() {
  uint32_t flightLogCount = 0;
  uint32_t candidateFlightNumber = scanFlightLogs(flightLogCount) + 1u;
  deviceProfile.logCount = flightLogCount;
  char path[sizeof(missionLogPath)];
  for (;;) {
    snprintf(path, sizeof(path), "/logs/Flight_%08lu.csv", static_cast<unsigned long>(candidateFlightNumber));
    if (!storageReady || !FatFS.exists(path)) {
      break;
    }
    candidateFlightNumber++;
  }
  nextFlightNumber = candidateFlightNumber;
}

// Runs at launch detection: RAM only. The flight count and records are written to
// flash after landing (finalizeFlightPersonalRecords), never during the flight.
void startMissionLog() {
  flightCountBeforeLaunch = deviceProfile.flightCount;
  currentFlightNumber = nextFlightNumber;
  snprintf(missionLogPath, sizeof(missionLogPath), "/logs/Flight_%08lu.csv", static_cast<unsigned long>(currentFlightNumber));
  deviceProfile.flightCount = currentFlightNumber;
  assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
  Serial.printf("[LOG] Starting mission file: %s\n", missionLogPath);
  missionLogCount = 0;
  logCounter = 0;
  for (uint8_t index = 0; index < prelaunchSampleCount; index++) {
    missionLog[missionLogCount] = prelaunchSamples[index];
    missionLog[missionLogCount].counter = ++logCounter;
    missionLogCount++;
  }
  prelaunchSampleCount = 0;
  peakAltitude = 0.0f;
  launchConfirmCount = 0;
  boostDecayConfirmCount = 0;
  apogeeConfirmCount = 0;
  apogeeFallbackConfirmCount = 0;
  landingConfirmCount = 0;
  apogeePeakHoldCount = 0;
  apogeePeakHoldStartMs = 0;
  missionStarted = true;
  missionLogFlushed = false;
  flightRecordsPendingSave = true;
  peakFlightSpeedMps = 0.0f;
  peakClimbRateMps = 0.0f;
  peakAccelMagG = 0.0f;
  apogeeDetected = false;
  detectedApogeeAltitudeM = 0.0f;
  detectedApogeeMs = 0;
  apogeeTrigger = "none";
  landingTrigger = "none";
  baroRejectedSamples = 0;
  pyroBlockedReason = "none";
  pyroRetryUntilMs = 0;
  sampleGaps = 0;
  resetMissionLogWriter();
}

// Undoes startMissionLog() after a false launch (a bump while handling): the RAM log is
// discarded, the flight number is reused and the board is back in Idle, so the real launch
// is detected and the pyro is still available. Nothing was written to flash for it.
// Returns false (caller then lands normally) if the state change is refused.
bool abortFalseLaunch() {
  if (!setFlightState(FlightState::Idle)) {
    return false;
  }
  falseLaunchCount++;
  Serial.printf("[FLIGHT] False launch: climbed only %.2f m, back to idle (count=%lu).\n",
                peakAltitude - launchAltitudeM, static_cast<unsigned long>(falseLaunchCount));
  missionStarted = false;
  missionLogCount = 0;
  logCounter = 0;
  flightRecordsPendingSave = false;
  deviceProfile.flightCount = flightCountBeforeLaunch;
  assets.TotalFlights = static_cast<uint16_t>(deviceProfile.flightCount > UINT16_MAX ? UINT16_MAX : deviceProfile.flightCount);
  resetMissionLogWriter();
  launchConfirmCount = 0;
  boostDecayConfirmCount = 0;
  apogeeConfirmCount = 0;
  apogeeFallbackConfirmCount = 0;
  landingConfirmCount = 0;
  apogeePeakHoldCount = 0;
  apogeePeakHoldStartMs = 0;
  peakAltitude = 0.0f;
  peakFlightSpeedMps = 0.0f;
  peakClimbRateMps = 0.0f;
  peakAccelMagG = 0.0f;
  landingTrigger = "none";
  pyroBlockedReason = "none";
  return true;
}

void finalizeFlightPersonalRecords() {
  if (!flightRecordsPendingSave) {
    return;
  }

  if (peakAltitude > deviceProfile.maxFlightAltitudeM) {
    deviceProfile.maxFlightAltitudeM = peakAltitude;
  }
  if (peakFlightSpeedMps > deviceProfile.maxFlightSpeedMps) {
    deviceProfile.maxFlightSpeedMps = peakFlightSpeedMps;
  }

  deviceProfile.lastFlightAltitudeM = peakAltitude;
  deviceProfile.lastFlightSpeedMps = peakFlightSpeedMps;
  persistDeviceProfile();
  flightRecordsPendingSave = false;
}

void appendMissionSample() {
  if (!missionStarted) {
    if (prelaunchSampleCount >= PRELAUNCH_SAMPLE_CAPACITY) {
      memmove(&prelaunchSamples[0], &prelaunchSamples[1],
              sizeof(FlightSample) * (PRELAUNCH_SAMPLE_CAPACITY - 1));
      prelaunchSampleCount = PRELAUNCH_SAMPLE_CAPACITY - 1;
    }
  }

  if (missionStarted && missionLogCount >= FLIGHT_LOG_CAPACITY) {
    boardHealth.warning = true;
    missionLogRamDropped++;
    return;
  }

  FlightSample &sample = missionStarted
    ? missionLog[missionLogCount++]
    : prelaunchSamples[prelaunchSampleCount++];
  sample.counter = missionStarted ? ++logCounter : 0;
  sample.ms = millis() - bootMs;
  sample.axMg = static_cast<int16_t>(lroundf(currentAx * 1000.0f));
  sample.ayMg = static_cast<int16_t>(lroundf(currentAy * 1000.0f));
  sample.azMg = static_cast<int16_t>(lroundf(currentAz * 1000.0f));
  // The gyro range is +-4000 dps, more than int16 holds at 0.1 dps resolution: clamp
  // (saturates at +-3276.7 dps in the log) instead of wrapping around.
  sample.gxDps10 = static_cast<int16_t>(lroundf(clampFloat((currentGx - gyroBiasX) * 10.0f, -32768.0f, 32767.0f)));
  sample.gyDps10 = static_cast<int16_t>(lroundf(clampFloat((currentGy - gyroBiasY) * 10.0f, -32768.0f, 32767.0f)));
  sample.gzDps10 = static_cast<int16_t>(lroundf(clampFloat((currentGz - gyroBiasZ) * 10.0f, -32768.0f, 32767.0f)));
  sample.tempCd10 = static_cast<int16_t>(lroundf(currentTemperature * 10.0f));
  sample.pressureHd10 = static_cast<uint16_t>(lroundf(clampFloat(currentPressure * 10.0f, 0.0f, 65535.0f)));
  sample.batteryVoltageMv = static_cast<int16_t>(lroundf(clampFloat(currentVinVoltage * 1000.0f, -32768.0f, 32767.0f)));
  sample.altitudeCm = static_cast<int32_t>(lroundf(clampFloat(currentAltitude * 100.0f, -2.0e9f, 2.0e9f)));
  sample.velocityCms = static_cast<int32_t>(lroundf(clampFloat(currentVerticalVelocity * 100.0f, -2.0e9f, 2.0e9f)));
  sample.rollD10 = static_cast<int16_t>(lroundf(clampFloat(currentRollDeg * 10.0f, -32768.0f, 32767.0f)));
  sample.pitchD10 = static_cast<int16_t>(lroundf(clampFloat(currentPitchDeg * 10.0f, -32768.0f, 32767.0f)));
  sample.healthBits = healthBits();
  sample.state = static_cast<uint8_t>(flightState);
  sample.pyro = pyroLatched ? 1u : 0u;
  sample.baroRejected = baroSampleRejected ? 1u : 0u;
}

void ensureLogsDirectory() {
  if (!storageReady) {
    return;
  }
  FatFS.mkdir("/logs");
}

// Scans /logs for Flight_NNNNNNNN.csv files. Read-only: other files in /logs belong to the
// user and are never deleted.
uint32_t scanFlightLogs(uint32_t &flightLogCount) {
  flightLogCount = 0;
  if (!storageReady) {
    return 0;
  }

  File logsDirectory = FatFS.open("/logs", "r");
  if (!logsDirectory || !logsDirectory.isDirectory()) {
    return 0;
  }

  uint32_t highestFlightNumber = 0;
  File entry = logsDirectory.openNextFile();
  while (entry) {
    const char *entryName = entry.name();
    bool validFlightLog = !entry.isDirectory() && strlen(entryName) == 19 &&
                          strncmp(entryName, "Flight_", 7) == 0 &&
                          strcmp(entryName + 15, ".csv") == 0;
    for (size_t i = 7; validFlightLog && i < 15; i++) {
      validFlightLog = isdigit(static_cast<unsigned char>(entryName[i])) != 0;
    }

    if (validFlightLog) {
      char numberBuffer[9] = {0};
      memcpy(numberBuffer, entryName + 7, 8);
      uint32_t flightNumber = static_cast<uint32_t>(strtoul(numberBuffer, nullptr, 10));
      if (flightNumber > highestFlightNumber) {
        highestFlightNumber = flightNumber;
      }
      flightLogCount++;
    }

    entry.close();
    entry = logsDirectory.openNextFile();
  }

  logsDirectory.close();
  return highestFlightNumber;
}

void cleanupLegacyUsbFiles() {
  if (!storageReady) {
    return;
  }

  FatFS.remove("/SETTINGS.BIN");
  FatFS.remove("/settings.bin");
  FatFS.remove("/USB_MODE_DEMO");
  FatFS.remove("/USB_MODE_DEMO.TXT");
  FatFS.remove("/USB_MODE_DEMO.INI");
  FatFS.remove(LEGACY_HOWTO_FILE);
  FatFS.remove(LEGACY_HOWTO_TEXT_FILE);
  FatFS.remove("/PERSONAL.TXT");
  FatFS.remove("/Profile.bin");
  FatFS.remove("/PROFILE.BIN");
  FatFS.remove(LEGACY_SETTINGS_FILE);
  FatFS.remove(LEGACY_PROFILE_FILE);
  FatFS.remove(LEGACY_DEBUG_FILE);
  FatFS.remove(LEGACY_WEBSITE_FILE);
  FatFS.remove(LEGACY_MOREHELP_FILE);
  FatFS.remove(LEGACY_AUTORUN_FILE);
  FatFS.remove("/SETTINGS.TXT");
  FatFS.remove("/WEBSITE.TXT");
}

bool configureUsbVolumeLabel() {
  if (!storageReady) {
    return false;
  }

  fatfs::f_setlabel("ASTRONAV");

  return true;
}

void ensureUsbInfoFiles() {
  if (!storageReady || usbDriveReady || ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES) {
    return;
  }

  cleanupLegacyUsbFiles();

  File howToFile = FatFS.open(HOWTO_FILE, "w");
  if (!howToFile) {
    boardHealth.warning = true;
    return;
  }

  UsbInfoFiles::writeHowtoText(howToFile);
  howToFile.flush();
  howToFile.close();

  // Avoid autorun metadata to reduce Windows trust/scanning warnings.
  FatFS.remove(AUTORUN_FILE);

  File websiteFile = FatFS.open(WEBSITE_FILE, "w");
  if (!websiteFile) {
    boardHealth.warning = true;
    return;
  }

  UsbInfoFiles::writeWebsiteShortcut(websiteFile);
  websiteFile.flush();
  websiteFile.close();

  persistDeviceProfile();
}

// Runs in the USB task when the host ejects the drive. Only sets flags: the flash may not
// be touched here, and from now on the host is told the medium is gone so it stops reading.
void onUsbStorageUnplug(uint32_t cbData) {
  (void) cbData;
  usbMediaPresent = false;
  if (!usbFlightOverride) {
    usbExitRequested = true;
  }
}

// TEST UNIT READY: false makes the host treat the drive as removed, so it cannot access
// the flash while the firmware leaves USB mode.
bool usbStorageDriveReady(uint32_t cbData) {
  (void) cbData;
  return usbMediaPresent;
}

bool shouldExitUsbModeFromUsbFiles() {
  if (!storageReady) {
    return false;
  }

  return usbExitRequested;
}

void enterFlightModeFromUsb() {
  usbFlightOverride = true;
  usbExitRequested = false;
  usbMediaPresent = false;

  // 1. Detach the drive from the host before the firmware touches the flash. The host and
  //    the firmware must never access the flash translation layer at the same time.
  //    end() removes only the storage interface; USB serial stays available.
  if (usbDriveReady) {
    FatFSUSB.end();
    usbDriveReady = false;
    boardHealth.usbStorageOk = false;
  }

  // 2. Remount so cached FAT data from before the USB session is dropped; the host may
  //    have edited Settings.ini or removed logs.
  if (storageReady) {
    FatFS.end();
    storageReady = FatFS.begin();
    boardHealth.flashFsOk = storageReady;
    Serial.printf("[USB] Drive detached, filesystem remounted ok=%s\n", boolText(storageReady));
  }

  // 3. Now it is safe to read the edited settings and prepare the next flight on the ground.
  if (storageReady) {
    loadFlightSettings();
    if (settingsNeedPersist) {
      persistDeviceProfile();
      settingsNeedPersist = false;
    }
    prepareNextFlightNumber();
  }

  setFlightState(FlightState::Calibrating);
  if (calibratePadOrientation()) {
    resetAltitudeEstimator();
    setFlightState(FlightState::Idle);
  } else {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
    setFlightState(FlightState::Fault);
  }
}

// Debug only, read-only: never touches the pyro or the flight logic. Runs in the sample tick
// and stops by itself after GYRO_TEST_DURATION_MS, or as soon as the board leaves Idle.
void updateGyroTest(bool imuSampleOk, uint32_t now) {
  if (!gyroTestActive) {
    return;
  }
  if (flightState != FlightState::Idle || missionStarted) {
    gyroTestActive = false;
    Serial.printf("[GYROTEST] Cancelled: board left idle (state=%s).\n", stateText(flightState));
    return;
  }
  // Signed: the command can start the test after loop() captured `now`, which would make an
  // unsigned difference wrap around to ~49 days on the first sample.
  int32_t elapsedMs = static_cast<int32_t>(now - gyroTestLastSampleMs);
  if (imuSampleOk && elapsedMs > 0) {
    float dtS = static_cast<float>(elapsedMs) / 1000.0f;
    gyroTestAngleX += (currentGx - gyroBiasX) * dtS;
    gyroTestAngleY += (currentGy - gyroBiasY) * dtS;
    gyroTestAngleZ += (currentGz - gyroBiasZ) * dtS;
    gyroTestLastSampleMs = now;
  }
  if (static_cast<int32_t>(now - gyroTestEndMs) >= 0) {
    gyroTestActive = false;
    Serial.printf("[GYROTEST] Result: x=%.1f y=%.1f z=%.1f deg (turned 90 deg flat on the table: z should read about +-90)\n",
                  gyroTestAngleX, gyroTestAngleY, gyroTestAngleZ);
  }
}

void handleSerialCommand(const char *command) {
  if (!command || !command[0]) {
    return;
  }

  if (pyroTestPendingConfirm && static_cast<int32_t>(millis() - pyroTestConfirmUntilMs) > 0) {
    pyroTestPendingConfirm = false;
  }

  char upperCommand[48] = {0};
  size_t len = strlen(command);
  if (len >= sizeof(upperCommand)) {
    len = sizeof(upperCommand) - 1;
  }

  for (size_t i = 0; i < len; i++) {
    upperCommand[i] = static_cast<char>(toupper(static_cast<unsigned char>(command[i])));
  }
  upperCommand[len] = '\0';

  if (strcmp(upperCommand, "PYROTEST") == 0) {
    // Ground igniter test only: never in USB storage mode and never once a flight started.
    if (flightState != FlightState::Idle || missionStarted) {
      pyroTestPendingConfirm = false;
      Serial.printf("[PYROTEST] Blocked: only allowed in idle on the ground (state=%s).\n", stateText(flightState));
      return;
    }

    if (digitalRead(PIN_PYRO) == HIGH) {
      pyroTestPendingConfirm = false;
      Serial.println("[PYROTEST] Ignored: pyro output is already active.");
      return;
    }

    if (!pyroTestPendingConfirm) {
      pyroTestPendingConfirm = true;
      pyroTestConfirmUntilMs = millis() + PYRO_TEST_CONFIRM_WINDOW_MS;
      Serial.println("[PYROTEST] WARNING: Send PYROTEST again within 5s to ignite pyro output.");
      return;
    }

    pyroTestPendingConfirm = false;
    firePyroTestPulse();
    if (digitalRead(PIN_PYRO) == HIGH) {
      Serial.println("[PYROTEST] WARNING: Pyro output is now active for 1 s.");
    }
    return;
  }

  pyroTestPendingConfirm = false;

  if (strcmp(upperCommand, "GYROTEST") == 0) {
    // Flight mode only (calibrated, in Idle): the gyro bias comes from the pad calibration.
    if (flightState != FlightState::Idle || missionStarted) {
      Serial.printf("[GYROTEST] Blocked: only allowed in idle in flight mode (state=%s).\n", stateText(flightState));
      return;
    }
    gyroTestAngleX = 0.0f;
    gyroTestAngleY = 0.0f;
    gyroTestAngleZ = 0.0f;
    gyroTestLastSampleMs = millis();
    gyroTestEndMs = gyroTestLastSampleMs + GYRO_TEST_DURATION_MS;
    gyroTestActive = true;
    Serial.println("[GYROTEST] Started: within 10 s, slowly turn the board flat on the table by exactly 90 deg (against a straight edge) and hold it still.");
    return;
  }

  if (strcmp(upperCommand, "DUMP_OTP") == 0) {
    printAstroNavOtpSummary();
    return;
  }

  if (strcmp(upperCommand, "DUMP_HEALTH") == 0) {
    Serial.printf("[HEALTH] healthy=%s state=%s power=%s vin_mv=%ld warning=%s critical=%s bits=%u\n",
                  systemHealthy() ? "YES" : "NO",
                  stateText(flightState),
                  powerModeText(powerMode),
                  static_cast<long>(lroundf(currentVinVoltage * 1000.0f)),
                  boardHealth.warning ? "YES" : "NO",
                  boardHealth.critical ? "YES" : "NO",
                  static_cast<unsigned int>(healthBits()));
    Serial.printf("[HEALTH] core=%s heap=%s spi=%s imu_whoami=%s imu_config=%s imu_stream=%s baro=%s flash=%s vin=%s\n",
                  boardHealth.coreTickOk ? "OK" : "FAIL",
                  boardHealth.heapOk ? "OK" : "FAIL",
                  boardHealth.spiOk ? "OK" : "FAIL",
                  boardHealth.imuWhoAmIOk ? "OK" : "FAIL",
                  boardHealth.imuConfigOk ? "OK" : "FAIL",
                  boardHealth.imuStreamOk ? "OK" : "FAIL",
                  boardHealth.baroOk ? "OK" : "FAIL",
                  boardHealth.flashFsOk ? "OK" : "FAIL",
                  boardHealth.vinOk ? "OK" : "FAIL");
    return;
  }

  // Only from USB storage mode. powerMode is re-measured every sample, so a battery reading
  // in the USB range (sag under load, or a 1S LiPo with a USB cable) must not re-run the
  // remount and recalibration from Idle or during a flight.
  if ((strcmp(upperCommand, "FLIGHT") == 0 || strcmp(upperCommand, "EXITUSB") == 0 || strcmp(upperCommand, "ARM") == 0) &&
      flightState == FlightState::UsbMode && !usbFlightOverride) {
    enterFlightModeFromUsb();
  }
}

void resetMissionLogWriter() {
  logSummary = MissionLogSummary();
  missionLogStreaming = false;
  missionLogRowsWritten = 0;
  missionLogRowsDropped = 0;
  missionLogRamDropped = 0;
  missionLogOpenAttempts = 0;
}

// Bytes still free on the filesystem, minus a reserve so Settings.ini can always be rewritten.
uint64_t logStorageAvailableBytes() {
  FSInfo info = {};
  if (!storageReady || !FatFS.info(info) || info.totalBytes <= info.usedBytes) {
    return 0;
  }
  uint64_t freeBytes = info.totalBytes - info.usedBytes;
  return (freeBytes > LOG_STORAGE_RESERVE_BYTES) ? (freeBytes - LOG_STORAGE_RESERVE_BYTES) : 0;
}

void accumulateLogSummary(MissionLogSummary &summary, const FlightSample &sample) {
  float altitudeM = sample.altitudeCm / 100.0f;
  summary.maxAltitudeM = fmaxf(summary.maxAltitudeM, altitudeM);
  summary.maxSpeedMps = fmaxf(summary.maxSpeedMps, fabsf(sample.velocityCms / 100.0f));
  float axG = sample.axMg / 1000.0f;
  float ayG = sample.ayMg / 1000.0f;
  float azG = sample.azMg / 1000.0f;
  summary.maxAccelG = fmaxf(summary.maxAccelG, sqrtf((axG * axG) + (ayG * ayG) + (azG * azG)));
  summary.maxBatteryV = fmaxf(summary.maxBatteryV, sample.batteryVoltageMv / 1000.0f);
  summary.lastAltitudeM = altitudeM;
}

// Writes one CSV row (same columns and formatting as before). Returns the bytes written,
// 0 when the filesystem refused the write.
size_t writeLogRow(File &logFile, const FlightSample &sample, float peakSoFarM) {
  float altitudeM = sample.altitudeCm / 100.0f;
  float actualApogeeDropM = fmaxf(0.0f, peakSoFarM - altitudeM);
  char row[LOG_ROW_MAX_BYTES];
  int length = snprintf(row, sizeof(row),
    "%lu;%lu;%s;%s;%u;%.3f;%.3f;%.3f;%.2f;%.2f;%.2f;%.1f;%.2f;%.2f;%.2f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%u;%u\n",
    static_cast<unsigned long>(sample.counter),
    static_cast<unsigned long>(sample.ms),
    stateText(static_cast<FlightState>(sample.state)),
    powerModeText(powerMode),
    static_cast<unsigned>(sample.pyro),
    sample.axMg / 1000.0f, sample.ayMg / 1000.0f, sample.azMg / 1000.0f,
    sample.gxDps10 / 10.0f, sample.gyDps10 / 10.0f, sample.gzDps10 / 10.0f,
    sample.tempCd10 / 10.0f,
    sample.pressureHd10 / 10.0f,
    sample.batteryVoltageMv / 1000.0f,
    altitudeM,
    sample.velocityCms / 100.0f,
    sample.rollD10 / 10.0f, sample.pitchD10 / 10.0f,
    flightApogeeDropM,
    actualApogeeDropM,
    actualApogeeDropM - flightApogeeDropM,
    static_cast<unsigned>(sample.healthBits),
    static_cast<unsigned>(sample.baroRejected));
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(row)) {
    return 0;
  }
  size_t size = static_cast<size_t>(length);
  return (logFile.write(reinterpret_cast<const uint8_t *>(row), size) == size) ? size : 0;
}

void writeLogSummaryLines(File &logFile, const char *prefix, const MissionLogSummary &summary) {
  logFile.printf("# %sMAX_ALTITUDE_M=%.2f\n", prefix, summary.maxAltitudeM);
  logFile.printf("# %sMAX_SPEED_MPS=%.2f\n", prefix, summary.maxSpeedMps);
  logFile.printf("# %sMAX_ACCEL_G=%.2f\n", prefix, summary.maxAccelG);
  logFile.printf("# %sMAX_BATTERY_V=%.2f\n", prefix, summary.maxBatteryV);
  logFile.printf("# %sESTIMATED_APOGEE_DROP_M=%.2f\n", prefix, flightApogeeDropM);
  float actualDropM = fmaxf(0.0f, summary.maxAltitudeM - summary.lastAltitudeM);
  logFile.printf("# %sACTUAL_APOGEE_DROP_M=%.2f\n", prefix, actualDropM);
  logFile.printf("# %sAPOGEE_DROP_ERROR_M=%.2f\n", prefix, actualDropM - flightApogeeDropM);
  if (apogeeDetected) {
    logFile.printf("# %sDETECTED_APOGEE_M=%.2f\n", prefix, detectedApogeeAltitudeM);
    logFile.printf("# %sDETECTED_APOGEE_MS=%lu\n", prefix, static_cast<unsigned long>(detectedApogeeMs));
  } else {
    logFile.printf("# %sDETECTED_APOGEE_M=none\n", prefix);
    logFile.printf("# %sDETECTED_APOGEE_MS=none\n", prefix);
  }
  logFile.printf("# %sAPOGEE_TRIGGER=%s\n", prefix, apogeeTrigger);
  logFile.printf("# %sLAUNCH_ALTITUDE_M=%.2f\n", prefix, launchAltitudeM);
  logFile.printf("# %sMIN_PYRO_FIRE_HEIGHT_M=%.1f\n", prefix, flightMinPyroFireHeightM);
  const char *pyroReason = pyroBlockedReason;
  if (!apogeeDetected && !pyroLatched) {
    pyroReason = pyroFireAtApogeeAllowed ? "no_apogee" : "disabled";
  }
  logFile.printf("# %sPYRO_BLOCKED_REASON=%s\n", prefix, pyroReason);
  logFile.printf("# %sSAMPLE_GAPS=%lu\n", prefix, static_cast<unsigned long>(sampleGaps));
  logFile.printf("# %sLANDING_TRIGGER=%s\n", prefix, landingTrigger);
  logFile.printf("# %sBARO_REJECTED_SAMPLES=%lu\n", prefix, static_cast<unsigned long>(baroRejectedSamples));
  logFile.printf("# %sPYRO_AT_APOGEE_ENABLED=%s\n", prefix, pyroFireAtApogeeAllowed ? "TRUE" : "FALSE");
  logFile.printf("# %sRAM_DROPPED_SAMPLES=%lu\n", prefix, static_cast<unsigned long>(missionLogRamDropped));
}

void writeLogHeader(File &logFile, bool summaryIsFinal, const MissionLogSummary &summary) {
  logFile.println("# AstroNav Nano Flight Log");
  logFile.printf("# Flight Number: %lu\n", static_cast<unsigned long>(currentFlightNumber));
  logFile.printf("# Boot Time Ms: %lu\n", static_cast<unsigned long>(bootMs));
  if (summaryIsFinal) {
    logFile.printf("# Samples: %u\n", missionLogCount);
  } else {
    logFile.println("# Samples: long flight, see the FINAL_ lines at the end of this file");
  }
  writeLogSummaryLines(logFile, "", summary);
  logFile.println("counter;ms;state;power;pyro;ax_g;ay_g;az_g;gx_dps;gy_dps;gz_dps;temp_c;pressure_hpa;battery_v;altitude_m;vertical_velocity_mps;roll_deg;pitch_deg;estimated_apogee_drop_m;actual_apogee_drop_m;apogee_drop_error_m;health_bits;baro_rejected");
}

MissionLogSummary previewLogSummary() {
  MissionLogSummary preview = logSummary;
  for (uint16_t i = 0; i < missionLogCount; i++) {
    accumulateLogSummary(preview, missionLog[i]);
  }
  return preview;
}

// Moves the oldest `count` RAM samples to the file (as far as storage allows) and removes them
// from RAM. logSummary always includes every sample, also the ones that did not fit on flash.
void writeOldestLogRows(File &logFile, uint16_t count) {
  uint64_t availableBytes = logStorageAvailableBytes();
  for (uint16_t i = 0; i < count; i++) {
    const FlightSample &sample = missionLog[i];
    accumulateLogSummary(logSummary, sample);
    size_t written = 0;
    if (availableBytes >= LOG_ROW_MAX_BYTES) {
      written = writeLogRow(logFile, sample, logSummary.maxAltitudeM);
      availableBytes -= written;
    }
    if (written > 0) {
      missionLogRowsWritten++;
    } else {
      missionLogRowsDropped++;
    }
  }
  if (count < missionLogCount) {
    memmove(&missionLog[0], &missionLog[count], sizeof(FlightSample) * (missionLogCount - count));
  }
  missionLogCount = static_cast<uint16_t>(missionLogCount - count);
}

bool openMissionLogFile(File &logFile, const char *mode) {
  if (usbDriveReady) {
    return false;   // the computer owns the flash while the drive is attached
  }
  ensureLogsDirectory();
  logFile = FatFS.open(missionLogPath, mode);
  if (!logFile) {
    boardHealth.warning = true;
    missionLogOpenAttempts++;
    lastLogStatus = "open_failed";
    return false;
  }
  return true;
}

// Long flights only: once the pyro decision is final and the RAM buffer is mostly full, the
// oldest samples move to flash in small batches so the descent is logged until landing.
// Normal flights never reach this threshold and write nothing to flash before landing.
void streamMissionLogIfNeeded() {
  if (!storageReady || missionLogFlushed || !missionStarted || flightState != FlightState::Descent ||
      missionLogCount < LOG_STREAM_START_SAMPLES || missionLogOpenAttempts >= LOG_MAX_OPEN_ATTEMPTS) {
    return;
  }
  bool pyroDecisionFinal = pyroLatched || !pyroFireAtApogeeAllowed ||
                           static_cast<int32_t>(millis() - pyroRetryUntilMs) >= 0;
  if (!pyroDecisionFinal) {
    return;
  }

  File logFile;
  if (!missionLogStreaming) {
    if (!openMissionLogFile(logFile, "w")) {
      return;
    }
    // Summary so far; the complete values follow as FINAL_ lines when the flight ends.
    writeLogHeader(logFile, false, previewLogSummary());
    missionLogStreaming = true;
  } else if (!openMissionLogFile(logFile, "a")) {
    return;
  }

  uint16_t batch = (missionLogCount < LOG_STREAM_BATCH_SAMPLES) ? missionLogCount : LOG_STREAM_BATCH_SAMPLES;
  writeOldestLogRows(logFile, batch);
  logFile.close();
}

void flushMissionLogToFlash() {
  if (!storageReady || missionLogFlushed || missionLogOpenAttempts >= LOG_MAX_OPEN_ATTEMPTS) {
    return;
  }
  if (!missionLogStreaming && missionLogCount == 0) {
    return;
  }

  File logFile;
  if (missionLogStreaming) {
    if (!openMissionLogFile(logFile, "a")) {
      return;
    }
    writeOldestLogRows(logFile, missionLogCount);
    logFile.println("# --- final summary (flight longer than the RAM buffer) ---");
    logFile.printf("# FINAL_SAMPLES=%lu\n", static_cast<unsigned long>(missionLogRowsWritten + missionLogRowsDropped));
    logFile.printf("# FINAL_STORAGE_DROPPED_SAMPLES=%lu\n", static_cast<unsigned long>(missionLogRowsDropped));
    writeLogSummaryLines(logFile, "FINAL_", logSummary);
  } else {
    // Normal flight: summary first, then all rows, same layout as before.
    if (!openMissionLogFile(logFile, "w")) {
      return;
    }
    writeLogHeader(logFile, true, previewLogSummary());
    writeOldestLogRows(logFile, missionLogCount);
    if (missionLogRowsDropped > 0) {
      logFile.printf("# STORAGE_FULL: %lu samples could not be saved\n", static_cast<unsigned long>(missionLogRowsDropped));
    }
  }

  logFile.flush();
  logFile.close();

  peakAltitude = logSummary.maxAltitudeM;
  peakFlightSpeedMps = logSummary.maxSpeedMps;
  peakAccelMagG = logSummary.maxAccelG;
  if (missionLogRowsDropped > 0) {
    lastLogStatus = "truncated_storage_full";
    boardHealth.warning = true;
    Serial.printf("[LOG] Storage full: %lu samples not saved.\n", static_cast<unsigned long>(missionLogRowsDropped));
  } else {
    lastLogStatus = "ok";
  }
  missionLogFlushed = true;
}

void setStatusLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void setLedProfile(LedProfile profile) {
  // Pattern: blinking = needs attention (calibrating, landed), breathing = waiting on the
  // ground (boot, idle, USB), solid = in flight.
  float pulse = 1.0f;
  if (profile == LedProfile::Calibrating) {
    pulse = (millis() % 500UL) < 250UL ? 1.0f : 0.0f;
  } else if (profile == LedProfile::Landed) {
    pulse = (millis() % 400UL) < 200UL ? 1.0f : 0.0f;
  } else if (profile == LedProfile::Boot || profile == LedProfile::Idle || profile == LedProfile::Usb) {
    uint32_t now = millis();
    float phase = static_cast<float>((now % 2000UL)) / 2000.0f;
    pulse = 0.10f + 0.90f * (0.5f + 0.5f * sinf(phase * 6.2831853f));
  }

  auto scaleAndSet = [&](uint8_t r, uint8_t g, uint8_t b) {
    setStatusLED(static_cast<uint8_t>(r * pulse), static_cast<uint8_t>(g * pulse), static_cast<uint8_t>(b * pulse));
  };

  switch (profile) {
    case LedProfile::Boot:
      scaleAndSet(255, 255, 255);
      break;
    case LedProfile::Calibrating:
      scaleAndSet(255, 255, 0);
      break;
    case LedProfile::Idle:
      scaleAndSet(0, 255, 0);
      break;
    case LedProfile::Usb:
      scaleAndSet(0, 128, 255);
      break;
    case LedProfile::Boost:
      scaleAndSet(255, 80, 0);
      break;
    case LedProfile::Coast:
      scaleAndSet(160, 0, 255);
      break;
    case LedProfile::Descent:
      scaleAndSet(0, 255, 255);
      break;
    case LedProfile::Landed:
      scaleAndSet(255, 0, 255);
      break;
    case LedProfile::Fault:
    default:
      setStatusLED(255, 0, 0);
      break;
  }
}

void updateStatusLed() {
  if (boardHealth.critical || lowVoltageFault || sensorFault) {
    static uint32_t lastFaultToggleMs = 0;
    static bool faultLedOn = false;
    uint32_t now = millis();
    if (now - lastFaultToggleMs >= 100) {
      lastFaultToggleMs = now;
      faultLedOn = !faultLedOn;
    }
    setStatusLED(faultLedOn ? 255 : 0, 0, 0);
    return;
  }

  if (millis() < bootPulseUntilMs) {
    setLedProfile(LedProfile::Boot);
    return;
  }

  switch (flightState) {
    case FlightState::Booting:
      setLedProfile(LedProfile::Boot);
      break;
    case FlightState::Calibrating:
      setLedProfile(LedProfile::Calibrating);
      break;
    case FlightState::Idle:
    case FlightState::Armed:   // not used by the flight logic; shown like Idle
      setLedProfile(LedProfile::Idle);
      break;
    case FlightState::UsbMode:
      setLedProfile(LedProfile::Usb);
      break;
    case FlightState::Boost:
      setLedProfile(LedProfile::Boost);
      break;
    case FlightState::Coast:
      setLedProfile(LedProfile::Coast);
      break;
    case FlightState::Descent:
      setLedProfile(LedProfile::Descent);
      break;
    case FlightState::Landed:
      setLedProfile(LedProfile::Landed);
      break;
    case FlightState::Fault:
    default:
      {
        uint32_t now = millis();
        static uint32_t lastFaultToggleMs = 0;
        static bool faultLedOn = false;
        if (now - lastFaultToggleMs >= 150) {
          lastFaultToggleMs = now;
          faultLedOn = !faultLedOn;
        }
        setStatusLED(faultLedOn ? 255 : 0, 0, 0);
      }
      break;
  }
}

void printStartupSummary() {
  Serial.printf("[BOOT] mode=%s state=%s healthy=%s\n",
                runtimeModeText(),
                stateText(flightState),
                boolText(systemHealthy()));
}

void printRuntimeSummary() {
  Serial.printf("[RUNTIME] mode=%s state=%s healthy=%s\n",
                runtimeModeText(),
                stateText(flightState),
                boolText(systemHealthy()));
}

bool missionAtSafeGroundState() {
  return flightState == FlightState::Landed || flightState == FlightState::Fault;
}

float historySpan(const float *history, uint8_t count) {
  if (count == 0) {
    return 0.0f;
  }
  float lowest = history[0];
  float highest = history[0];
  for (uint8_t i = 1; i < count; i++) {
    lowest = fminf(lowest, history[i]);
    highest = fmaxf(highest, history[i]);
  }
  return highest - lowest;
}

float altitudeSamplesAgo(uint8_t samplesAgo) {
  uint8_t index = static_cast<uint8_t>((altitudeHistoryIndex + SENSOR_HISTORY_SAMPLES - 1 - samplesAgo) % SENSOR_HISTORY_SAMPLES);
  return altitudeHistory[index];
}

uint8_t baroSamplesSinceAccepted = 0;
float anchorVerticalVelocity = 0.0f;

void resetAltitudeEstimator() {
  memset(altitudeHistory, 0, sizeof(altitudeHistory));
  memset(accelMagHistory, 0, sizeof(accelMagHistory));
  altitudeHistoryIndex = 0;
  altitudeHistoryCount = 0;
  accelHistoryIndex = 0;
  accelHistoryCount = 0;
  lastAcceptedRawAltitude = 0.0f;
  baroConsecutiveRejects = 0;
  baroSamplesSinceAccepted = 0;
  anchorVerticalVelocity = 0.0f;
  baroSampleRejected = false;
  filteredAltitude = 0.0f;
  currentAltitude = 0.0f;
  currentVerticalVelocity = 0.0f;
}

// Turns raw pressure into altitude and vertical speed with as little lag as possible:
// physically impossible jumps are rejected first, so only light smoothing is needed.
void updateAltitudeEstimate() {
  const float dt = SAMPLE_PERIOD_MS / 1000.0f;
  baroSampleRejected = false;
  if (baroSamplesSinceAccepted < 255) {
    baroSamplesSinceAccepted++;
  }

  // Extrapolate with the velocity frozen at the last accepted sample: the live velocity is
  // computed from these predictions and would otherwise feed back and run away.
  uint8_t extrapolationSamples = (baroSamplesSinceAccepted < BARO_MAX_EXTRAPOLATION_SAMPLES)
    ? baroSamplesSinceAccepted
    : BARO_MAX_EXTRAPOLATION_SAMPLES;
  float predicted = lastAcceptedRawAltitude + anchorVerticalVelocity * dt * extrapolationSamples;
  float rawAltitude = predicted;
  float measured = (baroSampleFresh && currentPressure > 0.0f) ? altitudeFromPressure(currentPressure) : NAN;

  if (isfinite(measured)) {
    float allowed = flightMaxPlausibleSpeedMps * dt * baroSamplesSinceAccepted + BARO_SPIKE_NOISE_MARGIN_M;
    bool plausible = fabsf(measured - predicted) <= allowed;
    if (plausible || baroConsecutiveRejects >= BARO_SPIKE_MAX_CONSECUTIVE_REJECTS) {
      if (!plausible) {
        // The jump persisted, so it is real (or our estimate was off): re-anchor on it and
        // shift the history so the velocity slope does not see an artificial step.
        float offset = measured - filteredAltitude;
        for (uint8_t i = 0; i < altitudeHistoryCount; i++) {
          altitudeHistory[i] += offset;
        }
        filteredAltitude = measured;
      }
      rawAltitude = measured;
      lastAcceptedRawAltitude = measured;
      baroConsecutiveRejects = 0;
      baroSamplesSinceAccepted = 0;
    } else {
      baroConsecutiveRejects++;
      baroSampleRejected = true;
      if (missionStarted) {
        baroRejectedSamples++;
      }
    }
  }

  filteredAltitude += ALTITUDE_FILTER_ALPHA * (rawAltitude - filteredAltitude);
  currentAltitude = filteredAltitude;

  altitudeHistory[altitudeHistoryIndex] = currentAltitude;
  altitudeHistoryIndex = static_cast<uint8_t>((altitudeHistoryIndex + 1) % SENSOR_HISTORY_SAMPLES);
  if (altitudeHistoryCount < SENSOR_HISTORY_SAMPLES) {
    altitudeHistoryCount++;
  }

  uint8_t window = (altitudeHistoryCount - 1 < VELOCITY_WINDOW_SAMPLES)
    ? static_cast<uint8_t>(altitudeHistoryCount - 1)
    : VELOCITY_WINDOW_SAMPLES;
  currentVerticalVelocity = (window == 0)
    ? 0.0f
    : (currentAltitude - altitudeSamplesAgo(window)) / (window * dt);
  if (baroSamplesSinceAccepted == 0) {
    anchorVerticalVelocity = currentVerticalVelocity;
  }
}

// Apogee pyro with software safety gates. PYROTEST uses firePyroTestPulse() instead.
void tryApogeePyro(bool baroReliable) {
  if (pyroLatched) {
    return;
  }
  if (!pyroFireAtApogeeAllowed) {
    pyroBlockedReason = "disabled";
    return;
  }
  // Parachute timing depends on the barometer: never fire on a stale or rejected reading.
  if (!baroReliable || baroSampleRejected || baroSamplesSinceAccepted != 0) {
    pyroBlockedReason = "baro_unreliable";
    return;
  }
  // Protects people handling an armed board: below this height the pyro never fires.
  if (peakAltitude - launchAltitudeM < flightMinPyroFireHeightM) {
    pyroBlockedReason = "below_min_height";
    return;
  }
  pyroBlockedReason = "none";
  firePyro();
}

void updateFlightStateFromSamples() {
  float accelMag = magnitude3(currentAx, currentAy, currentAz);
  float gravityProjection = (currentAx * gravityAxisX) + (currentAy * gravityAxisY) + (currentAz * gravityAxisZ);
  currentRollDeg = atan2f(currentAy, currentAz) * 57.2957795f;
  currentPitchDeg = atan2f(-currentAx, sqrtf((currentAy * currentAy) + (currentAz * currentAz))) * 57.2957795f;

  if (currentTemperature > -100.0f) {
    filteredTemperature = (filteredTemperature * 0.80f) + (currentTemperature * 0.20f);
  }

  updateAltitudeEstimate();
  // Peak only from a barometer reading accepted this tick. During a dropout the estimate is
  // extrapolated at the last climb rate, which overshoots while the rocket slows down; a peak
  // taken from that could make the drop-from-peak trigger fire while still climbing.
  bool baroAcceptedThisSample = baroSampleFresh && baroSamplesSinceAccepted == 0;
  if ((flightState == FlightState::Boost || flightState == FlightState::Coast) && baroAcceptedThisSample) {
    if (currentAltitude > peakAltitude) {
      peakAltitude = currentAltitude;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = millis();
    } else {
      uint32_t holdDeltaMs = millis() - apogeePeakHoldStartMs;
      if (holdDeltaMs <= APOGEE_FIRE_MAX_MS) {
        if (currentAltitude >= peakAltitude - APOGEE_PEAK_HOLD_BAND_M &&
            fabsf(currentVerticalVelocity) <= APOGEE_PEAK_HOLD_MAX_SPEED_MPS) {
          if (apogeePeakHoldCount < 255) {
            apogeePeakHoldCount++;
          }
        } else {
          apogeePeakHoldCount = 0;
          apogeePeakHoldStartMs = millis();
        }
      } else {
        apogeePeakHoldCount = 0;
        apogeePeakHoldStartMs = millis();
      }
    }
  }

  if (missionStarted) {
    float verticalSpeedAbs = fabsf(currentVerticalVelocity);
    if (verticalSpeedAbs > peakFlightSpeedMps) {
      peakFlightSpeedMps = verticalSpeedAbs;
    }
    if (baroAcceptedThisSample && currentVerticalVelocity > peakClimbRateMps) {
      peakClimbRateMps = currentVerticalVelocity;
    }
  }

  bool imuReliable = boardHealth.imuStreamOk && isfinite(accelMag) && isfinite(gravityProjection);
  if (imuReliable && accelMag > peakAccelMagG) {
    peakAccelMagG = accelMag;
  }
  if (imuReliable) {
    accelMagHistory[accelHistoryIndex] = accelMag;
    accelHistoryIndex = static_cast<uint8_t>((accelHistoryIndex + 1) % SENSOR_HISTORY_SAMPLES);
    if (accelHistoryCount < SENSOR_HISTORY_SAMPLES) {
      accelHistoryCount++;
    }
  }
  bool baroReliable = boardHealth.baroOk && isfinite(currentAltitude) && isfinite(currentVerticalVelocity);

  if (flightState == FlightState::Idle) {
    bool launchDetected = false;
    if (imuReliable) {
      launchDetected = (accelMag > flightLaunchThresholdG);
    } else if (baroReliable) {
      launchDetected = (currentVerticalVelocity > 1.0f) && (currentAltitude > 1.0f);
    }

    if (launchDetected) {
      if (launchConfirmCount < 255) {
        launchConfirmCount++;
      }
    } else {
      launchConfirmCount = 0;
    }

    if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
      launchMs = millis();
      launchConfirmCount = 0;
      launchAltitudeM = currentAltitude;
      peakAltitude = currentAltitude;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = millis();
      startMissionLog();
      apogeeConfirmCount = 0;
      landingConfirmCount = 0;
      setFlightState(FlightState::Boost);
    }
  } else if (flightState == FlightState::Boost || flightState == FlightState::Coast) {
    if (flightState == FlightState::Boost) {
      bool boostDecayDetected = false;
      if (imuReliable) {
        boostDecayDetected = (accelMag < 1.15f);
      } else if (baroReliable) {
        boostDecayDetected = (currentVerticalVelocity <= 0.5f) && (currentAltitude > 2.0f);
      }

      if (boostDecayDetected) {
        if (boostDecayConfirmCount < 255) {
          boostDecayConfirmCount++;
        }
      } else {
        boostDecayConfirmCount = 0;
      }

      if (boostDecayConfirmCount >= BOOST_DECAY_CONFIRM_SAMPLES) {
        boostDecayConfirmCount = 0;
        setFlightState(FlightState::Coast);
      }
    }

    // Apogee needs a trustworthy barometer. Without it no apogee is declared, so the
    // pyro cannot fire on a guess; landing detection still saves the log.
    bool pastMinFlightTime = (millis() - launchMs) >= LAUNCH_MIN_TIME_MS;
    float altitudeDropFromPeakM = peakAltitude - currentAltitude;
    bool fallingFree = imuReliable
      ? (accelMag <= APOGEE_ACCEL_MAX_G)
      : (currentVerticalVelocity <= APOGEE_BARO_ONLY_DESCENT_MPS);
    bool apogeeGate = baroReliable && !baroSampleRejected && pastMinFlightTime && fallingFree;

    // Primary: climb has stopped. Backup: the drop from the peak is already clearly
    // larger than noise, in case the velocity estimate lags or is noisy.
    bool velocityApogee = apogeeGate && currentVerticalVelocity <= 0.0f &&
                          altitudeDropFromPeakM >= APOGEE_MIN_DROP_M;
    bool dropApogee = apogeeGate && altitudeDropFromPeakM >= flightApogeeDropM;

    if (velocityApogee || dropApogee) {
      if (apogeeConfirmCount < 255) {
        apogeeConfirmCount++;
      }
    } else {
      apogeeConfirmCount = 0;
    }

    // The estimate only speeds up confirmation near the expected apogee; it never blocks it.
    float peakAboveLaunchM = peakAltitude - launchAltitudeM;
    bool nearExpectedApogee = peakAboveLaunchM >= flightExpectedApogeeLowM &&
                              peakAboveLaunchM <= flightExpectedApogeeHighM;
    uint8_t requiredApogeeSamples = nearExpectedApogee ? APOGEE_CONFIRM_SAMPLES_NEAR_EXPECTED : APOGEE_CONFIRM_SAMPLES;

    bool peakHoldFire = baroReliable && pastMinFlightTime &&
                        (!imuReliable || accelMag <= APOGEE_ACCEL_MAX_G) &&
                        apogeePeakHoldCount >= APOGEE_HOLD_SAMPLES &&
                        (millis() - apogeePeakHoldStartMs) <= APOGEE_FIRE_MAX_MS;

    // Fallback without the free-fall check: if the IMU never reads free fall (tumbling,
    // spinning, board far from the centre of gravity), a clear, sustained barometer descent
    // still declares apogee. A late deployment is safer than none. Not while the motor
    // could still be pushing (IMU above the launch threshold).
    bool notUnderThrust = !imuReliable || accelMag < flightLaunchThresholdG;
    bool descentFallback = baroReliable && !baroSampleRejected && pastMinFlightTime && notUnderThrust &&
                           peakClimbRateMps >= APOGEE_FALLBACK_MIN_CLIMB_MPS &&
                           currentVerticalVelocity <= APOGEE_FALLBACK_DESCENT_MPS &&
                           altitudeDropFromPeakM >= fmaxf(APOGEE_FALLBACK_MIN_DROP_M, flightApogeeDropM);
    if (descentFallback) {
      if (apogeeFallbackConfirmCount < 255) {
        apogeeFallbackConfirmCount++;
      }
    } else {
      apogeeFallbackConfirmCount = 0;
    }
    bool fallbackFire = apogeeFallbackConfirmCount >= APOGEE_FALLBACK_CONFIRM_SAMPLES;

    if (apogeeConfirmCount >= requiredApogeeSamples || peakHoldFire || fallbackFire) {
      apogeeDetected = true;
      detectedApogeeAltitudeM = peakAltitude;
      detectedApogeeMs = millis() - bootMs;
      if (apogeeConfirmCount >= requiredApogeeSamples) {
        apogeeTrigger = velocityApogee ? "velocity" : "drop";
      } else if (peakHoldFire) {
        apogeeTrigger = "peak_hold";
      } else {
        apogeeTrigger = "baro_descent";
      }
      setFlightState(FlightState::Descent);
      pyroRetryUntilMs = millis() + PYRO_RETRY_WINDOW_MS;
      tryApogeePyro(baroReliable);
      apogeeConfirmCount = 0;
      apogeeFallbackConfirmCount = 0;
      apogeePeakHoldCount = 0;
      apogeePeakHoldStartMs = 0;
    }

  } else if (flightState == FlightState::Descent && !pyroLatched &&
             static_cast<int32_t>(pyroRetryUntilMs - millis()) > 0) {
    tryApogeePyro(baroReliable);
  }

  bool postLaunchState = flightState == FlightState::Boost ||
                         flightState == FlightState::Coast ||
                         flightState == FlightState::Descent;
  if (postLaunchState && missionStarted && !missionLogFlushed) {
    // Resting on the ground reads ~1 g with a steady signal. Free fall reads ~0 g, and a
    // steady descent also reads ~1 g, so the IMU alone can never prove a landing:
    // the barometer must agree the altitude has stopped changing.
    float gyroMag = magnitude3(currentGx - gyroBiasX, currentGy - gyroBiasY, currentGz - gyroBiasZ);
    float accelSpan = historySpan(accelMagHistory, accelHistoryCount);
    float altitudeSpan = historySpan(altitudeHistory, altitudeHistoryCount);
    bool accelWindowFull = accelHistoryCount >= SENSOR_HISTORY_SAMPLES;
    bool altitudeWindowFull = altitudeHistoryCount >= SENSOR_HISTORY_SAMPLES;

    bool imuCalm = imuReliable && accelWindowFull &&
                   fabsf(accelMag - 1.0f) <= LANDING_IMU_CALM_ACCEL_G &&
                   accelSpan <= LANDING_IMU_CALM_ACCEL_SPAN_G &&
                   gyroMag <= LANDING_IMU_CALM_GYRO_DPS;
    bool imuMinimal = imuReliable && accelWindowFull &&
                      fabsf(accelMag - 1.0f) <= LANDING_IMU_MINIMAL_ACCEL_G &&
                      accelSpan <= LANDING_IMU_MINIMAL_ACCEL_SPAN_G &&
                      gyroMag <= LANDING_IMU_MINIMAL_GYRO_DPS;
    bool baroCalm = baroReliable && altitudeWindowFull && altitudeSpan <= LANDING_BARO_CALM_SPAN_M;
    bool baroMinimal = baroReliable && altitudeWindowFull && altitudeSpan <= LANDING_BARO_MINIMAL_SPAN_M;

    bool landingCandidate = false;
    const char *candidateTrigger = "none";
    uint8_t requiredLandingSamples = (flightState == FlightState::Descent)
      ? LANDING_CONFIRM_SAMPLES
      : LANDING_CONFIRM_SAMPLES_WITHOUT_APOGEE;
    if (baroReliable && imuReliable) {
      // One sensor fully calm, the other at most slightly moving (wind). Still swinging
      // (e.g. hanging in a tree) keeps both above "minimal", so we wait until it settles.
      landingCandidate = (imuCalm && baroMinimal) || (baroCalm && imuMinimal);
      candidateTrigger = "imu_baro";
    } else if (baroReliable) {
      landingCandidate = baroCalm;
      candidateTrigger = "baro_only";
    } else if (imuReliable) {
      landingCandidate = imuCalm;
      candidateTrigger = "imu_only";
      requiredLandingSamples = LANDING_CONFIRM_SAMPLES_SINGLE_IMU;
    }

    if (landingCandidate) {
      if (landingConfirmCount < 255) {
        landingConfirmCount++;
      }
    } else {
      landingConfirmCount = 0;
    }

    // Came to rest before any apogee and the barometer never saw a real climb: this was a
    // bump while handling, not a flight. Only with a working barometer, so a real flight
    // whose barometer failed still lands and keeps its log.
    bool noRealClimb = !apogeeDetected && baroReliable &&
                       (flightState == FlightState::Boost || flightState == FlightState::Coast) &&
                       (peakAltitude - launchAltitudeM) < FALSE_LAUNCH_MAX_CLIMB_M;

    if (landingConfirmCount >= requiredLandingSamples) {
      if (noRealClimb && abortFalseLaunch()) {
        return;
      }
      landingTrigger = candidateTrigger;
      setFlightState(FlightState::Landed);
    } else {
      // Last resort so the flight log is always saved, even if both sensors failed.
      // Scales with the real peak, so a flight far above the estimate is not cut short,
      // and waits while the barometer still clearly shows a descent.
      uint32_t sinceLaunchMs = millis() - launchMs;
      float heightForTimeoutM = fmaxf(flightExpectedApogeeHighM, peakAltitude - launchAltitudeM);
      float timeoutS = heightForTimeoutM / LANDING_SLOWEST_DESCENT_MPS + 60.0f;
      uint32_t timeoutMs = static_cast<uint32_t>(clampFloat(timeoutS * 1000.0f,
                                                            static_cast<float>(LANDING_TIMEOUT_MIN_MS),
                                                            static_cast<float>(LANDING_TIMEOUT_MAX_MS)));
      bool stillDescending = baroReliable && altitudeWindowFull && altitudeSpan > LANDING_BARO_MINIMAL_SPAN_M;
      if ((sinceLaunchMs >= timeoutMs && !stillDescending) || sinceLaunchMs >= LANDING_TIMEOUT_MAX_MS) {
        if (noRealClimb && abortFalseLaunch()) {
          return;
        }
        landingTrigger = "timeout";
        setFlightState(FlightState::Landed);
      }
    }
  }
}

void setup() {
  // Drive the pyro and servo outputs low before anything else (serial wait, OTP, sensors),
  // so they are never left floating during boot.
  pinMode(PIN_PYRO, OUTPUT);
  digitalWrite(PIN_PYRO, LOW);
  pinMode(PIN_SERVO_1, OUTPUT);
  pinMode(PIN_SERVO_2, OUTPUT);
  digitalWrite(PIN_SERVO_1, LOW);
  digitalWrite(PIN_SERVO_2, LOW);

  Serial.begin(115200);
  led.begin();
  led.setBrightness(40);
  setStatusLED(0, 0, 0);
  analogReadResolution(12);

  bootMs = millis();
  bootPulseUntilMs = bootMs + BOOT_PULSE_MS;
  nextSampleMs = bootMs;
  nextLedMs = bootMs;

  uint32_t startWait = millis();
  while (!Serial && (millis() - startWait < 1500)) {
    delay(10);
  }

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  AstroNav flight firmware");
  Serial.println("=============================================");
  Serial.printf("[BOOT] firmware=%s reset_reason=%s\n", AUTO_VERSION, resetReasonText());

  initializeAstroNavOtp();
  printAstroNavOtpSummary();
  Serial.println("[BOOT] stage=otp_complete");

  pinMode(PIN_CS_IMU, OUTPUT);
  pinMode(PIN_CS_BMP, OUTPUT);
  pinMode(PIN_VIN_SENSE, INPUT);
  digitalWrite(PIN_CS_IMU, HIGH);
  digitalWrite(PIN_CS_BMP, HIGH);

  digitalWrite(PIN_CS_BMP, LOW);
  delay(15);
  digitalWrite(PIN_CS_BMP, HIGH);
  delay(15);

  SPI.setRX(PIN_MISO);
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  Serial.println("[BOOT] stage=spi_begin");
  SPI.begin();
  Serial.println("[BOOT] stage=spi_complete");

  boardHealth.coreTickOk = testCoreTick();
  boardHealth.heapOk = testHeap();
  boardHealth.vinOk = readInputVoltage(currentVinVoltage);
  powerMode = classifyPowerSource(currentVinVoltage);
  if (powerMode == PowerMode::Unknown) {
    delay(100);
    boardHealth.vinOk = readInputVoltage(currentVinVoltage);
    powerMode = classifyPowerSource(currentVinVoltage);
  }
  Serial.printf("[BOOT] stage=vin_complete ok=%s voltage=%.3f power=%s\n",
                boolText(boardHealth.vinOk),
                currentVinVoltage,
                powerModeText(powerMode));
  if (powerMode == PowerMode::LiPo1S || powerMode == PowerMode::LiPo2S) {
    detectedBatteryMode = powerMode;
  }
  if (!boardHealth.vinOk || powerMode == PowerMode::Unknown) {
    boardHealth.warning = true;
  }

  Serial.println("[BOOT] stage=baro_begin");
  baro.beginSPI(PIN_CS_BMP, 250000);
  delay(30);
  boardHealth.baroOk = readBaroSample(currentTemperature, currentPressure);
  Serial.printf("[BOOT] stage=baro_complete ok=%s\n", boolText(boardHealth.baroOk));

  Serial.println("[BOOT] stage=imu_begin");
  bool imuBootOk = configureImu();
  imuInitialized = boardHealth.spiOk && boardHealth.imuWhoAmIOk && boardHealth.imuConfigOk;
  if (imuBootOk) {
    boardHealth.imuStreamOk = verifyImuStream();
    readImuFrame(currentAx, currentAy, currentAz, currentGx, currentGy, currentGz);
  }
  Serial.printf("[BOOT] stage=imu_complete ok=%s\n", boolText(imuBootOk));

  Serial.println("[BOOT] stage=fatfs_begin");
  storageReady = ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES ? false : FatFS.begin();
  boardHealth.flashFsOk = storageReady;
  Serial.printf("[BOOT] stage=fatfs_complete ok=%s\n", boolText(storageReady));
  if (ASTRONAV_RECOVERY_SKIP_FILESYSTEM_WRITES) {
    Serial.println("[BOOT] stage=filesystem_access_skipped_recovery");
  } else if (storageReady) {
    Serial.println("[BOOT] stage=volume_label_begin");
    configureUsbVolumeLabel();
    Serial.println("[BOOT] stage=volume_label_complete");
    Serial.println("[BOOT] stage=logs_directory_begin");
    ensureLogsDirectory();
    Serial.println("[BOOT] stage=logs_directory_complete");
    Serial.println("[BOOT] stage=settings_begin");
    loadFlightSettings();
    Serial.println("[BOOT] stage=settings_complete");
    Serial.println("[BOOT] stage=profile_begin");
    loadOrCreateDeviceProfile();
    Serial.println("[BOOT] stage=profile_complete");
    Serial.println("[BOOT] stage=serial_begin");
    ensureDeviceSerial();
    Serial.println("[BOOT] stage=serial_complete");
    syncAssetsFromDeviceProfile();
    // After the profile is loaded, so the scanned LOG_COUNT is not overwritten.
    prepareNextFlightNumber();
    // Keep FLASH_STAMP aligned with the currently flashed firmware build.
    buildFlashStamp(deviceProfile.flashStamp, sizeof(deviceProfile.flashStamp));
    // Settings.ini is written once per boot: by ensureUsbInfoFiles() before the USB drive is
    // exported, or at the end of setup() in flight mode. That write also covers any
    // corrected settings from loadFlightSettings().
    settingsNeedPersist = false;
  }

  Serial.println("[BOOT] stage=after_persist_return");
  Serial.printf("[BOOT] stage=post_persist power=%s critical=%s\n",
                powerModeText(powerMode),
                boardHealth.critical ? "YES" : "NO");
  if (powerMode == PowerMode::USB && storageReady && !usbFlightOverride) {
    Serial.println("[BOOT] stage=usb_begin");
    // Write the drive's files before exporting it: once the host has the drive, the firmware
    // must not write the flash. USB_STORAGE_OK is written as the expected state.
    boardHealth.usbStorageOk = true;
    ensureUsbInfoFiles();
    usbMediaPresent = true;
    FatFSUSB.onUnplug(onUsbStorageUnplug);
    FatFSUSB.driveReady(usbStorageDriveReady);
    usbDriveReady = FatFSUSB.begin();
    boardHealth.usbStorageOk = usbDriveReady;
    if (!usbDriveReady) {
      boardHealth.warning = true;
    }
    Serial.printf("[BOOT] stage=usb_complete ok=%s\n", boolText(usbDriveReady));
  }

  Serial.println("[BOOT] stage=health_checks_begin");
  if (!boardHealth.coreTickOk || !boardHealth.heapOk || !boardHealth.spiOk ||
      !boardHealth.imuWhoAmIOk || !boardHealth.imuConfigOk || !boardHealth.imuStreamOk ||
      !boardHealth.baroOk) {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
  }

  if (powerMode == PowerMode::Unknown) {
    boardHealth.critical = true;
    recordFaultAndSafeStop();
  }

  Serial.printf("[BOOT] stage=health_checks_complete critical=%s power=%s\n",
                boardHealth.critical ? "YES" : "NO",
                powerModeText(powerMode));
  printStartupSummary();

  if (boardHealth.critical) {
    setFlightState(FlightState::Fault);
  } else if (powerMode == PowerMode::USB && !usbFlightOverride) {
    setFlightState(FlightState::UsbMode);
  } else {
    setFlightState(FlightState::Calibrating);
    if (!calibratePadOrientation()) {
      boardHealth.critical = true;
      recordFaultAndSafeStop();
      setFlightState(FlightState::Fault);
    } else {
      resetAltitudeEstimator();
      setFlightState(FlightState::Idle);
    }
  }

  // Skipped while the USB drive is attached (checked inside persistDeviceProfile): in USB
  // mode the file was already written before the drive was exported.
  if (storageReady && !usbDriveReady) {
    Serial.println("[BOOT] stage=profile_persist_begin");
    persistDeviceProfile();
    Serial.println("[BOOT] stage=profile_persist_complete");
  }

  updateStatusLed();
  Serial.println("=============================================");
}

void loop() {
  uint32_t now = millis();

  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r' || incoming == '\n') {
      if (serialCommandLength > 0) {
        serialCommandBuffer[serialCommandLength] = '\0';
        handleSerialCommand(serialCommandBuffer);
        serialCommandLength = 0;
      }
    } else if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = incoming;
    }
  }

  if (flightState == FlightState::UsbMode && !usbFlightOverride && shouldExitUsbModeFromUsbFiles()) {
    enterFlightModeFromUsb();
  }

  if (now >= nextLedMs) {
    updateStatusLed();
    nextLedMs = now + LED_PERIOD_MS;
  }

  updatePyroOutput();

  if (boardHealth.critical) {
    setFlightState(FlightState::Fault);
  }

  if (static_cast<int32_t>(now - nextSampleMs) >= 0) {
    // After a stall, resume on schedule instead of replaying a burst of samples with
    // the same timestamp (that would distort the fixed-interval velocity estimate).
    if (now - nextSampleMs >= SAMPLE_PERIOD_MS) {
      if (missionStarted) {
        sampleGaps++;
      }
      nextSampleMs = now + SAMPLE_PERIOD_MS;
    } else {
      nextSampleMs += SAMPLE_PERIOD_MS;
    }

    if (readInputVoltage(currentVinVoltage)) {
      powerMode = classifyPowerSource(currentVinVoltage);
    } else {
      boardHealth.warning = true;
    }

    if (isLiPoVoltageTooLow()) {
      boardHealth.warning = true;
      lowVoltageFault = true;
    }

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    bool imuSampleOk = imuInitialized && readImuFrame(ax, ay, az, gx, gy, gz);
    if (imuSampleOk) {
      currentAx = ax;
      currentAy = ay;
      currentAz = az;
      currentGx = gx;
      currentGy = gy;
      currentGz = gz;
      recordSensorResult(true, boardHealth.imuStreamOk, imuFailureCount, imuFailureWindowStartMs);
    } else if (imuInitialized) {
      recordSensorResult(false, boardHealth.imuStreamOk, imuFailureCount, imuFailureWindowStartMs);
    }

    float temp = 0.0f;
    float pressure = 0.0f;
    baroSampleFresh = readBaroSample(temp, pressure);
    if (baroSampleFresh) {
      currentTemperature = temp;
      currentPressure = pressure;
      recordSensorResult(true, boardHealth.baroOk, baroFailureCount, baroFailureWindowStartMs);
    } else {
      recordSensorResult(false, boardHealth.baroOk, baroFailureCount, baroFailureWindowStartMs);
    }

    // Also runs when both sensors failed: every decision inside requires a reliable sensor,
    // but the landing timeout must still run so the flight log is saved.
    if (!boardHealth.critical) {
      updateFlightStateFromSamples();
    }
    updateGyroTest(imuSampleOk, now);

    if (missionStarted || flightState == FlightState::Idle) {
      appendMissionSample();
    }
    streamMissionLogIfNeeded();

    if (missionAtSafeGroundState() && !missionLogFlushed) {
      flushMissionLogToFlash();
    }

    if (missionAtSafeGroundState() && missionLogFlushed) {
      finalizeFlightPersonalRecords();
    }

    if (now - lastLogMs >= 1000) {
      lastLogMs = now;
      printRuntimeSummary();
    }
  }

  delay(1);
}

