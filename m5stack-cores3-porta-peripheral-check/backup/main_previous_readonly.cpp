#include <Arduino.h>
#include <M5Unified.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>

namespace {

constexpr int kPortASdaPin = 2;
constexpr int kPortASclPin = 1;
constexpr uint32_t kI2cClockHz = 400000UL;
constexpr uint32_t kVescBaud = 115200UL;
constexpr float kInaShuntOhm = 0.002f;
constexpr uint8_t kPcaAddress = 0x40;
constexpr uint8_t kTofAddress = 0x29;

enum class Mode : uint8_t { Servo, Distance, Power, Motor };

Mode mode = Mode::Servo;
HardwareSerial vescUart(1);
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData{};
bool tofRunning = false;
bool devicePresent = false;
bool vescValid = false;
bool inaValid = false;
uint8_t inaAddress = 0;
uint8_t pcaMode1 = 0;
uint8_t pcaPrescale = 0;
uint16_t tofCenterMm = 0;
uint32_t tofFrames = 0;
float inaBusV = NAN;
float inaShuntMv = NAN;
float inaCurrentA = NAN;
float vescInputV = NAN;
float vescInputA = NAN;
float vescDuty = NAN;
float vescErpm = NAN;
String status = "Tap a role, then connect one device.";
uint32_t lastPollMs = 0;
uint32_t lastDrawMs = 0;
uint32_t lastVescRequestMs = 0;

bool ack(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool read8(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(address), 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool read16(uint8_t address, uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(address), 2) != 2) return false;
  value = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  return true;
}

void startI2c() {
  vescUart.end();
  Wire.end();
  Wire.begin(kPortASdaPin, kPortASclPin, kI2cClockHz);
  Wire.setTimeOut(25);
}

void stopTof() {
  if (tofRunning) tof.stopRanging();
  tofRunning = false;
}

uint16_t vescCrc16(const uint8_t* data, size_t size) {
  uint16_t crc = 0;
  while (size--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

int16_t readI16(const uint8_t* value) { return static_cast<int16_t>((value[0] << 8) | value[1]); }
int32_t readI32(const uint8_t* value) {
  return static_cast<int32_t>((static_cast<uint32_t>(value[0]) << 24) | (static_cast<uint32_t>(value[1]) << 16) |
                              (static_cast<uint32_t>(value[2]) << 8) | value[3]);
}

void enterMode(Mode next) {
  if (next == mode) return;
  stopTof();
  mode = next;
  devicePresent = false;
  inaValid = false;
  vescValid = false;
  tofCenterMm = 0;
  tofFrames = 0;

  if (mode == Mode::Motor) {
    Wire.end();
    // Port A yellow=GPIO2 is RX and white=GPIO1 is TX in MOTOR mode.
    vescUart.begin(kVescBaud, SERIAL_8N1, kPortASdaPin, kPortASclPin);
    status = "UART active: G2=RX, G1=TX. Read-only VESC query.";
  } else {
    startI2c();
    status = "I2C active: G2=SDA, G1=SCL. Reading only.";
    if (mode == Mode::Distance) {
      devicePresent = ack(kTofAddress);
      if (devicePresent && tof.begin(kTofAddress, Wire) && tof.setResolution(64) && tof.setRangingFrequency(10) &&
          tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS) && tof.startRanging()) {
        tofRunning = true;
        status = "VL53L5CX ranging: 8x8, 10 Hz.";
      } else {
        status = "VL53L5CX not ready at 0x29.";
      }
    }
  }
}

void pollServo() {
  devicePresent = ack(kPcaAddress);
  if (!devicePresent) {
    status = "PCA9685 not found at 0x40.";
    return;
  }
  if (read8(kPcaAddress, 0x00, pcaMode1) && read8(kPcaAddress, 0xFE, pcaPrescale)) {
    status = "PCA9685 detected. No PWM registers are written.";
  } else {
    status = "PCA9685 ACKed but register read failed.";
  }
}

void pollDistance() {
  if (!tofRunning || !tof.isDataReady()) return;
  if (!tof.getRangingData(&tofData)) {
    status = "VL53L5CX data read failed.";
    return;
  }
  tofCenterMm = tofData.distance_mm[32];
  ++tofFrames;
  status = "Distance frames are read from the center zone.";
}

void pollPower() {
  constexpr uint8_t candidates[] = {0x40, 0x41, 0x44};
  inaValid = false;
  for (const uint8_t address : candidates) {
    uint16_t maker = 0;
    uint16_t die = 0;
    if (read16(address, 0xFE, maker) && read16(address, 0xFF, die) && maker == 0x5449 && die == 0x2260) {
      inaAddress = address;
      uint16_t bus = 0;
      uint16_t shunt = 0;
      if (read16(address, 0x02, bus) && read16(address, 0x01, shunt)) {
        inaBusV = bus * 1.25e-3f;
        inaShuntMv = static_cast<int16_t>(shunt) * 2.5e-3f;
        inaCurrentA = (inaShuntMv * 1e-3f) / kInaShuntOhm;
        inaValid = true;
        status = "INA226 detected. Current uses configured 2 mOhm shunt.";
      }
      return;
    }
  }
  status = "INA226 not found (checked 0x40, 0x41, 0x44).";
}

void sendVescRequest() {
  const uint8_t payload[] = {4};  // COMM_GET_VALUES
  const uint16_t crc = vescCrc16(payload, sizeof(payload));
  const uint8_t frame[] = {2, 1, 4, static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc), 3};
  vescUart.write(frame, sizeof(frame));
}

void parseVesc() {
  static uint8_t state = 0;
  static uint8_t expected = 0;
  static uint8_t index = 0;
  static uint8_t payload[80]{};
  static uint16_t receivedCrc = 0;
  while (vescUart.available()) {
    const uint8_t byte = static_cast<uint8_t>(vescUart.read());
    if (state == 0) {
      if (byte == 2) state = 1;
    } else if (state == 1) {
      expected = byte;
      index = 0;
      state = (expected > 0 && expected <= sizeof(payload)) ? 2 : 0;
    } else if (state == 2) {
      payload[index++] = byte;
      if (index == expected) state = 3;
    } else if (state == 3) {
      receivedCrc = static_cast<uint16_t>(byte) << 8;
      state = 4;
    } else if (state == 4) {
      receivedCrc |= byte;
      state = 5;
    } else {
      if (byte == 3 && receivedCrc == vescCrc16(payload, expected) && expected >= 29 && payload[0] == 4) {
        const uint8_t* values = payload + 1;
        vescInputA = readI32(values + 8) / 100.0f;
        vescDuty = readI16(values + 20) / 1000.0f;
        vescErpm = static_cast<float>(readI32(values + 22));
        vescInputV = readI16(values + 26) / 10.0f;
        vescValid = true;
        status = "VESC telemetry received. No motor command is sent.";
      }
      state = 0;
    }
  }
}

const char* roleName() {
  switch (mode) {
    case Mode::Servo: return "SERVO / PCA9685";
    case Mode::Distance: return "DISTANCE / VL53L5CX";
    case Mode::Power: return "POWER / INA226";
    case Mode::Motor: return "MOTOR / VESC";
  }
  return "UNKNOWN";
}

void drawButton(int x, const char* label, bool selected) {
  const uint16_t bg = selected ? 0x07E0 : 0x39E7;
  M5.Display.fillRoundRect(x, 202, 76, 34, 5, bg);
  M5.Display.setTextColor(selected ? 0x0000 : 0xFFFF, bg);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(x + 5, 215);
  M5.Display.print(label);
}

void drawScreen() {
  M5.Display.fillScreen(0x0000);
  M5.Display.setTextColor(0xFFFF, 0x0000);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(8, 8);
  M5.Display.print("PORT A CHECKER");
  M5.Display.setTextColor(0x07FF, 0x0000);
  M5.Display.setCursor(8, 34);
  M5.Display.print(roleName());
  M5.Display.setTextColor(0xFFFF, 0x0000);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 64);

  if (mode == Mode::Servo) {
    M5.Display.printf("Port A I2C  G2=SDA  G1=SCL\n\nAddress: 0x40  %s\nMODE1: 0x%02X  PRE_SCALE: %u\n\nRead-only. PWM / servo output is disabled.",
                      devicePresent ? "FOUND" : "NOT FOUND", pcaMode1, pcaPrescale);
  } else if (mode == Mode::Distance) {
    M5.Display.printf("Port A I2C  G2=SDA  G1=SCL\n\nAddress: 0x29  %s\nCenter distance: %u mm\nFrames: %lu\n\n8x8 continuous ranging at 10 Hz.",
                      tofRunning ? "RANGING" : "NOT READY", tofCenterMm, static_cast<unsigned long>(tofFrames));
  } else if (mode == Mode::Power) {
    M5.Display.printf("Port A I2C  G2=SDA  G1=SCL\n\nINA226: %s  address: 0x%02X\nBus: %.3f V\nShunt: %.3f mV\nCurrent: %.3f A (2 mOhm setting)",
                      inaValid ? "FOUND" : "NOT FOUND", inaAddress, inaBusV, inaShuntMv, inaCurrentA);
  } else {
    M5.Display.printf("Port A UART  G2=RX  G1=TX\n\nVESC: %s  115200 bps 8N1\nInput: %.1f V  %.2f A\nDuty: %.3f\nSpeed: %.0f ERPM\n\nRead-only COMM_GET_VALUES.",
                      vescValid ? "TELEMETRY OK" : "WAITING", vescInputV, vescInputA, vescDuty, vescErpm);
  }
  M5.Display.setTextColor(0xFFE0, 0x0000);
  M5.Display.setCursor(8, 178);
  M5.Display.print(status);
  drawButton(2, "SERVO", mode == Mode::Servo);
  drawButton(82, "DISTANCE", mode == Mode::Distance);
  drawButton(162, "POWER", mode == Mode::Power);
  drawButton(242, "MOTOR", mode == Mode::Motor);
}

void handleTouch() {
  const auto touch = M5.Touch.getDetail();
  if (!touch.wasClicked() || touch.y < 198) return;
  if (touch.x < 80) enterMode(Mode::Servo);
  else if (touch.x < 160) enterMode(Mode::Distance);
  else if (touch.x < 240) enterMode(Mode::Power);
  else enterMode(Mode::Motor);
}

}  // namespace

void setup() {
  auto config = M5.config();
  config.serial_baudrate = 115200;
  M5.begin(config);
  Serial.begin(115200);
  startI2c();
  status = "Ready. Select the connected Port A role.";
  Serial.println("CoreS3 Port A peripheral checker: read-only mode.");
}

void loop() {
  M5.update();
  handleTouch();
  const uint32_t now = millis();
  if (mode == Mode::Motor) {
    if (now - lastVescRequestMs >= 500) {
      sendVescRequest();
      lastVescRequestMs = now;
    }
    parseVesc();
  } else if (now - lastPollMs >= 250) {
    if (mode == Mode::Servo) pollServo();
    else if (mode == Mode::Distance) pollDistance();
    else pollPower();
    lastPollMs = now;
  }
  if (now - lastDrawMs >= 150) {
    drawScreen();
    lastDrawMs = now;
  }
  delay(5);
}
