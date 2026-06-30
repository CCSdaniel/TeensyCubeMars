/*
  TeensyCubeMarsCLI

  USB Serial CLI for a Teensy 4.1 controlling a CubeMars AK-series driver in
  servo-mode position-velocity control over CAN.

  Primary CLI command:
    +90.0   -> move +90.0 degrees from the current commanded target
    -45.5   -> move -45.5 degrees from the current commanded target

  Hardware:
    Teensy 4.1 CAN1 TX pin 22 -> 3.3 V CAN transceiver TXD
    Teensy 4.1 CAN1 RX pin 23 -> 3.3 V CAN transceiver RXD
    CAN transceiver CANH/CANL -> CubeMars driver CANH/CANL
    Common GND between Teensy/transceiver and driver logic ground

  Required Arduino library:
    FlexCAN_T4
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// -------------------------- User configuration --------------------------

static constexpr uint8_t MOTOR_ID = 1;                 // CubeMars driver CAN ID
static constexpr uint32_t CAN_BAUD = 1000000;          // AK-series servo CAN rate
static constexpr float DEFAULT_SPEED_ERPM = 5000.0f;   // Max electrical RPM
static constexpr float DEFAULT_ACCEL_ERPM_S = 30000.0f; // Max electrical RPM/s
static constexpr float MAX_ABS_TARGET_DEG = 36000.0f;  // Manual position range
static constexpr uint32_t SERIAL_BAUD = 115200;

// Teensy 4.1 CAN1: CTX1 = pin 22, CRX1 = pin 23.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;

enum CanPacketId : uint32_t {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT = 1,
  CAN_PACKET_SET_CURRENT_BRAKE = 2,
  CAN_PACKET_SET_RPM = 3,
  CAN_PACKET_SET_POS = 4,
  CAN_PACKET_SET_ORIGIN_HERE = 5,
  CAN_PACKET_SET_POS_SPD = 6,
};

struct ServoFeedback {
  bool valid = false;
  float positionDeg = 0.0f;
  float speedErpm = 0.0f;
  float currentA = 0.0f;
  int8_t temperatureC = 0;
  uint8_t errorCode = 0;
  uint32_t lastUpdateMs = 0;
};

static ServoFeedback feedback;
static float targetDeg = 0.0f;
static float speedErpm = DEFAULT_SPEED_ERPM;
static float accelErpmS = DEFAULT_ACCEL_ERPM_S;
static char cliBuffer[64];
static size_t cliLength = 0;

static void appendInt32(uint8_t *buffer, int32_t value, uint8_t &index) {
  buffer[index++] = static_cast<uint8_t>(value >> 24);
  buffer[index++] = static_cast<uint8_t>(value >> 16);
  buffer[index++] = static_cast<uint8_t>(value >> 8);
  buffer[index++] = static_cast<uint8_t>(value);
}

static void appendInt16(uint8_t *buffer, int16_t value, uint8_t &index) {
  buffer[index++] = static_cast<uint8_t>(value >> 8);
  buffer[index++] = static_cast<uint8_t>(value);
}

static int16_t toServoInt16Units(float value, float scale, int16_t minValue, int16_t maxValue) {
  long scaled = lroundf(value / scale);
  if (scaled < minValue) {
    scaled = minValue;
  } else if (scaled > maxValue) {
    scaled = maxValue;
  }
  return static_cast<int16_t>(scaled);
}

static void transmitExtended(uint32_t extendedId, const uint8_t *data, uint8_t length) {
  CAN_message_t message = {};
  message.id = extendedId;
  message.len = length;
  message.flags.extended = 1;
  message.flags.remote = 0;

  for (uint8_t i = 0; i < length && i < 8; ++i) {
    message.buf[i] = data[i];
  }

  CanBus.write(message);
}

static void setPositionVelocity(float positionDeg, float maxSpeedErpm, float maxAccelErpmS) {
  uint8_t payload[8];
  uint8_t index = 0;

  const int32_t positionRaw = static_cast<int32_t>(lroundf(positionDeg * 10000.0f));
  const int16_t speedRaw = toServoInt16Units(fabsf(maxSpeedErpm), 10.0f, 0, 32767);
  const int16_t accelRaw = toServoInt16Units(fabsf(maxAccelErpmS), 10.0f, 0, 32767);

  appendInt32(payload, positionRaw, index);
  appendInt16(payload, speedRaw, index);
  appendInt16(payload, accelRaw, index);

  transmitExtended(static_cast<uint32_t>(MOTOR_ID) | (CAN_PACKET_SET_POS_SPD << 8), payload, index);
}

static void setCurrentPositionAsTemporaryOrigin() {
  const uint8_t temporaryOrigin = 0;
  transmitExtended(static_cast<uint32_t>(MOTOR_ID) | (CAN_PACKET_SET_ORIGIN_HERE << 8),
                   &temporaryOrigin,
                   1);
  targetDeg = 0.0f;
}

static const char *faultText(uint8_t errorCode) {
  switch (errorCode) {
    case 0:
      return "none";
    case 1:
      return "motor over-temperature";
    case 2:
      return "over-current";
    case 3:
      return "over-voltage";
    case 4:
      return "under-voltage";
    case 5:
      return "encoder fault";
    case 6:
      return "MOSFET over-temperature";
    case 7:
      return "motor stall";
    default:
      return "unknown";
  }
}

static void printHelp() {
  Serial.println();
  Serial.println(F("CubeMars AK servo CLI over CAN"));
  Serial.println(F("Commands:"));
  Serial.println(F("  +<deg>       move positive relative angle, e.g. +90.0"));
  Serial.println(F("  -<deg>       move negative relative angle, e.g. -45.5"));
  Serial.println(F("  zero         set current motor position as temporary origin"));
  Serial.println(F("  target <deg> set absolute target angle from origin"));
  Serial.println(F("  speed <erpm> set position-velocity max speed"));
  Serial.println(F("  accel <e/s2> set position-velocity acceleration"));
  Serial.println(F("  status       print last CAN feedback frame"));
  Serial.println(F("  help         show this help"));
  Serial.println();
}

static void printStatus() {
  Serial.print(F("target="));
  Serial.print(targetDeg, 3);
  Serial.print(F(" deg, speed_limit="));
  Serial.print(speedErpm, 1);
  Serial.print(F(" ERPM, accel_limit="));
  Serial.print(accelErpmS, 1);
  Serial.println(F(" ERPM/s"));

  if (!feedback.valid) {
    Serial.println(F("feedback: no servo feedback frame received yet"));
    return;
  }

  Serial.print(F("feedback: pos="));
  Serial.print(feedback.positionDeg, 2);
  Serial.print(F(" deg, speed="));
  Serial.print(feedback.speedErpm, 1);
  Serial.print(F(" ERPM, current="));
  Serial.print(feedback.currentA, 2);
  Serial.print(F(" A, temp="));
  Serial.print(static_cast<int>(feedback.temperatureC));
  Serial.print(F(" C, error="));
  Serial.print(static_cast<unsigned int>(feedback.errorCode));
  Serial.print(F(" ("));
  Serial.print(faultText(feedback.errorCode));
  Serial.print(F("), age_ms="));
  Serial.println(millis() - feedback.lastUpdateMs);
}

static bool parseFloatAfterPrefix(const char *line, const char *prefix, float &value) {
  const size_t prefixLength = strlen(prefix);
  if (strncmp(line, prefix, prefixLength) != 0) {
    return false;
  }

  char *endPointer = nullptr;
  value = strtof(line + prefixLength, &endPointer);
  return endPointer != line + prefixLength && *endPointer == '\0';
}

static void executeMove(float deltaDeg) {
  if (!isfinite(deltaDeg)) {
    Serial.println(F("ERR movement must be a finite number"));
    return;
  }

  const float requestedTarget = targetDeg + deltaDeg;
  if (fabsf(requestedTarget) > MAX_ABS_TARGET_DEG) {
    Serial.println(F("ERR target exceeds +/-36000 deg servo-mode range"));
    return;
  }

  targetDeg = requestedTarget;
  setPositionVelocity(targetDeg, speedErpm, accelErpmS);

  Serial.print(F("OK moving to target "));
  Serial.print(targetDeg, 3);
  Serial.println(F(" deg"));
}

static void executeAbsoluteTarget(float requestedTargetDeg) {
  if (!isfinite(requestedTargetDeg)) {
    Serial.println(F("ERR target must be a finite number"));
    return;
  }

  if (fabsf(requestedTargetDeg) > MAX_ABS_TARGET_DEG) {
    Serial.println(F("ERR target exceeds +/-36000 deg servo-mode range"));
    return;
  }

  targetDeg = requestedTargetDeg;
  setPositionVelocity(targetDeg, speedErpm, accelErpmS);

  Serial.print(F("OK target set to "));
  Serial.print(targetDeg, 3);
  Serial.println(F(" deg"));
}

static void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  size_t length = strlen(line);
  while (length > 0 && (line[length - 1] == ' ' || line[length - 1] == '\t')) {
    line[--length] = '\0';
  }

  if (line[0] == '\0') {
    return;
  }

  if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(line, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(line, "zero") == 0) {
    setCurrentPositionAsTemporaryOrigin();
    Serial.println(F("OK temporary origin set; target reset to 0 deg"));
    return;
  }

  float value = 0.0f;
  if (parseFloatAfterPrefix(line, "target ", value)) {
    executeAbsoluteTarget(value);
    return;
  }

  if (parseFloatAfterPrefix(line, "speed ", value)) {
    if (!isfinite(value) || value <= 0.0f || value > 327670.0f) {
      Serial.println(F("ERR speed must be >0 and <=327670 ERPM"));
      return;
    }
    speedErpm = value;
    Serial.println(F("OK speed updated"));
    return;
  }

  if (parseFloatAfterPrefix(line, "accel ", value)) {
    if (!isfinite(value) || value <= 0.0f || value > 327670.0f) {
      Serial.println(F("ERR accel must be >0 and <=327670 ERPM/s"));
      return;
    }
    accelErpmS = value;
    Serial.println(F("OK accel updated"));
    return;
  }

  if (line[0] == '+' || line[0] == '-') {
    char *endPointer = nullptr;
    const float delta = strtof(line, &endPointer);
    if (endPointer != line && *endPointer == '\0') {
      executeMove(delta);
      return;
    }
  }

  Serial.println(F("ERR unknown command; type help"));
}

static void pollSerialCli() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      if (cliLength > 0) {
        cliBuffer[cliLength] = '\0';
        handleCommand(cliBuffer);
        cliLength = 0;
      }
      continue;
    }

    if (cliLength < sizeof(cliBuffer) - 1) {
      cliBuffer[cliLength++] = c;
    } else {
      cliLength = 0;
      Serial.println(F("ERR command too long"));
    }
  }
}

static void pollCanFeedback() {
  CAN_message_t message = {};
  while (CanBus.read(message)) {
    if (!message.flags.extended || message.len < 8) {
      continue;
    }

    const uint32_t functionId = message.id >> 8;
    const uint8_t sourceId = static_cast<uint8_t>(message.id & 0xFF);
    if (sourceId != MOTOR_ID || functionId != 0x29) {
      continue;
    }

    const int16_t posRaw = static_cast<int16_t>((message.buf[0] << 8) | message.buf[1]);
    const int16_t speedRaw = static_cast<int16_t>((message.buf[2] << 8) | message.buf[3]);
    const int16_t currentRaw = static_cast<int16_t>((message.buf[4] << 8) | message.buf[5]);

    feedback.valid = true;
    feedback.positionDeg = static_cast<float>(posRaw) * 0.1f;
    feedback.speedErpm = static_cast<float>(speedRaw) * 10.0f;
    feedback.currentA = static_cast<float>(currentRaw) * 0.01f;
    feedback.temperatureC = static_cast<int8_t>(message.buf[6]);
    feedback.errorCode = message.buf[7];
    feedback.lastUpdateMs = millis();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Give the Arduino Serial Monitor a short window to attach without blocking
  // standalone operation.
  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2500) {
  }

  CanBus.begin();
  CanBus.setBaudRate(CAN_BAUD);

  printHelp();
  Serial.println(F("Ready. Type zero after powering/enabling the driver, then send +<deg> or -<deg>."));
}

void loop() {
  pollSerialCli();
  pollCanFeedback();
}
