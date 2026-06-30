/*
  TeensyCubeMarsCLI

  USB Serial CLI for a Teensy 4.1 controlling a CubeMars AK-series driver in
  servo-mode position-velocity control over UART.

  Primary CLI command:
    +90.0   -> move +90.0 degrees from the current commanded target
    -45.5   -> move -45.5 degrees from the current commanded target

  Hardware:
    PC USB Serial Monitor -> Teensy USB port
    Teensy Serial1 TX pin 1 -> CubeMars driver serial RX
    Teensy Serial1 RX pin 0 -> CubeMars driver serial TX
    Teensy GND -> CubeMars driver signal GND
*/

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// -------------------------- User configuration --------------------------

static constexpr uint32_t USB_SERIAL_BAUD = 115200;
static constexpr uint32_t MOTOR_SERIAL_BAUD = 115200;  // Match CubeMarsTool/driver setting.
static constexpr float DEFAULT_SPEED_ERPM = 5000.0f;
static constexpr float DEFAULT_ACCEL_ERPM_S = 30000.0f;
static constexpr float MAX_ABS_TARGET_DEG = 36000.0f;
static constexpr uint32_t STATUS_WAIT_MS = 150;

// Teensy 4.1 Serial1: RX1 = pin 0, TX1 = pin 1.
#define MOTOR_SERIAL Serial1

enum CommPacketId : uint8_t {
  COMM_FW_VERSION = 0,
  COMM_GET_VALUES = 4,
  COMM_SET_DUTY = 5,
  COMM_SET_CURRENT = 6,
  COMM_SET_CURRENT_BRAKE = 7,
  COMM_SET_RPM = 8,
  COMM_SET_POS = 9,
  COMM_ROTOR_POSITION = 22,
  COMM_SET_POS_SPD = 91,
  COMM_SET_POS_MULTI = 92,
  COMM_SET_POS_SINGLE = 93,
  COMM_SET_POS_ORIGIN = 95,
};

struct MotorMetrics {
  bool valid = false;
  float mosTemperatureC = 0.0f;
  float motorTemperatureC = 0.0f;
  float outputCurrentA = 0.0f;
  float inputCurrentA = 0.0f;
  float idCurrentA = 0.0f;
  float iqCurrentA = 0.0f;
  float duty = 0.0f;
  float speedErpm = 0.0f;
  float inputVoltageV = 0.0f;
  uint8_t statusCode = 0;
  float positionDeg = 0.0f;
  uint8_t motorId = 0;
  float vdVoltage = 0.0f;
  float vqVoltage = 0.0f;
  uint32_t lastUpdateMs = 0;
};

enum class UartRxState : uint8_t {
  WaitStart,
  ReadLength,
  ReadPayload,
  ReadCrcHigh,
  ReadCrcLow,
  ReadEnd,
};

static MotorMetrics metrics;
static float targetDeg = 0.0f;
static float speedErpm = DEFAULT_SPEED_ERPM;
static float accelErpmS = DEFAULT_ACCEL_ERPM_S;
static char cliBuffer[64];
static size_t cliLength = 0;

static UartRxState uartRxState = UartRxState::WaitStart;
static uint8_t uartRxBuffer[96];
static uint16_t uartRxLength = 0;
static uint16_t uartRxIndex = 0;
static uint16_t uartRxCrc = 0;

static void appendInt32(uint8_t *buffer, int32_t value, uint8_t &index) {
  buffer[index++] = static_cast<uint8_t>(value >> 24);
  buffer[index++] = static_cast<uint8_t>(value >> 16);
  buffer[index++] = static_cast<uint8_t>(value >> 8);
  buffer[index++] = static_cast<uint8_t>(value);
}

static int16_t readInt16(const uint8_t *buffer, uint16_t &index) {
  const int16_t value = static_cast<int16_t>((static_cast<uint16_t>(buffer[index]) << 8) |
                                            static_cast<uint16_t>(buffer[index + 1]));
  index += 2;
  return value;
}

static int32_t readInt32(const uint8_t *buffer, uint16_t &index) {
  const int32_t value = static_cast<int32_t>((static_cast<uint32_t>(buffer[index]) << 24) |
                                            (static_cast<uint32_t>(buffer[index + 1]) << 16) |
                                            (static_cast<uint32_t>(buffer[index + 2]) << 8) |
                                            static_cast<uint32_t>(buffer[index + 3]));
  index += 4;
  return value;
}

static uint16_t crc16Ccitt(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0;

  for (uint16_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

static void sendPacket(const uint8_t *payload, uint16_t length) {
  if (length == 0 || length > 255) {
    return;
  }

  const uint16_t crc = crc16Ccitt(payload, length);

  MOTOR_SERIAL.write(static_cast<uint8_t>(0x02));
  MOTOR_SERIAL.write(static_cast<uint8_t>(length));
  MOTOR_SERIAL.write(payload, length);
  MOTOR_SERIAL.write(static_cast<uint8_t>(crc >> 8));
  MOTOR_SERIAL.write(static_cast<uint8_t>(crc & 0xFF));
  MOTOR_SERIAL.write(static_cast<uint8_t>(0x03));
  MOTOR_SERIAL.flush();
}

static void requestValues() {
  const uint8_t payload[] = {COMM_GET_VALUES};
  sendPacket(payload, sizeof(payload));
}

static void setPositionVelocity(float positionDeg, float maxSpeedErpm, float maxAccelErpmS) {
  uint8_t payload[13];
  uint8_t index = 0;

  payload[index++] = COMM_SET_POS_SPD;
  appendInt32(payload, static_cast<int32_t>(lroundf(positionDeg * 1000.0f)), index);
  appendInt32(payload, static_cast<int32_t>(lroundf(fabsf(maxSpeedErpm))), index);
  appendInt32(payload, static_cast<int32_t>(lroundf(fabsf(maxAccelErpmS))), index);

  sendPacket(payload, index);
}

static void setCurrentPositionAsTemporaryOrigin() {
  const uint8_t payload[] = {COMM_SET_POS_ORIGIN, 0x01};
  sendPacket(payload, sizeof(payload));
  targetDeg = 0.0f;
}

static void setPositionLoopMode(uint8_t command) {
  const uint8_t payload[] = {command, 0x00, 0x00, 0x00, 0x00};
  sendPacket(payload, sizeof(payload));
}

static const char *faultText(uint8_t statusCode) {
  switch (statusCode) {
    case 0:
      return "none";
    case 1:
      return "over-voltage";
    case 2:
      return "under-voltage";
    case 3:
      return "driver fault";
    case 4:
      return "motor over-current";
    case 5:
      return "MOS over-temperature";
    case 6:
      return "motor over-temperature";
    case 11:
      return "encoder SPI fault";
    default:
      return "see CubeMars fault table";
  }
}

static void handleMotorPacket(const uint8_t *payload, uint16_t length) {
  if (length == 0) {
    return;
  }

  if (payload[0] == COMM_GET_VALUES && length >= 73) {
    uint16_t index = 1;

    metrics.mosTemperatureC = static_cast<float>(readInt16(payload, index)) / 10.0f;
    metrics.motorTemperatureC = static_cast<float>(readInt16(payload, index)) / 10.0f;
    metrics.outputCurrentA = static_cast<float>(readInt32(payload, index)) / 100.0f;
    metrics.inputCurrentA = static_cast<float>(readInt32(payload, index)) / 100.0f;
    metrics.idCurrentA = static_cast<float>(readInt32(payload, index)) / 100.0f;
    metrics.iqCurrentA = static_cast<float>(readInt32(payload, index)) / 100.0f;
    metrics.duty = static_cast<float>(readInt16(payload, index)) / 1000.0f;
    metrics.speedErpm = static_cast<float>(readInt32(payload, index));
    metrics.inputVoltageV = static_cast<float>(readInt16(payload, index)) / 10.0f;

    index += 24;  // Reserved bytes in the CubeMars full-values response.
    metrics.statusCode = payload[index++];
    metrics.positionDeg = static_cast<float>(readInt32(payload, index)) / 1000.0f;
    metrics.motorId = payload[index++];

    index += 6;  // Temperature reserved values.
    metrics.vdVoltage = static_cast<float>(readInt32(payload, index)) / 1000.0f;
    metrics.vqVoltage = static_cast<float>(readInt32(payload, index)) / 1000.0f;
    metrics.valid = true;
    metrics.lastUpdateMs = millis();
    return;
  }

  if (payload[0] == COMM_ROTOR_POSITION && length >= 5) {
    uint16_t index = 1;
    metrics.positionDeg = static_cast<float>(readInt32(payload, index)) / 10000.0f;
    metrics.valid = true;
    metrics.lastUpdateMs = millis();
  }
}

static void resetUartParser() {
  uartRxState = UartRxState::WaitStart;
  uartRxLength = 0;
  uartRxIndex = 0;
  uartRxCrc = 0;
}

static void processMotorByte(uint8_t byteValue) {
  switch (uartRxState) {
    case UartRxState::WaitStart:
      if (byteValue == 0x02) {
        uartRxState = UartRxState::ReadLength;
      }
      break;

    case UartRxState::ReadLength:
      uartRxLength = byteValue;
      uartRxIndex = 0;
      if (uartRxLength == 0 || uartRxLength > sizeof(uartRxBuffer)) {
        resetUartParser();
      } else {
        uartRxState = UartRxState::ReadPayload;
      }
      break;

    case UartRxState::ReadPayload:
      uartRxBuffer[uartRxIndex++] = byteValue;
      if (uartRxIndex >= uartRxLength) {
        uartRxState = UartRxState::ReadCrcHigh;
      }
      break;

    case UartRxState::ReadCrcHigh:
      uartRxCrc = static_cast<uint16_t>(byteValue) << 8;
      uartRxState = UartRxState::ReadCrcLow;
      break;

    case UartRxState::ReadCrcLow:
      uartRxCrc |= byteValue;
      uartRxState = UartRxState::ReadEnd;
      break;

    case UartRxState::ReadEnd:
      if (byteValue == 0x03 && crc16Ccitt(uartRxBuffer, uartRxLength) == uartRxCrc) {
        handleMotorPacket(uartRxBuffer, uartRxLength);
      }
      resetUartParser();
      break;
  }
}

static void pollMotorUart() {
  while (MOTOR_SERIAL.available() > 0) {
    processMotorByte(static_cast<uint8_t>(MOTOR_SERIAL.read()));
  }
}

static void requestStatusAndWait() {
  const uint32_t previousUpdateMs = metrics.lastUpdateMs;
  const uint32_t requestStartMs = millis();

  requestValues();
  while (millis() - requestStartMs < STATUS_WAIT_MS) {
    pollMotorUart();
    if (metrics.valid && metrics.lastUpdateMs != previousUpdateMs) {
      return;
    }
  }
}

static void printHelp() {
  Serial.println();
  Serial.println(F("CubeMars AK servo CLI over UART"));
  Serial.println(F("Commands:"));
  Serial.println(F("  +<deg>       move positive relative angle, e.g. +90.0"));
  Serial.println(F("  -<deg>       move negative relative angle, e.g. -45.5"));
  Serial.println(F("  multi        set driver position ring to multi-turn mode"));
  Serial.println(F("  single       set driver position ring to single-turn mode"));
  Serial.println(F("  zero         set current motor position as temporary origin"));
  Serial.println(F("  target <deg> set absolute target angle from origin"));
  Serial.println(F("  speed <erpm> set position-velocity max speed"));
  Serial.println(F("  accel <e/s2> set position-velocity acceleration"));
  Serial.println(F("  status       request and print voltage/current/position metrics"));
  Serial.println(F("  help         show this help"));
  Serial.println();
}

static void printStatus() {
  requestStatusAndWait();

  Serial.print(F("commanded_target="));
  Serial.print(targetDeg, 3);
  Serial.print(F(" deg, speed_limit="));
  Serial.print(speedErpm, 1);
  Serial.print(F(" ERPM, accel_limit="));
  Serial.print(accelErpmS, 1);
  Serial.println(F(" ERPM/s"));

  if (!metrics.valid) {
    Serial.println(F("metrics: no valid UART response yet"));
    Serial.println(F("Check driver power, TX/RX crossing, GND, baud rate, and serial-mode firmware."));
    return;
  }

  Serial.print(F("metrics: position="));
  Serial.print(metrics.positionDeg, 3);
  Serial.print(F(" deg, speed="));
  Serial.print(metrics.speedErpm, 1);
  Serial.print(F(" ERPM, input_voltage="));
  Serial.print(metrics.inputVoltageV, 2);
  Serial.println(F(" V"));

  Serial.print(F("currents: output="));
  Serial.print(metrics.outputCurrentA, 2);
  Serial.print(F(" A, input="));
  Serial.print(metrics.inputCurrentA, 2);
  Serial.print(F(" A, id="));
  Serial.print(metrics.idCurrentA, 2);
  Serial.print(F(" A, iq="));
  Serial.print(metrics.iqCurrentA, 2);
  Serial.println(F(" A"));

  Serial.print(F("temps: mos="));
  Serial.print(metrics.mosTemperatureC, 1);
  Serial.print(F(" C, motor="));
  Serial.print(metrics.motorTemperatureC, 1);
  Serial.print(F(" C, status="));
  Serial.print(static_cast<unsigned int>(metrics.statusCode));
  Serial.print(F(" ("));
  Serial.print(faultText(metrics.statusCode));
  Serial.print(F("), motor_id="));
  Serial.print(static_cast<unsigned int>(metrics.motorId));
  Serial.print(F(", age_ms="));
  Serial.println(millis() - metrics.lastUpdateMs);
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

  if (strcmp(line, "multi") == 0) {
    setPositionLoopMode(COMM_SET_POS_MULTI);
    Serial.println(F("OK requested multi-turn position mode"));
    return;
  }

  if (strcmp(line, "single") == 0) {
    setPositionLoopMode(COMM_SET_POS_SINGLE);
    Serial.println(F("OK requested single-turn position mode"));
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

void setup() {
  Serial.begin(USB_SERIAL_BAUD);

  const uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2500) {
  }

  MOTOR_SERIAL.begin(MOTOR_SERIAL_BAUD);

  printHelp();
  Serial.println(F("Ready. Type zero after powering/enabling the driver, then send +<deg> or -<deg>."));
}

void loop() {
  pollSerialCli();
  pollMotorUart();
}
