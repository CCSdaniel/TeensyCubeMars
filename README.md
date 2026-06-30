# TeensyCubeMars

Arduino IDE project for controlling a CubeMars AK-series servo brushless motor
from a Teensy 4.1.

The PC connects to the Teensy over USB. The Teensy sends CubeMars servo-mode
position commands to the driver over CAN. In the Arduino IDE Serial Monitor you
can type movement commands such as:

```text
+90.0
-45.5
```

The leading sign is the direction and the floating-point number is the relative
movement angle in degrees.

## Recommended communication method: CAN

Use **CAN**, not UART, for the Teensy-to-CubeMars link.

Why:

- The AK-series manual describes the servo control protocol as an extended CAN
  protocol, with position, velocity, and position-velocity modes mapped directly
  to CAN packet IDs.
- The CAN interface on the driver is isolated and intended for robust motor
  control in noisy power-electronics environments.
- CAN at 1 Mbps gives fixed 8-byte frames and no application CRC framing in the
  Teensy sketch. The UART protocol is usable, but it requires packet framing,
  CRC-16, and more parsing/recovery code.
- Teensy 4.1 has built-in CAN controllers. You only need an external CAN
  transceiver because the Teensy pins are logic-level CAN TX/RX, not CANH/CANL.

UART is still useful for CubeMarsTool setup or debugging, but this project uses
CAN for runtime movement commands.

## Repository contents

```text
TeensyCubeMarsCLI/
  TeensyCubeMarsCLI.ino   Arduino IDE sketch for Teensy 4.1
```

## Hardware required

- Teensy 4.1
- CubeMars AK-series motor and matching AK-series driver board
- Motor power supply within the driver/motor rating
- 3.3 V CAN transceiver module capable of 1 Mbps
  - Examples: SN65HVD230, TCAN332, MCP2562FD configured with 3.3 V logic I/O
  - Avoid feeding 5 V logic into Teensy CAN RX
- CAN bus termination:
  - 120 ohm between CANH and CANL at each end of the bus
  - For one Teensy and one driver, that normally means one termination at the
    transceiver end and one at the driver end, if the driver does not already
    include switchable termination
- USB cable from PC to Teensy

## Wiring

### Teensy 4.1 to CAN transceiver

The sketch uses Teensy CAN1:

| Teensy 4.1 pin | Signal | Connect to |
| --- | --- | --- |
| 22 | CTX1 | CAN transceiver TXD |
| 23 | CRX1 | CAN transceiver RXD |
| 3.3 V | 3V3 | CAN transceiver VCC, if using a 3.3 V module |
| GND | GND | CAN transceiver GND |

### CAN transceiver to CubeMars driver

| CAN transceiver | CubeMars driver |
| --- | --- |
| CANH | CANH |
| CANL | CANL |
| GND | Driver signal/logic GND |

Power the motor driver from its rated DC supply. Do **not** power the motor from
the Teensy. Keep the motor power wiring sized for the expected current.

## CubeMars driver setup

Use CubeMarsTool/R-link or your existing setup process before running the sketch:

1. Load/configure servo-mode firmware for the motor/driver if it is not already
   in servo mode.
2. Calibrate the motor according to the CubeMars manual.
3. Confirm the CAN ID. The sketch defaults to motor ID `1`.
4. Confirm CAN bus speed is `1 Mbps` as specified by the manual.
5. Optionally enable periodic servo feedback upload if you want the `status`
   command to show live position/current/temperature.

If your motor ID is not `1`, edit this line in
`TeensyCubeMarsCLI/TeensyCubeMarsCLI.ino`:

```cpp
static constexpr uint8_t MOTOR_ID = 1;
```

## Arduino IDE configuration

1. Install Arduino IDE.
2. Install Teensyduino / Teensy board support for Arduino IDE.
3. Install the `FlexCAN_T4` library from Arduino Library Manager if it is not
   already available with your Teensy environment.
4. Open `TeensyCubeMarsCLI/TeensyCubeMarsCLI.ino`.
5. Select:
   - Board: `Teensy 4.1`
   - USB Type: `Serial`
   - CPU Speed: any normal Teensy 4.1 setting is fine
   - Port: the Teensy USB serial port
6. Click Upload.
7. Open Serial Monitor at `115200 baud`.
8. Set line ending to `Newline` or `Both NL & CR`.

## CLI commands

Primary movement format:

```text
+<degrees>
-<degrees>
```

Examples:

```text
+90.0
-45.5
+0.25
```

The sketch treats these as **relative moves from the current commanded target**.
For example, after `+90.0`, a subsequent `-45.5` commands the motor to `+44.5`
degrees from the current origin.

Additional setup/debug commands:

| Command | Meaning |
| --- | --- |
| `zero` | Set the current motor position as temporary origin and reset commanded target to `0` |
| `target <deg>` | Command an absolute target angle from the current origin |
| `speed <erpm>` | Set max electrical RPM used by position-velocity mode |
| `accel <erpm/s>` | Set acceleration limit used by position-velocity mode |
| `status` | Print last received servo feedback frame |
| `help` | Print command help |

Default motion limits in the sketch:

```cpp
static constexpr float DEFAULT_SPEED_ERPM = 5000.0f;
static constexpr float DEFAULT_ACCEL_ERPM_S = 30000.0f;
```

Start conservatively. Increase these values only after confirming the motor,
mechanism, supply, and safety limits.

## Activation sequence

1. Disconnect or mechanically unload the motor while first testing if possible.
2. Verify CANH/CANL polarity and termination.
3. Power the Teensy from USB.
4. Power the CubeMars driver from the motor supply.
5. Upload the sketch.
6. Open Serial Monitor at `115200 baud`.
7. Type:

   ```text
   zero
   ```

   This sets the current motor position as the temporary origin. Temporary origin
   is cleared when the driver loses power.

8. Send a small test move:

   ```text
   +5.0
   ```

9. If the direction is correct, continue with larger commands such as:

   ```text
   +90.0
   -45.5
   ```

10. Type `status` to inspect feedback if periodic CAN feedback is enabled on the
    driver.

## CAN packet used by the sketch

The sketch uses the CubeMars servo-mode **position-velocity loop** CAN command:

- Extended CAN ID: `(CAN_PACKET_SET_POS_SPD << 8) | MOTOR_ID`
- `CAN_PACKET_SET_POS_SPD = 6`
- DLC: 8 bytes
- Payload:

| Bytes | Value |
| --- | --- |
| 0..3 | target position, signed int32, degrees x 10000, big-endian |
| 4..5 | speed limit, signed int16, ERPM / 10, big-endian |
| 6..7 | acceleration limit, signed int16, ERPM/s / 10, big-endian |

The sketch also parses servo feedback frames with function ID `0x29` and the
configured motor ID:

- position = signed int16 x `0.1 deg`
- speed = signed int16 x `10 ERPM`
- current = signed int16 x `0.01 A`
- temperature = signed int8, degrees C
- error = CubeMars servo feedback error code

## Safety notes

- Keep hands and loose objects away from the motor during activation.
- Use current-limited bench power for early tests if practical.
- Confirm the driver voltage rating and motor phase wiring before applying
  power.
- Add an external emergency stop or power disconnect for real mechanisms.
- The `zero` command is temporary. Do not rely on it as a safety-rated homing
  procedure.
