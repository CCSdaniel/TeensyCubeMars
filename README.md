# TeensyCubeMars

Arduino IDE project for controlling a CubeMars AK-series servo brushless motor
from a Teensy 4.1 using **UART/serial**.

The PC connects to the Teensy over USB. The Teensy connects to the CubeMars
driver serial port using a hardware UART. In the Arduino IDE Serial Monitor you
can type movement commands such as:

```text
+90.0
-45.5
```

The leading sign is the direction and the floating-point number is the relative
movement angle in degrees.

## Communication method: UART

This project uses the CubeMars servo serial protocol:

```text
0x02 + length + command/data payload + CRC16 + 0x03
```

This UART version only needs TX, RX, and GND between the Teensy and the
CubeMars driver serial port.

## Repository contents

```text
TeensyCubeMarsCLI/
  TeensyCubeMarsCLI.ino   Arduino IDE sketch for Teensy 4.1
```

## Hardware required

- Teensy 4.1
- CubeMars AK-series motor and matching AK-series driver board
- Motor power supply within the driver/motor rating
- USB cable from PC to Teensy
- Three signal wires for UART:
  - Teensy TX to driver RX
  - Teensy RX to driver TX
  - Ground to ground

## Wiring

The sketch uses Teensy `Serial1`. The CubeMars manual lists the driver serial
connector as: pin 1 = GND, pin 2 = driver TX, pin 3 = driver RX.

| Teensy 4.1 board pin | Teensy signal | CubeMars serial connector |
| --- | --- | --- |
| Pin 1 | TX1 | Pin 3, serial RX |
| Pin 0 | RX1 | Pin 2, serial TX |
| GND | GND | Pin 1, serial GND |

Important notes:

- The default sketch uses **pin 1 for TX1** and **pin 0 for RX1**.
- Cross TX and RX: Teensy TX goes to driver RX, and Teensy RX goes to driver TX.
- Confirm the CubeMars driver serial voltage level before wiring. Teensy 4.1 pins
  are **not 5 V tolerant**. If the driver TX is 5 V logic, use a level shifter
  into Teensy RX.
- Do not power the motor from the Teensy. The motor driver needs its rated DC
  supply.
- Keep UART wires short and separated from phase wires and high-current wiring.

## CubeMars driver setup

Use CubeMarsTool/R-link or your existing setup process before running the sketch:

1. Load/configure servo-mode firmware for the motor/driver if it is not already
   in servo mode.
2. Calibrate the motor according to the CubeMars manual.
3. Confirm the driver serial/UART baud rate.
4. Set the sketch baud rate to match the driver.

The sketch defaults to `115200` baud for the CubeMars UART:

```cpp
static constexpr uint32_t MOTOR_SERIAL_BAUD = 115200;
```

If CubeMarsTool shows a different serial baud rate for your driver, update that
constant and upload again.

## Arduino IDE configuration

1. Install Arduino IDE.
2. Install Teensyduino / Teensy board support for Arduino IDE.
3. Open `TeensyCubeMarsCLI/TeensyCubeMarsCLI.ino`.
4. Select:
   - Board: `Teensy 4.1`
   - USB Type: `Serial`
   - CPU Speed: any normal Teensy 4.1 setting is fine
   - Port: the Teensy USB serial port
5. Click Upload.
6. Open Serial Monitor at `115200 baud`.
7. Set line ending to `Newline` or `Both NL & CR`.

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
| `multi` | Set the driver position ring to multi-turn mode (`+/-100` turns) |
| `single` | Set the driver position ring to single-turn mode (`0..360 deg`) |
| `zero` | Set the current motor position as temporary origin and reset commanded target to `0` |
| `target <deg>` | Command an absolute target angle from the current origin |
| `speed <erpm>` | Set max electrical RPM used by position-velocity mode |
| `accel <erpm/s>` | Set acceleration limit used by position-velocity mode |
| `status` | Request and print driver metrics |
| `rawstatus` | Dump raw UART bytes from a status request and parser counters |
| `help` | Print command help |

Default motion limits in the sketch:

```cpp
static constexpr float DEFAULT_SPEED_ERPM = 5000.0f;
static constexpr float DEFAULT_ACCEL_ERPM_S = 30000.0f;
```

Start conservatively. Increase these values only after confirming the motor,
mechanism, supply, and safety limits.

## Metrics available with `status`

The `status` command sends `COMM_GET_VALUES` over UART and prints the latest
valid response.

Metrics printed by the sketch:

- Teensy-side commanded target angle
- Position reported by the driver
- Speed in ERPM
- Input voltage
- Output current
- Input current
- `Id` and `Iq` currents
- MOS temperature
- Motor temperature
- Driver status/fault code
- Motor ID reported in the values packet

Example:

```text
status
```

If no response is received, check:

- Driver power
- TX/RX are crossed correctly
- Common ground is connected
- `MOTOR_SERIAL_BAUD` matches the driver setting
- The driver firmware supports the servo serial protocol

For lower-level troubleshooting, type:

```text
rawstatus
```

The sketch sends the manual's `COMM_GET_VALUES` request:

```text
02 01 04 40 84 03
```

Then it dumps raw RX bytes and parser counters. A normal full status response
starts with:

```text
02 49 04 ...
```

Interpretation:

- `RX bytes captured=0`: the driver is not replying. Re-check driver power,
  CubeMars connector pinout, crossed TX/RX, common ground, baud rate, and whether
  the driver firmware supports serial servo mode.
- Random-looking bytes with many `crc_errors` or `bad_tail_errors`: baud rate,
  voltage level, noise, or grounding is probably wrong.
- Bytes arrive but do not start with `02 49 04`: the driver may be sending a
  different serial message, such as a position-only frame, or the wrong command
  is being triggered.
- Valid packets increase but `status` still does not show voltage/current:
  capture the raw bytes; the response layout may differ for that firmware and
  the parser offsets need adjustment.

## Activation sequence

1. Disconnect or mechanically unload the motor while first testing if possible.
2. Verify TX/RX/GND wiring and voltage levels.
3. Power the Teensy from USB.
4. Power the CubeMars driver from the motor supply.
5. Upload the sketch.
6. Open Serial Monitor at `115200 baud`.
7. Type:

   ```text
   status
   ```

   Confirm that voltage and temperature values are returned. If not, fix serial
   wiring or baud rate before commanding motion.

8. Type:

   ```text
   multi
   ```

   Multi-turn mode is recommended for signed relative movement commands. The
   CubeMars manual describes this mode as `+/-100` turns.

9. Type:

   ```text
   zero
   ```

   This sets the current motor position as the temporary origin. Temporary origin
   is cleared when the driver loses power.

10. Send a small test move:

   ```text
   +5.0
   ```

11. If the direction is correct, continue with larger commands such as:

    ```text
    +90.0
    -45.5
    ```

12. Type `status` again to inspect voltage, current, position, speed, and
    temperature while testing.

## UART packets used by the sketch

All packets are framed as:

```text
0x02 length payload crc_high crc_low 0x03
```

CRC is CRC-16/CCITT with polynomial `0x1021` and initial value `0`.

### Position-velocity command

The movement commands use CubeMars `COMM_SET_POS_SPD`:

- Command byte: `91` / `0x5B`
- Payload length: 13 bytes
- Payload:

| Bytes | Value |
| --- | --- |
| 0 | `COMM_SET_POS_SPD` / `0x5B` |
| 1..4 | target position, signed int32, degrees x 1000, big-endian |
| 5..8 | speed limit, signed int32, ERPM, big-endian |
| 9..12 | acceleration limit, signed int32, ERPM/s, big-endian |

### Set zero/origin command

The `zero` command sends:

```text
COMM_SET_POS_ORIGIN, 0x01
```

### Multi-turn and single-turn commands

The `multi` command sends `COMM_SET_POS_MULTI` plus four zero bytes. The `single`
command sends `COMM_SET_POS_SINGLE` plus four zero bytes.

### Status command

The `status` command sends:

```text
COMM_GET_VALUES
```

The response is parsed according to the CubeMars manual for MOS temperature,
motor temperature, output/input current, `Id`/`Iq`, speed, input voltage, status
code, outer-loop position, motor ID, and `Vd`/`Vq`.

## Safety notes

- Keep hands and loose objects away from the motor during activation.
- Use current-limited bench power for early tests if practical.
- Confirm the driver voltage rating and motor phase wiring before applying
  power.
- Add an external emergency stop or power disconnect for real mechanisms.
- The `zero` command is temporary. Do not rely on it as a safety-rated homing
  procedure.
