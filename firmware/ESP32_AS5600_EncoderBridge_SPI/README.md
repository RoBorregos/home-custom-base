# ESP32 + AS5600 → ODrive SPI absolute-encoder bridge (MagAlpha MA732 emulation)

Second, separate approach from `../ESP32_AS5600_EncoderBridge/` (the quadrature
bridge). Instead of emulating a plain incremental encoder — which throws away
the AS5600's inherent absoluteness and forces a fresh offset calibration on
every ODrive boot — this emulates the SPI protocol of a **MagAlpha MA732**
absolute encoder, which the ODrive's fw-v0.5.6 firmware (already flashed on
this board) supports natively via `ENCODER_MODE_SPI_ABS_MA732`. This is also
the same encoder family (MagAlpha, e.g. MA702) the project's other 3 wheels
already use successfully — confirmed via their saved configs
(`spi_encoder0` enabled with real baudrate/pins, `hall_encoder0`/`inc_encoder0`
both `enabled=False`).

**Status: TESTED, then SHELVED — the quadrature bridge
(`../ESP32_AS5600_EncoderBridge/`) is what node 33 actually runs.**
Bench results before shelving:
- **What worked** (proven on real hardware): SPI link solid on the first try
  (byte order + wiring both correct, ~8 kHz ODrive polling, `spi_error_rate=0.0`
  throughout), and — the actual goal — **`pre_calibrated` persisted across a real
  ODrive reboot with zero re-calibration**, booting straight into closed loop.
- **Why it was shelved anyway**: commutation was **unstable under actual motion** —
  velocity commands produced large currents with little/wrong-direction movement,
  escalating to `SPINOUT_DETECTED` + `CURRENT_LIMIT_VIOLATION` trips. Suspected
  root cause: the bridge's I2C-read→SPI-serve latency acts as a small constant
  angle error that the (slow) offset calibration absorbs consistently — the
  calibration reproduced an *identical* `phase_offset_float` across runs, which
  masked the systematic error — but at real speed the phase reference is wrong
  enough to destabilize FOC. If revisited: cut the latency (faster polling,
  or predictive extrapolation of the served angle) before trusting motion again.

Two real bring-up issues were found, root-caused with data, and **fixed in the
sketch** (keep the fixes if reviving this): a glitched-I2C-read plausibility
filter, and halved I2C transaction overhead to close a 2x rate gap vs. the
ODrive's SPI polling. See the sketch's `STATUS` header for details.

## Why this is worth the extra complexity
- **Real persistence.** `pre_calibrated` should actually survive reboots,
  because from the ODrive's point of view this becomes a genuine absolute
  encoder — same as the other 3 wheels. The quadrature bridge's index-search
  workaround only succeeded 1 of 3 attempts and didn't actually solve this
  (see `../../ODriveV3_6_Test.md` §11 and `closed_loop_test.py`).
- **Better resolution.** 14-bit (16384 counts/rev) vs. the quadrature
  approach's 4096.
- **No running state to corrupt.** Every SPI transaction just reports "the
  angle right now" — no count, no unwrap-across-zero logic, none of the
  aliasing risk the quadrature bridge had at fast spin rates.

The cost: a real SPI slave peripheral to get right (timing, mode, frame
format) instead of a simple GPIO timer loop, and it's untested.

## Wiring
| AS5600 | ESP32 |
|--------|-------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

| ESP32 (VSPI, default pins) | ODrive v3.6 | Signal |
|---|---|---|
| GPIO18 | *(locate physically — see below)* | SCK |
| GPIO19 | *(locate physically — see below)* | MISO — **this is our output**, the angle data |
| GPIO23 | *(locate physically — see below)* | MOSI — ODrive's dummy byte, ignored |
| GPIO5  | your chosen spare GPIO | nCS |
| GND | GND | common ground — required |

**SCK/MISO/MOSI are hardwired on ODrive v3.x** — confirmed from the firmware
interface source itself: `SPI_A: {doc: Note that the SPI pins on ODrive v3.x
are hardwired so they cannot be configured through software...}`. They are
*not* part of the numbered `gpioN_mode` system the quadrature bridge used.
Documentation search could not confirm the exact physical connector/silkscreen
label for this board revision — locate the pins physically labeled
`SCK`/`MISO`/`MOSI` on your actual v3.6 board (or its schematic) before wiring.

**nCS is the one line that IS software-selectable** — any spare GPIO, set to
`GPIO_MODE_DIGITAL`, referenced via `encoder.config.abs_spi_cs_gpio_pin`.

ESP32 GPIO is 3.3 V logic, matching the ODrive's STM32F405 inputs directly.

## ODrive-side config
```python
odrv0.config.gpioN_mode = GPIO_MODE_DIGITAL   # N = whichever spare GPIO you wire nCS to
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = N
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_MA732
odrv0.axis0.encoder.config.cpr = 16384        # 14-bit
odrv0.save_configuration()
odrv0.reboot()
```
Then calibration — same routines as before, but this time expected to only
be needed once:
```python
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.save_configuration()
```
On subsequent boots: straight to `AXIS_STATE_CLOSED_LOOP_CONTROL`, no
re-calibration spin — that's the whole point of this approach. Verify this
empirically before relying on it.

## Protocol details (from ODrive fw-v0.5.6 `encoder.cpp`, `MODE_SPI_ABS_MA732`)
- 16-bit SPI frames, **SPI mode 3** (CPOL=1, CPHA=1).
- ODrive (master) sends a dummy `0x0000` and reads back 16 bits; bits `[15:2]`
  are taken as the 14-bit angle, bits `[1:0]` are ignored.
- No checksum/parity required for this mode (unlike the AMS/CUI SPI modes).

## First things to check once flashed and wired
1. Serial monitor: `angle` should track shaft rotation smoothly, `spi_txns`
   should be incrementing (confirms the ODrive is actually clocking
   transactions), `i2c_errs` should stay at 0.
2. On the ODrive: `axis0.encoder.error` should be 0, not
   `ENCODER_ERROR_ABS_SPI_COM_FAIL`/`ABS_SPI_NOT_READY`/`ABS_SPI_TIMEOUT` — if
   you see one of those, the byte order or physical SCK/MISO/MOSI wiring is
   the first thing to re-check (see the sketch's `STATUS` note).
3. Once `encoder.is_ready`, verify `pre_calibrated` actually survives a
   `odrv0.reboot()` without needing offset calibration again — that's the
   entire point of this approach and needs empirical confirmation before
   trusting it.
