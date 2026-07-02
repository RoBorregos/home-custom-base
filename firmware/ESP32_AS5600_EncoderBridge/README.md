# ESP32 + AS5600 → ODrive incremental-encoder bridge

Bridges an AS5600 magnetic angle sensor (I2C-only — no SPI, no native ABI/quadrature
output) to a synthesized quadrature A/B + Z-index signal the ODrive v3.6 already
understands natively via `ENCODER_MODE_INCREMENTAL`. No ODrive firmware changes
needed — this replaces the coarse external-Hall-tachometer workaround (see
`../../ODriveV3_6_Test.md`) with a real commutation-grade position reference,
enabling true `AXIS_STATE_CLOSED_LOOP_CONTROL` instead of forced-commutation
`LOCKIN_SPIN`.

## Why a bridge at all
The AS5600 only exposes I2C (plus an analog/PWM `OUT` pin) — no SPI absolute mode
and no incremental ABI outputs, so it can't talk to the ODrive directly in any
mode the firmware supports. An **AS5047P** would (native `ENCODER_MODE_SPI_ABS_AMS`,
zero firmware/bridge needed) — this project exists because the AS5600 was already
on hand. The tradeoff: real closed-loop commutation now depends on the ESP32 never
missing a beat. See the "Risks" section below before trusting this at full current.

## Hardware
- ESP32-WROOM-32 dev board
- AS5600 breakout, diametric magnet centered on the motor shaft end per the AS5600
  datasheet's air-gap spec (same mounting rules as any AMS-style sensor: axial gap
  ~0.5–3 mm depending on magnet, tight radial centering — a few tenths of a mm off
  axis shows up as angle error/current-loop noise)

## Wiring
| AS5600 | ESP32       |
|--------|-------------|
| VCC    | 3V3         |
| GND    | GND         |
| SDA    | GPIO21      |
| SCL    | GPIO22      |

| ESP32 out | ODrive v3.6 GPIO | Signal |
|-----------|------------------|--------|
| GPIO25    | GPIO12           | A      |
| GPIO26    | GPIO13           | B      |
| GPIO27    | GPIO14           | Z (index) |
| GND       | GND              | common ground — required |

ESP32 GPIO is 3.3 V logic, matching the ODrive's STM32F405 inputs directly — no
level shifting needed.

## ODrive-side config
```python
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr  = 4096          # AS5600 native 12-bit resolution
odrv0.axis1.config.gpio12_mode  = GPIO_MODE_ENC1
odrv0.axis1.config.gpio13_mode  = GPIO_MODE_ENC1
odrv0.axis1.config.gpio14_mode  = GPIO_MODE_ENC1
odrv0.save_configuration()
odrv0.reboot()   # GPIO alternate-function mode is boot-time-only, see main .md Gotcha #2
```
Then the standard calibration flow (same as any incremental encoder — this is
what the *original* dead S1 setup would have used):
```python
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# wait, check axis1.motor.error == 0
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
# wait, check axis1.encoder.error == 0
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.save_configuration()
```
On every subsequent boot: `AXIS_STATE_ENCODER_INDEX_SEARCH` (finds the absolute
reference from the Z pulse, quick low-current spin) then
`AXIS_STATE_CLOSED_LOOP_CONTROL` directly — no more `LOCKIN_SPIN`.

If the wheel spins opposite to the commanded sign, flip
`encoder.config.direction` rather than re-wiring A/B.

## Firmware design (what `ESP32_AS5600_EncoderBridge.ino` actually does)
- Reads the AS5600's raw 12-bit angle over I2C (Fast-mode, 400 kHz) as fast as
  the bus allows, unwrapping across the 0/4095 boundary into a continuously
  running signed count (`accumulated`).
- A hardware timer ISR at 80 kHz steps an emitted quadrature count toward
  `accumulated` by **at most 1 count per tick**, driving A/B through the
  standard 4-state x4 quadrature sequence. This bounds the edge rate the
  ODrive ever sees and smooths out the bursty arrival of new I2C readings
  instead of jumping instantly on each read.
  - 80 kHz has >3x margin over the fastest edge rate this application needs:
    40 wheel RPM × 9:1 reduction = 360 motor RPM = 6 rev/s × 4096 cpr ≈ 24.6 kHz.
- Z index is **level-based**, not a one-shot pulse: high whenever the emitted
  position is within a small window (±8 counts) of the raw zero-crossing.
  Self-correcting in both spin directions, no extra state to desync.
- WiFi and Bluetooth are explicitly disabled at boot, and CPU is pinned to
  240 MHz — these are the main sources of interrupt jitter that would
  otherwise corrupt the quadrature timing a real closed loop is trusting.
- Serial (115200) prints a throttled status line every 200 ms:
  `raw`, `accum` (unwrapped position), `emitted` (what's actually been output),
  `lag` (accum − emitted — should stay small/near-zero; a growing lag means the
  ISR can't keep up), `i2c_errs` (cumulative failed I2C transactions).

## Risks / things to verify before trusting this at full current
- **The ESP32 is now in the commutation-critical path.** Any timing hiccup —
  missed I2C transaction, an interrupt storm, brownout — corrupts the phase
  reference the ODrive's current loop is actively trusting. This is a more
  direct path to a current spike than the old open-loop hack going wrong.
  Watch `lag` and `i2c_errs` on the serial monitor during initial bring-up.
- Effective resolution is quantized to the AS5600's native 4096 counts/rev →
  ~205 counts per electrical revolution (4096 / 20 pole pairs) — workable but
  grainier than a dedicated SPI absolute chip.
- I2C read latency (not the 80 kHz ISR) is the real bandwidth bottleneck —
  if `lag` grows unbounded during fast moves, the AS5600 read loop can't keep
  up and something (bus speed, cable length/pull-ups) needs attention.
- Bring the motor up **unloaded, at low current**, and watch for smooth
  tracking before trusting this under load — exactly as was done for the
  open-loop bring-up in the main `.md`.

## Arduino environment
The timer setup is guarded by `#if ESP_ARDUINO_VERSION >= ...` and auto-selects
between the Arduino-ESP32 core 3.x (ESP-IDF 5.x) timer API and the core 2.x
legacy API (`timerBegin(num, divider, countUp)` + `timerAlarmWrite`) at compile
time — it builds either way, no manual edits needed.

Note: **Arduino IDE version and ESP32 core/board-package version are separate
things.** IDE 2.3.6 (the app itself) can have any ESP32 board package installed
underneath it. To check which core you actually have: Tools → Board → Boards
Manager → search "esp32" → the installed version is shown next to "esp32 by
Espressif Systems".
