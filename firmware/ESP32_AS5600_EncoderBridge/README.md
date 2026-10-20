# ESP32 + AS5600 → ODrive incremental-encoder bridge

**PRODUCTION SETUP for node 33** — validated end-to-end: bench closed-loop tests
(`odrive_config/closed_loop_test.py`), power-on self-calibration + self-arm, and a
real drive test of the full 4-wheel base over the Bluetooth interface with the STM32
omnibase firmware. (A second approach, SPI-absolute MA732 emulation in
`../ESP32_AS5600_EncoderBridge_SPI/`, solved calibration persistence but was shelved
for commutation instability under motion — see its README.)

Bridges an AS5600 magnetic angle sensor (I2C-only — no SPI, no native ABI/quadrature
output) to a synthesized quadrature A/B signal the ODrive v3.6 already understands
natively via `ENCODER_MODE_INCREMENTAL`. No ODrive firmware changes needed — this
replaces the coarse external-Hall-tachometer workaround (see `../../ODriveV3_6_Test.md`)
with a real commutation-grade position reference, enabling true
`AXIS_STATE_CLOSED_LOOP_CONTROL` instead of forced-commutation `LOCKIN_SPIN`.

**No Z/index output**, by design. The AS5600 has no hardware index channel to begin
with, and a software-emulated one (level-based, near the raw zero-crossing) was
tried and dropped: ODrive's `AXIS_STATE_ENCODER_INDEX_SEARCH` turns out to use the
exact same forced-commutation lockin mechanism as `AXIS_STATE_ENCODER_OFFSET_CALIBRATION`
itself (`encoder.cpp`'s `run_index_search()` calls `run_lockin_spin()`), so it doesn't
avoid the "does this survive load" question either — while our software Z pulse only
found the index 1 out of 3 attempts in testing. Not worth the wire or the firmware
complexity. See `../../ODriveV3_6_Test.md` §11/closed_loop_test.py for the full story
and the reliable alternative: a full offset calibration on every boot (~9s, small
excursion, 100% success rate in testing).

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
| GND       | GND              | common ground — required |

ESP32 GPIO is 3.3 V logic, matching the ODrive's STM32F405 inputs directly — no
level shifting needed.

Note: the motor + AS5600 are currently bench-wired to **M0/axis0** (not M1/axis1
where the wheel will ultimately live) — `ENC0` vs `ENC1` selects which axis a given
GPIO pin's encoder signal routes to; see `../../ODriveV3_6_Test.md` for why.

## ODrive-side config
```python
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr  = 4096          # AS5600 native 12-bit resolution
odrv0.axis0.encoder.config.use_index = False    # no Z wired -- see "No Z/index output" above
odrv0.config.gpio12_mode  = GPIO_MODE_ENC0
odrv0.config.gpio13_mode  = GPIO_MODE_ENC0
odrv0.save_configuration()
odrv0.reboot()   # GPIO alternate-function mode is boot-time-only, see main .md Gotcha #2
```
Then, every boot (this encoder has no absolute reference to persist across a
power cycle without an index, so this always needs to run fresh — see
`closed_loop_test.py`'s `ensure_calibrated_and_ready()` for the actual
implementation used):
```python
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION   # only needed once; pre_calibrated persists
# wait, check axis0.motor.error == 0
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION   # every boot, ~9s, small excursion
# wait, check axis0.encoder.error == 0 and axis0.encoder.is_ready == True
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
```

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
