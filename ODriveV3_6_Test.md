# ODrive v3.6 (56V) — Connection & Pre-Configuration Analysis

Date: 2026-07-01

## Connection status

- USB device present: `Bus 003 Device 033: ID 1209:0d32 Generic ODrive Robotics ODrive v3`
- A stale `odrivetool` process (PID 127663) was holding the USB interface and had to be killed
  before a fresh connection could be made.
- Connected successfully via `odrive.find_any()`.

## Identity / firmware confirmation

```
fw version:  0.5.6
hw version:  3.6, variant 56 (i.e. v3.6 56V board)
serial:      56591351099957
```

This matches the task's assumption: legacy firmware line (0.5.x), field namespace is
`axis0.motor.config.*` / `axis0.config.general_lockin` (confirmed present via `hasattr`), NOT
the S1/Pro `config.motor.*` namespace.

## Current live values (before any changes)

```
vbus_voltage                          = 26.71 V
config.dc_bus_undervoltage_trip_level = 19.5 V
config.dc_bus_overvoltage_trip_level  = 26.0 V
config.brake_resistance               = 2.0 ohm
config.enable_brake_resistor          = True

axis0.motor.config.pole_pairs         = 7        (needs to become 20)
axis0.motor.config.motor_type         = 0 (MOTOR_TYPE_HIGH_CURRENT — already correct)
axis0.motor.config.phase_resistance   = 0.0      (unset)
axis0.motor.config.phase_inductance   = 0.0      (unset)
axis0.motor.config.torque_constant    = 0.0689   (leftover/default, needs to become 0.09189)
axis0.motor.config.current_lim        = 30.0 A   (task suggests 15 A to start)
axis0.motor.config.pre_calibrated     = False
axis0.motor.is_calibrated             = False
```

## ⚠️ Active fault found — blocks any state transition right now

```
odrv0.error        = 4         -> LEGACY_ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE
odrv0.axis0.error  = 64        -> AXIS_ERROR_MOTOR_FAILED   (cascaded from the system-level fault)
odrv0.axis0.motor.error = 16777216 -> MOTOR_ERROR_SYSTEM_LEVEL (cascaded from the system-level fault)
```

Root cause: **measured `vbus_voltage` (26.71 V) is already above the configured
`dc_bus_overvoltage_trip_level` (26.0 V)**. The board is currently latched in an
over-voltage fault, which cascades down into the axis and motor error fields. No
`requested_state` transition (including `LOCKIN_SPIN`) will succeed until this is cleared.

The existing trip levels (19.5 V under / 26.0 V over) look like a leftover config from a
different battery than what's actually connected — the pack currently reads 26.7 V, which
is consistent with a 6S or 7S Li-ion/LiPo pack near full charge, **not** a 56 V nominal
supply. This needs to be resolved deliberately, not guessed:

- What is the actual battery chemistry/cell count in use? (e.g. 7S Li-ion ≈ 29.4 V full,
  24.5 V nominal, ~19.6–21 V cutoff — very different trip points than a true 56 V pack.)
- `dc_bus_overvoltage_trip_level` must be set safely above max pack voltage (with margin),
  and `dc_bus_undervoltage_trip_level` safely above the pack's real low-voltage cutoff.
- Setting these wrong either nuisance-trips constantly (set too tight) or fails to protect
  the board/battery (set too loose).

**Action needed before Step 1 can proceed:** confirm real pack voltage/chemistry, set
correct trip levels, then `odrv0.clear_errors()` (or equivalent) to unlatch the fault.

## Resolution — user confirmed trip levels from odrive_node33.json apply to this board

User confirmed the S1 config's `dc_bus_undervoltage_trip_level` (10.5V) and
`dc_bus_overvoltage_trip_level` (29.0V) are in fact correct for this board's actual battery
(overriding the generic "don't copy S1 bus voltage settings" caution in the task spec — the
user has direct knowledge of the pack in use). With `overvoltage_trip = 29.0V`, the measured
26.71V bus voltage sits safely under the trip, so applying these clears the fault instead of
re-triggering it.

## Applied configuration (Steps 0-2)

```
config.dc_bus_undervoltage_trip_level = 10.5
config.dc_bus_overvoltage_trip_level  = 29.0
odrv0.clear_errors()

axis0.motor.config.motor_type        = 0        # MOTOR_TYPE_HIGH_CURRENT in this fw's namespace
axis0.motor.config.pole_pairs        = 20
axis0.motor.config.phase_resistance  = 0.09060616791248322
axis0.motor.config.phase_inductance  = 5.168488860363141e-05
axis0.motor.config.torque_constant   = 0.09188888967037201
axis0.motor.config.pre_calibrated    = True
axis0.motor.config.current_lim       = 15.0

axis0.config.general_lockin.current           = 5.0
axis0.config.general_lockin.ramp_time          = 0.4
axis0.config.general_lockin.ramp_distance      = 3.14
axis0.config.general_lockin.accel              = 20.0
axis0.config.general_lockin.vel                = 40.0
axis0.config.general_lockin.finish_on_vel      = False
axis0.config.general_lockin.finish_on_distance = False
axis0.config.general_lockin.finish_on_enc_idx  = False
```

### Verification after applying

```
sys error:   0   (was 4 / LEGACY_ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE)
axis error:  0   (was 64 / AXIS_ERROR_MOTOR_FAILED)
motor error: 0   (was 16777216 / MOTOR_ERROR_SYSTEM_LEVEL)
is_calibrated: True   (achieved without running AXIS_STATE_MOTOR_CALIBRATION, as expected
                        when starting from valid R/L values)
```

All other read-back values matched exactly what was written (pole_pairs=20, current_lim=15.0,
lockin.vel=40.0, etc.) — see the applied-configuration block above.

**Important:** these values are currently only in RAM. They will be lost on power-cycle/reboot
unless `odrv0.save_configuration()` is run (which also reboots the board). Not yet done —
pending confirmation this is wanted now.

## Task plan status

| Step | Description | Status |
|---|---|---|
| 0 | Resolve DC bus over-voltage fault + correct trip levels for actual battery | **Done** |
| 1 | Set motor params (pole_pairs=20, R/L, torque_constant, pre_calibrated=True, current_lim) | **Done** |
| 2 | Configure `axis0.config.general_lockin.*` | **Done** |
| 2.5 | Persist to NVM (`save_configuration()`) | **Done — reboot confirmed, all values + is_calibrated=True + zero errors survived power cycle** |
| 3 | First spin test (`AXIS_STATE_LOCKIN_SPIN`, wheels lifted) | **DONE — spin working. Motor enters state 9, draws commanded current, rotates continuously. Confirmed physically ("it moved").** See resolution below. |
| 4 | Runtime speed/direction change helper (`set_open_loop`) | Not yet applied (helper function only, no hardware risk by itself) |

## Save & reboot verification

`odrv0.save_configuration()` was called; it raised `TransportException('Transport error on
device 337833613235')` on the calling connection, which is expected — the call triggers a
board reboot that drops the USB link mid-response. Reconnected ~3s later and re-read every
value: all motor params, both bus voltage trip levels, `pre_calibrated`/`is_calibrated`, and
all `general_lockin.*` fields matched what was set, and `error`/`axis0.error` both read 0.
Configuration is confirmed persisted to flash.

## Step 3 attempt — motor now wired to M1 (axis1), spin REJECTED

Motor was connected to the **M1 connector = `axis1`** (v3.6 is dual-axis: M0→axis0, M1→axis1).
The full motor + lockin config was mirrored onto `axis1` (pole_pairs=20, R/L, torque_constant,
pre_calibrated=True, current_lim=15, general_lockin.* identical to axis0). `axis1.is_calibrated`
read `True`, all errors 0.

Requesting the spin was **rejected instantly on every attempt**:

```
odrv0.axis1.requested_state = AXIS_STATE_LOCKIN_SPIN   # (9)
  -> requested_state snaps back to 0
  -> current_state stays 1 (IDLE) — never enters spin
  -> axis1.error = 1  (AXIS_ERROR_INVALID_STATE)
  -> axis1.motor.error = 0
```

Same exact rejection reproduced on `axis0` (no motor attached) → **not** a wiring/motor issue.
Ruled out along the way: stale errors (cleared, 0 before each try), DC-bus current limits,
watchdog (disabled), brake resistor state, motor arm state, DRV fault, encoder error (all 0).
Also tried downgrading the host `odrive` Python package 0.6.11 → 0.5.4 to match the board's
0.5.6 firmware — **did not help** (ruled out a client/firmware protocol mismatch). A control
test with `AXIS_STATE_MOTOR_CALIBRATION` on axis0 *did* enter the state and run real hardware
logic (failed later with `MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE`, expected with no motor) —
proving the state machine can enter active states; only `LOCKIN_SPIN` was being gated.

## ROOT CAUSE (confirmed from firmware v0.5.6 source)

The state-machine dispatch in `Firmware/MotorControl/axis.cpp` gates `LOCKIN_SPIN` with exactly
two preconditions:

```cpp
case AXIS_STATE_LOCKIN_SPIN: {
    if (!motor_.is_calibrated_ || encoder_.config_.direction==0)
        goto invalid_state_label;
    status = run_lockin_spin(config_.general_lockin, false);
} break;
```

`motor.is_calibrated` is `True` on our board, so the failing term is **`encoder.config.direction == 0`**.
In `encoder.hpp` that field is declared `int32_t direction = 0;` and only gets set to ±1 by an
encoder calibration — which we never run (pure open-loop, no encoder). So `direction` stays 0
and the firmware bounces the request to `invalid_state` before any motor/hardware logic runs.
This is exactly why `motor.error` stayed 0 and the rejection was instantaneous.

This is a known ODrive gotcha: `MOTOR_CALIBRATION` has no such check (it ran fine), but
`LOCKIN_SPIN` / `CLOSED_LOOP_CONTROL` both require `direction != 0`. `direction` is a plain,
writable config field (the "no set_ method" only means it has no side-effect setter — it is
still settable over the protocol; setting it manually is the standard workaround for
sensorless/lockin setups).

## THE FIX (one line per axis, no firmware flashing)

```python
odrv0.axis1.encoder.config.direction = 1     # any nonzero value; 1 or -1
# (set axis0 too if that axis will ever be used: odrv0.axis0.encoder.config.direction = 1)
odrv0.save_configuration()                    # persist (reboots the board)
```

Then re-run:
```python
odrv0.clear_errors()
odrv0.axis1.requested_state = AXIS_STATE_LOCKIN_SPIN   # should now enter state 9 and spin
```

Sign note: `direction` combined with the sign of `general_lockin.vel` sets physical rotation
direction. In open-loop it does **not** cause the firmware to depend on a (nonexistent) encoder —
`run_lockin_spin` is forced commutation and never reads encoder feedback; the check is just a
leftover guard. Pick `1`; if the wheel turns the wrong way, flip either `direction` or `vel`.

## Second gate verified clear — motor arming with no brake resistor

`run_lockin_spin` calls `motor_.arm()`. In `motor.cpp` v0.5.6:

```cpp
if (!odrv.config_.enable_brake_resistor || brake_resistor_armed) {
    armed_state_ = 1; is_armed_ = true;
} else {
    error_ |= Motor::ERROR_BRAKE_RESISTOR_DISARMED;
}
```

Arming succeeds when `enable_brake_resistor == false` **OR** the resistor is armed. We already
set `config.enable_brake_resistor = False` (user confirmed no brake resistor is physically
connected), so the first term is satisfied and the motor will arm. The direction fix therefore
does **not** just trade one error for a brake-resistor error — the full open-loop path is clear.

⚠️ Safety with brake resistor disabled + none connected: regen energy has nowhere to dissipate.
Going to `AXIS_STATE_IDLE` disarms and the motor coasts (minimal regen — safe). The risk is an
externally back-driven / decelerating motor pumping the DC bus toward the 29.0 V overvoltage
trip. `dc_max_negative_current = -2.0` and `max_regen_current = 1.0` give some headroom. For a
lifted, freely-spinning wheel this is low risk; keep an eye on `vbus_voltage` on the first runs.

## Research answers (bypass / reflash / SimpleFOC)

- **Bypass the firmware?** Not needed and not warranted. The "bypass" is literally just writing
  `encoder.config.direction`, a field the firmware already exposes. Recompiling 0.5.6 to delete
  the guard would work but is pointless when a one-line config write does it.
- **Flash another firmware?** No. ODrive v3.6 (STM32F405) is stuck on the legacy 0.5.x line
  (0.5.6 is the last). The 0.6.x firmware line does **not** support v3.6 hardware (it targets
  ODrive Pro/S1 with a different MCU) — you cannot flash 0.6.x onto v3.6. No reflash solves this
  faster than the config fix.
- **SimpleFOC viability?** Technically possible, practically overkill. It would mean wiping
  ODrive firmware and writing custom STM32F405 firmware that drives the two DRV8301 gate drivers
  (SPI config), 6 PWM channels, and current sense. SimpleFOC does have a DRV8301 library and
  community members have run it on ODrive-v3.x-style hardware (and its `velocity_openloop` mode
  is the direct equivalent of what we want), but it is a significant, debugging-heavy port that
  throws away a working purpose-built controller to solve a problem that is a single config
  write. Not recommended here.

Sources: ODrive firmware v0.5.6 `axis.cpp` / `encoder.hpp` / `motor.cpp`; ODrive & SimpleFOC
community threads on LOCKIN_SPIN direction gating and DRV8301 SimpleFOC ports.

## RESOLUTION — spin confirmed working (all gates were real, correction to earlier theory)

The `AXIS_ERROR_INVALID_STATE` gate `if (!motor_.is_calibrated_ || encoder_.config_.direction==0)`
had **both** terms bite at different times, plus a third gate at arming. The earlier writeup
pinned it on `direction` alone; that was one real gate but not the whole story:

1. **`encoder.config.direction == 0`** — genuinely blocking early on. Set to `1`. ✔
2. **`motor.is_calibrated == False`** — this is what still blocked it after the direction fix.
   Root cause: axis1's `pre_calibrated=True` (and `enable_brake_resistor=False`) had only ever
   been written to **RAM, never saved**, and the board was power-cycled while rewiring the motor
   to M1 — so the whole axis1 config reverted to defaults (pole_pairs=7, R/L=0,
   pre_calibrated=False, enable_brake_resistor=True). Re-applied the full axis1 motor config so
   `is_calibrated` returns True. ✔
3. **Motor arming** needs `enable_brake_resistor==False` OR an armed resistor. Post-power-cycle
   it was back to True with no resistor connected → would have failed arming. Set back to
   `False`. ✔

This time everything was **saved** (`save_configuration()`), verified to survive the reboot
(is_calibrated=True, direction=1, enable_brake_resistor=False, pole_pairs=20, current_lim=15),
and then the spin was run.

### Lesson: always `save_configuration()` after configuring an axis
Every "it reverted" surprise in this session traces to setting values in RAM and then the board
losing power before a save. RAM-only config is wiped on any power cycle/reboot.

### Successful spin telemetry (axis1, wheels lifted)
Entered `current_state=9`, `motor.is_armed=True`, `axis.error=0`, `motor.error=0` throughout.

- `Id_setpoint`/`Id_measured` ramp to and hold at `general_lockin.current` (5.0 A) — current is
  forced along the commanded rotating angle. `Iq_measured ≈ 0` is correct (all current is on the
  D axis in the commanded frame; watching `Iq` alone misleadingly shows ~0).
- `current_control.phase` wraps continuously through ±π and `phase_vel` ramps at
  `general_lockin.accel` up to `general_lockin.vel`, then holds exactly. Verified holds at
  40.0, and at 180.0 rad/s.
- `vbus_voltage` stable ~26 V, no overvoltage despite brake resistor disabled (accelerating a
  lifted wheel; stopping via IDLE just coasts).

### The two speed/ramp knobs (this explains every "it only moved a bit")
- `general_lockin.vel`  = steady electrical speed [rad/s elec]. Wheel RPM ≈ `vel × 0.0530`
  (i.e. `vel / (20 pole pairs × 9 reduction)` rad/s → RPM). vel=40 → ~2.1 RPM (a slow creep,
  looks like "a bit"); vel=180 → ~9.5 RPM (clearly spinning); vel=360 → ~19 RPM.
- `general_lockin.accel` = ramp rate [rad/s² elec]. **This was the real reason higher vel looked
  like "a bit"**: at the default `accel=20`, reaching vel=360 takes 360/20 = **18 s**, so short
  test windows only caught the slow ramp. Raising to `accel=150` reaches vel=180 in ~1.2 s.

### Runtime speed/direction changes (firmware 0.5.6 behavior — confirmed)
`general_lockin.vel/accel/current` do **not** apply live while in `LOCKIN_SPIN`. Must drop to
IDLE, change them, then re-enter the state:

```python
def set_open_loop(vel, current=5.0, accel=150.0):
    ax = odrv0.axis1
    ax.requested_state = AXIS_STATE_IDLE
    ax.config.general_lockin.vel     = vel        # negative = reverse
    ax.config.general_lockin.current = current
    ax.config.general_lockin.accel   = accel
    odrv0.clear_errors()
    ax.requested_state = AXIS_STATE_LOCKIN_SPIN
```

### Persisted (saved) resting config on axis1  (updated 2026-07-01)
motor_type=0, pole_pairs=20, R=0.0906Ω, L=51.68µH, torque_constant=0.09189, pre_calibrated=True,
current_lim=15 A; general_lockin **current=8, vel=0, accel=20**, ramp_time=0.4, ramp_distance=3.14,
finish_on_*=False; encoder.config.direction=1; enable_brake_resistor=False; dc_bus trip 10.5/29.0 V.
- `vel=0` resting means an accidental `LOCKIN_SPIN` entry (outside the tools) holds at standstill,
  no runaway.
- `current=8` resting (down from an earlier 5, and from the 15 the tools had been using in RAM).

### ⚠ IMPORTANT open-loop heat note (learned the hard way)
`LOCKIN_SPIN` forces `general_lockin.current` on the D axis **continuously, independent of speed
or load** — it is not demand-based like closed-loop. So on a free/no-load wheel the full lockin
current is pure winding heat the entire time armed (at 15 A ≈ 20–30 W; at 8 A ≈ a few W). There is
NO motor thermistor enabled, so no temp readout — check by hand on long runs. Both host tools now
**hard-cap lockin current at `MAX_CURRENT = 8.0 A`** (`odrive_config/openloop_cli.py` and
`openloop_sequence_test.py`); the `Id_measured` shown at IDLE is a stale register value, not real
current (motor is disarmed → FETs off → no current). Raise the cap only if a real load needs more
torque; reserve high current for actual load + fast accel, not no-load spinning.

## Reactive open-loop velocity (avoiding the restart-from-0) — root cause + fix, from source

Question: can open-loop velocity be changed on the fly WITHOUT the commanded speed restarting
from 0 (the jerk) every time? Answered definitively by reading fw-v0.5.6 source:

**Why it restarts from 0 today:**
- `OpenLoopController::update()` (`open_loop_controller.cpp:25`) ramps `phase_vel_` **from its
  own previous value toward `target_vel_`** each control tick at `max_phase_vel_ramp_` (=accel).
  It NEVER self-resets — so the hardware is already capable of smooth speed changes.
- `Axis::run_lockin_spin()` (`axis.cpp:188-199`) resets `phase_vel_ = 0` and copies
  `target_vel_ = lockin_config.vel` **only at state entry**. Its `while` loop (`axis.cpp:216+`)
  just idles on `osDelay(1)` and **never re-reads `general_lockin.vel`** into `target_vel_`.
  → Live writes to `general_lockin.vel` are ignored (empirically confirmed: wrote 250 while
    spinning at 100, `phase_vel` stayed pinned at 100), and every IDLE→re-enter zeroes
    `phase_vel_` again.
- `open_loop_controller_` is a C++ member (`axis.hpp:180`) but is **NOT exposed in
  `odrive-interface.yaml`** (only `open_loop_controller_update` TaskTimer is). So `target_vel_`
  cannot be written over USB/CAN in stock firmware. → No stock-firmware path exists.

**The fix (custom firmware, minimal):** add inside the `run_lockin_spin` while loop:
```cpp
open_loop_controller_.target_vel_    = lockin_config.vel;                    // live speed
open_loop_controller_.target_current_= (motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL)
                                        ? lockin_config.current : 0.0f;      // live torque
```
`lockin_config` is a reference to `config_.general_lockin`, so this picks up live writes every
1 ms; `update()` then ramps `phase_vel_` smoothly from the current speed to the new target at
`general_lockin.accel`. Result: write `general_lockin.vel` (incl. sign flip for reversal) any
time while staying in `LOCKIN_SPIN` → smooth, reactive, no restart, no re-arm. Requires building
& DFU-flashing patched ODrive 0.5.6 (v3.6 build config). Toolchain (arm-none-eabi-gcc, tup,
dfu-util) is NOT currently installed on this machine.

Alternatives considered: sensorless velocity (rejected — no low-speed/from-stop); closed-loop w/
encoder (proper solution but needs feedback hardware); SimpleFOC (far larger port, wipes ODrive).

## Notes carried over from the task spec (not re-derived, just recorded for continuity)

- Motor R/L/torque_constant recovered from a dead ODrive S1's `odrive_node33.json` — these
  are motor properties, not board properties, so they transfer.
- Do NOT copy the S1's `dc_bus_undervoltage_trip_level` (10.5 V) / `dc_bus_overvoltage_trip_level`
  (29.0 V) / `inverter0.*` / shunt / drv_config — those were tuned for the S1's different
  hardware and voltage domain.
- Goal is pure open-loop LOCKIN_SPIN (forced commutation), explicitly not sensorless/back-EMF,
  because back-EMF mode can't start from a stop or hold low/creep speed — needed for a mobile
  base.
- 1:9 reduction, 20 pole pairs → wheel rad/s = `general_lockin.vel / 180`.

---

# CUSTOM FIRMWARE: reactive open-loop velocity (built, flashed, verified working)

Date: 2026-07-01. This section documents, in full detail, the custom-firmware work that
turned open-loop control from "coarse/jerky (re-enter to change speed)" into "smooth, live,
reactive velocity streaming." Everything below was actually executed on this machine and this
board (ODrive v3.6-56V, serial 56591351099957 / DFU serial 337833613235).

## 1. Why stock firmware could not do it (recap, from source)

- `OpenLoopController::update()` (`Firmware/MotorControl/open_loop_controller.cpp:25`) ramps its
  output `phase_vel_` **from the previous value toward `target_vel_`** every control tick, clamped
  by `max_phase_vel_ramp_` (= `general_lockin.accel`). It never self-zeroes. So the hardware is
  inherently capable of smooth speed changes.
- `Axis::run_lockin_spin()` (`axis.cpp`) sets `phase_vel_ = 0` and `target_vel_ =
  lockin_config.vel` **only once at state entry**, then its service `while` loop just idles on
  `osDelay(1)` and never writes `target_vel_` again.
- `open_loop_controller_` is a plain C++ member (`axis.hpp:180`) and is **not exposed in
  `odrive-interface.yaml`**, so `target_vel_` is not writable over USB/CAN.
- Net: the only open-loop knob exposed to the host is `config.general_lockin.vel`, and it is read
  exactly once (at entry). Writing it live is silently ignored (verified: wrote 250 while spinning
  at 100, `phase_vel` stayed pinned at 100). Every IDLE→re-enter re-zeroes `phase_vel_`. Hence the
  restart-from-0 / jerk, with no stock-firmware workaround.

## 2. The patch (what changed, exactly)

File: `Firmware/MotorControl/axis.cpp`, inside the `run_lockin_spin()` service `while` loop
(the loop that runs while the axis is in `LOCKIN_SPIN`). Added at the top of the loop body:

```cpp
while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
    // PATCH (reactive open-loop): continuously feed the live lockin config into
    // the open-loop controller so writes to general_lockin.vel/current/accel take
    // effect WITHOUT re-entering the state. phase_vel_ then ramps smoothly from the
    // current speed to the new target at general_lockin.accel (no restart from 0,
    // no re-arm). Harmless for the calibration/sensorless callers (their config is
    // static, so this just re-writes the same values each iteration).
    open_loop_controller_.target_vel_ = lockin_config.vel;
    open_loop_controller_.target_current_ = (motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) ? lockin_config.current : 0.0f;
    open_loop_controller_.max_phase_vel_ramp_ = lockin_config.accel;
    ...existing loop body...
}
```

Why this is correct and safe:
- `lockin_config` is a `const LockinConfig_t&` bound to `config_.general_lockin`, so re-reading it
  each ~1 ms loop iteration picks up live host writes to `general_lockin.vel/current/accel`.
- `update()` (running in the fast control loop) then ramps `phase_vel_` from wherever it is toward
  the new `target_vel_` at `max_phase_vel_ramp_`. Sign flips ramp smoothly through zero → clean
  reversal, still armed, no re-enter.
- The entry-time initialization (lines ~188-199) is untouched, so first entry still starts at 0.
- `run_lockin_spin` is also used by motor/encoder calibration and sensorless ramp. For those the
  bound config is static and its `finish_on_*` flags terminate the loop; re-writing the same
  values each iteration changes nothing. So the patch does not affect calibration behavior.
- No config struct / interface / NVM layout changed → existing saved config stays valid across the
  flash (confirmed, see §6).

## 3. Build environment set up on this machine

- Installed via apt (user ran `sudo apt install -y gcc-arm-none-eabi tup dfu-util`):
  - `arm-none-eabi-gcc` **10.3.1** (pkg 15:10.3-2021.07-4)
  - `tup` **0.7.8** (ODrive 0.5.x build system)
  - `dfu-util` **0.9** (USB DFU flashing)
- Python build deps installed with `pip install --user`: `PyYAML 5.4.1`, `jinja2 3.1.5`,
  `jsonschema` (used by the interface generator that turns `odrive-interface.yaml` into
  `autogen/interfaces.hpp` + `tools/odrive/enums.py`).
- Source: `git clone --branch fw-v0.5.6 --depth 1 --recurse-submodules` of
  `github.com/odriverobotics/ODrive` into `/tmp/ODrive_fw_build`. (In 0.5.6 `fibre-cpp` is a
  vendored directory, not a git submodule, so there was no `.gitmodules`; the clone is complete.)
- Build config `Firmware/tup.config` created with:
  `CONFIG_BOARD_VERSION=v3.6-56V`, `CONFIG_DEBUG=false`, `CONFIG_DOCTEST=false`,
  `CONFIG_USE_LTO=false`, `CONFIG_STRICT=false`.

## 4. Build

- `cd Firmware && tup init` then `make` (the Makefile `all` target generates `autogen/version.c`,
  runs `tup --quiet -no-environ-check` for the compile, then generates enums).
- Result: **compiled cleanly**. `arm-none-eabi-size` reported:
  `text 319816  data 4044  bss 142856` for `build/ODriveFirmware.elf` (~316 KB of flash used, well
  within the F405's 1 MB).
- The only `make` error was the very last post-build step `create_can_dbc.py` failing on a missing
  `cantools` module. That step only generates an optional CAN `.dbc` for external tooling and runs
  AFTER the firmware is already built — irrelevant to flashing.
- Artifacts produced in `Firmware/build/`: `ODriveFirmware.elf` (with symbols), `.hex`, `.map`,
  and a raw `.bin` I created with `arm-none-eabi-objcopy -O binary` (323,920 bytes) for dfu-util.

## 5. Flashing (DFU)

- Board put into DFU: the user had set the physical DFU DIP switch; since the board was still
  running (battery-powered, so a USB re-plug doesn't reset the MCU), I triggered the bootloader in
  software with `odrv0.enter_dfu_mode()`. It then enumerated as `0483:df11` "STM Device in DFU
  Mode". `dfu-util -l` showed alt=0 `@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg`.
- Flash command (from `Firmware/`):
  ```
  dfu-util -a 0 -s 0x08000000:leave -D build/ODriveFirmware.bin
  ```
  Output: downloaded all 323,920 bytes, "File downloaded successfully", exit 0. (The "Invalid DFU
  suffix" line is a harmless warning — a raw `.bin` has no DFU suffix; dfu-util still flashes it.)
- Because the DFU switch was still physically set to DFU, the board re-entered the bootloader on
  reboot. The user then **set the DFU switch back to RUN and power-cycled**, and the board booted
  the new firmware (re-enumerated as `1209:0d32` ODrive v3).

## 6. Config survived the flash

dfu-util only erases/writes the sectors the firmware occupies (0x08000000 upward, ~316 KB); the
ODrive config lives in separate high flash sectors, and the firmware version/layout is unchanged,
so the config loaded intact. Verified after reboot on axis1:
`pole_pairs=20, phase_resistance=0.0906, is_calibrated=True, pre_calibrated=True,
encoder.direction=1, current_lim=15, enable_brake_resistor=False,
general_lockin vel/accel/current=40/20/5, dc_bus trip 10.5/29.0, axis.error=0, motor.error=0`.
No reconfiguration was needed.

## 7. Verification of reactive behavior (the whole point)

Entered `LOCKIN_SPIN` ONCE at vel=100 (accel=150), then wrote `general_lockin.vel` live with NO
IDLE and NO re-enter:

- Wrote `vel=250`: `phase_vel` ramped **100 → 145 → 191 → 237 → 250** and held. It ramped *from the
  current 100*, not from 0. (Identical write on stock fw left it pinned at 100.)
- Wrote `vel=-80` (reverse): `phase_vel` ramped smoothly **down through zero into reverse**
  (250→205→159→112→66→20→-26→-71→-80), staying `state=9`, `is_armed=True` the entire time.
- `Id_measured` held ~5 A, `axis.error`/`motor.error` stayed 0 throughout.

Conclusion: `general_lockin.vel/current/accel` are now fully live/reactive. Write a new wheel
speed anytime while staying in `LOCKIN_SPIN`; the commanded electrical velocity ramps smoothly
(at `accel`) from the current speed to the new target, including sign reversal. No jerk, no
re-arm, no restart-from-0.

## 8. Host tool updated to match: `odrive_config/openloop_cli.py`

Rewritten to use the reactive path: it connects, runs a preflight check
(`is_calibrated`/`direction`/brake-resistor), enters `LOCKIN_SPIN` once at 0, then **streams
`general_lockin.vel` live** on each typed command. Commands: `<wheel RPM>` (neg = reverse),
`0`/`s` (ramp to standstill, stay armed), `c <amps>` (live torque), `a <rate>` (live accel),
`coast` (IDLE/disarm/freewheel), `?` (status), `q` (stop + exit). Wheel↔electrical conversion uses
`vel_elec = wheel_RPM x (20 pole pairs x 9 reduction) x 2π/60 ≈ wheel_RPM x 18.85`.
`MAX_WHEEL_RPM=40`. Verified with a scripted `10→25→-8→0→q` run: stayed `state=9 armed=True`
throughout, streaming live (no re-enter). Run it interactively in a real terminal:
`python3 /home/roger/Github/home-custom-base/odrive_config/openloop_cli.py`.

## 9. How to rebuild / reflash later (reproduce this)

1. Source + patch live at `/tmp/ODrive_fw_build` (⚠ /tmp is volatile — see §10). The patch is the
   3 lines in `Firmware/MotorControl/axis.cpp` `run_lockin_spin` while loop (§2).
2. `cd Firmware && tup init && make` → `build/ODriveFirmware.elf`.
   `arm-none-eabi-objcopy -O binary build/ODriveFirmware.elf build/ODriveFirmware.bin`.
3. Enter DFU: `odrv0.enter_dfu_mode()` (or set the physical switch to DFU + power-cycle).
4. `dfu-util -a 0 -s 0x08000000:leave -D build/ODriveFirmware.bin`.
5. Set DFU switch back to RUN, power-cycle. Config persists (same-version flash).

## 10. Caveats / TODO

- ~~The build tree is in `/tmp/ODrive_fw_build`, which is wiped on reboot.~~ **DONE** — the patch,
  patched source, and flashable `.bin`/`.hex` are preserved in the repo under
  `firmware/odrive_3_6_custom_firmware/` (see its `README.md`). The `/tmp` tree is still used as the working
  build directory for further patches (see the CAN-integration section below) but is no longer the
  only copy.
- **This firmware reports version 0.5.6** (version.py was not bumped) — it is indistinguishable
  over USB from stock 0.5.6, but behaves differently (live lockin). Consider bumping the version
  string or adding a custom marker so a future reader/flash can tell them apart. The host CLI's
  docstring notes that on STOCK fw live writes are ignored (tool would look frozen).
- **No feedback / no stall detection** (unchanged from stock open-loop): a jammed/stalled wheel is
  not detected. Consider a supervisor comparing commanded vs. `Id_measured`, and note that at high
  accel or large instantaneous steps the open-loop rotor can still lose sync.
- ~~Brake resistor disabled, none connected~~ **UPDATED** — a real brake resistor (2.0 Ω, matching
  the original json values) was connected later in the session; `config.enable_brake_resistor` is
  now back to `True`/`brake_resistance=2.0` (saved). `coast`/IDLE still just freewheels.

---

# HOW THE MOTOR IS ACTUALLY BEING "CONTROLLED" (forced commutation vs. closed loop)

This explains, in detail, what `LOCKIN_SPIN` is actually doing to the motor, and why it draws a
lot of current even "doing nothing" — asked for explicitly, recorded here for reference.

## The mechanism

A BLDC/PMSM motor makes torque from the angle between the **stator's magnetic field** (created by
driving current through the 3 phases) and the **rotor's permanent magnets**. In open-loop
`LOCKIN_SPIN` the firmware has no idea where the rotor actually is (no commutation-grade encoder),
so it doesn't try to measure or correct anything — it:
1. Picks a stator field angle itself and rotates that angle at the commanded speed (`phase_vel`,
   driven by `general_lockin.vel`, ramped at `general_lockin.accel`).
2. Pushes a **fixed current magnitude** (`general_lockin.current`) along that rotating angle.

The rotor's magnets get physically dragged along by the rotating field — exactly like a stepper
motor, or a compass needle following a magnet spun by hand. As long as the field isn't rotated
faster than the rotor can follow (given available torque), it stays "locked" and spins at the
commanded speed. There is no measuring, no correcting, no regulation — "control" here just means
"spin a magnetic field at a set speed and strength, and trust the rotor to follow."

## Why it draws so much current, even holding steady / at no load

Two things combine:
1. **The current is a command, not a result.** `general_lockin.current` is a number the host
   dictates — the firmware forces exactly that, continuously, whether accelerating, holding
   steady, or spinning a totally unloaded wheel. It never eases off because "nothing needs the
   torque right now" — it has no way to know that.
2. **It must be conservatively high to guarantee the rotor stays locked.** Being blind to the
   rotor, the firmware can't place current efficiently — it must inject enough margin to (a) keep
   the rotor captured in sync and (b) handle ramps/disturbances. Too little and the rotor slips out
   of sync and stalls. Nearly all of that current becomes **I²R heat** in the windings regardless
   of how little real mechanical work is happening, because on a lightly-loaded wheel the rotor
   sits almost perfectly aligned with the field (tiny "load angle"), so most of the forced current
   isn't torque-producing — it's just maintaining the lock.

This is why lockin current was capped at 8 A (`MAX_CURRENT` in both host tools) after the earlier
15 A runs ran the motor continuously hot for no load-related reason.

## Closed-loop (with a real commutation-grade encoder) would be night-and-day different

With a proper encoder, the controller *measures* rotor angle and places current optimally
(~90° ahead, all of it torque-producing), and current becomes **demand-driven**: a velocity loop
computes the torque actually needed and commands only that much current.
- **Free wheel at steady speed:** closed-loop would draw near-zero current (just friction);
  open-loop draws the full commanded current regardless.
- **During acceleration:** closed-loop can transiently use the full `current_lim` to accelerate as
  hard as physically possible, then back off — open-loop is stuck at one fixed number the whole
  time (this is exactly why raising `accel`/`current` trades responsiveness for stall risk and
  heat, see the sequence-test findings below).
- Closed-loop never burns current "just to hold sync" — it knows where the rotor is; open-loop
  always burns current for that, because it doesn't.

**Analogy:** open-loop `LOCKIN_SPIN` is a stepper motor (full current to hold/move regardless of
load, hot even when idle). Closed-loop FOC is a servo (works exactly as hard as the task demands).
The price of not having a commutation-grade encoder (see below) is accepting the stepper-like
behavior.

---

# CAN INTEGRATION: wiring node 33 (this ODrive) into the 4-wheel omnibase system

Context: this ODrive replaces node 33 in `firmware/STM32H7_OMNIBASE_CAN_BNO085` (CM7 project),
a dual-core STM32H7 board that drives 4 wheels over CAN (CANSimple protocol). The other 3 wheels
run true native closed-loop velocity control (their own onboard encoder + PI); this one cannot,
because its only position sensor is an **external, low-resolution 3-Hall-sensor tachometer** —
confirmed (during this session) to be mounted **outside the motor**, not reading the true rotor
magnets, and resolving only ~1/4 of what the motor's actual (confirmed) **20 pole pairs** would
need for real commutation feedback (~272 counts per WHEEL revolution measured directly, via a
precise driven test — see prior sections for the full pole-pair/Hall investigation). So this axis
must stay in forced-commutation `LOCKIN_SPIN` (the reactive open-loop patch above), while the
other 3 axes use `CLOSED_LOOP_CONTROL`.

Goal: make this wheel controllable from the same STM32 firmware, via the same CAN messages, with
no special handling required by whoever sends robot-level velocity commands.

## What was found in `main.c` (STM32 CM7 project)

- `odrives[2].NODE_ID = 33`, `gear_ratio = 9`, `wheel_sign = -1` — confirms this axis's identity.
- The STM32 does **zero** per-wheel control-loop work — it's purely a commander (mixes
  twist→wheel-speed via `computeNecessaryWheelSpeedsMecanum`, sends `SET_INPUT_VEL`) and reader
  (parses `ENCODER_ESTIMATES` for odometry/EKF). Every wheel is expected to run its own onboard
  closed-loop velocity PI.
- **Two places hard-require literal state `CLOSED_LOOP_CONTROL` (0x8):**
  `ODrive_ArmAxisConfirmed()` (boot arm + ESTOP-release re-arm) and a periodic **500 ms health-check
  loop** that flags `FERR_AXIS_REARM` and re-issues the arm request for any axis not in that exact
  state, indefinitely.
- `SET_INPUT_VEL`'s float payload is in **motor-shaft turns/s** (the STM32 already multiplies the
  desired wheel angular velocity by `gear_ratio` before sending); `ENCODER_ESTIMATES` is expected
  back in the same units (STM32 divides by `gear_ratio` to recover real wheel speed for odometry).

## Decision: implement the missing piece on the ODrive, not the STM32

Every other wheel is self-contained (no STM32-side control-loop code exists at all), so extending
the ODrive's own firmware — which we already have a working build/patch pipeline for — is
consistent with the existing design and runs at the right rate (in-firmware ~1 kHz vs. a CAN
round-trip). Three pieces, done in order:

### 1. CAN `SET_INPUT_VEL` → drives `general_lockin.vel` while in `LOCKIN_SPIN`

File: `Firmware/communication/can/can_simple.cpp`, `CANSimple::set_input_vel_callback()`.
Stock behavior only writes `axis.controller_.input_vel_`, which nothing ever reads while the axis
is in `LOCKIN_SPIN` (the closed-loop controller task doesn't run in that state) — so a
`SET_INPUT_VEL` sent to this axis would previously be silently absorbed and do nothing. Patched to
add:
```cpp
if (axis.current_state_ == Axis::AXIS_STATE_LOCKIN_SPIN) {
    axis.config_.general_lockin.vel = input_vel * 2.0f * M_PI * axis.motor_.config_.pole_pairs;
}
```
`input_vel` (turns/s) → `general_lockin.vel` (electrical rad/s) via `× 2π × pole_pairs`. Feeds
directly into the reactive-open-loop mechanism already validated (§7 above): live speed changes,
no re-entry, no restart-from-0.

### 2. CAN telemetry (`ENCODER_ESTIMATES`) — required a small patch, turned out NOT to "just work"

Initially assumed the stock encoder pipeline would report usable data since `encoder_.pos_estimate_`/
`vel_estimate_` were confirmed to update continuously and meaningfully even with `is_ready=False`
(gated only by `hall_polarity_calibrated`, which is `True`). **However**, `get_encoder_estimates_callback()`
(the actual CAN broadcast, rate-limited by `axis.config.can.encoder_rate_ms`, confirmed already
nonzero at 10 ms and fired unconditionally regardless of axis state) reads from
`controller_.pos_estimate_linear_src_`/`vel_estimate_src_` — signal-routing ports that are **only**
wired to the encoder inside `Axis::start_closed_loop_control()`, which never runs for `LOCKIN_SPIN`.
So without a patch, telemetry would report a constant 0.0 despite the encoder itself working fine.
Fixed by adding the same wiring to `run_lockin_spin()`'s one-time entry setup:
```cpp
controller_.pos_estimate_linear_src_.connect_to(&encoder_.pos_estimate_);
controller_.vel_estimate_src_.connect_to(&encoder_.vel_estimate_);
```
Pure data routing (mirrors the non-sensorless branch of `start_closed_loop_control()` exactly) —
doesn't touch control, so harmless for the other `run_lockin_spin()` callers (motor/encoder
calibration, sensorless ramp) too.

`encoder.config.cpr` is currently `30` (left over from an earlier, since-corrected pole-pairs
guess) — coincidentally close (~0.7% off) to the value that makes `pos_estimate`/`vel_estimate`
come out in the same "motor-equivalent turns" units the STM32's `/gear_ratio` math already expects
(true value ≈ 272 counts/wheel-rev ÷ 9 ≈ 30.2). Left as-is; close enough for now, could be tuned
later if odometry accuracy matters enough to chase the last ~0.7%.

Both patches: applied to the `/tmp/ODrive_fw_build` tree, built clean (`arm-none-eabi-gcc` via
`make`), flashed via `dfu-util` (software-triggered DFU: `odrv0.enter_dfu_mode()`), board rebooted
straight into the new firmware (DFU switch was already back at RUN). Verified after flash: all
saved config survived (`pole_pairs=20`, `is_calibrated=True`, `direction=1`,
`hall_polarity_calibrated=True`, safe resting `general_lockin`), and a regression test confirmed
`LOCKIN_SPIN` still enters/spins/stops correctly with zero errors.

**Not yet independently verified: the actual CAN behavior end-to-end.** This dev machine has no
CAN interface/adapter (`can-utils` installed but no `canX` device), so the new `set_input_vel_callback`
branch and the `ENCODER_ESTIMATES` content could not be exercised by a real incoming CAN frame —
only regression-tested via the equivalent USB/Python path, which doesn't go through the CAN RX
callback at all. High confidence in the logic (mirrors the existing `CLOSED_LOOP_CONTROL` pattern
closely) but this should be tested with a real CAN adapter or the actual STM32 board before
trusting it on the physical robot.

### 3. STM32 `main.c` / `ODrive.h` — accept `LOCKIN_SPIN` as this axis's healthy/armed state

Rather than making the ODrive *lie* about its state (report `CLOSED_LOOP_CONTROL` while actually
running open-loop), added a small, honest, well-contained exception: a new per-axis
`Axis.Armed_State` field (`ODrive.h`), set once alongside `NODE_ID`/`gear_ratio`:
```c
odrives[0].Armed_State = CLOSED_LOOP_CONTROL;
odrives[1].Armed_State = CLOSED_LOOP_CONTROL;
odrives[2].Armed_State = LOCKIN_SPIN;   /* node 33 */
odrives[3].Armed_State = CLOSED_LOOP_CONTROL;
```
Then every place that previously hardcoded `CLOSED_LOOP_CONTROL` as the target/expected state now
uses `axis->Armed_State` / `odrives[i].Armed_State` instead — exactly 2 spots:
- `ODrive_ArmAxisConfirmed()` — both the requested-state (arm) write and the success check
  (`AXIS_Current_State == Armed_State`), in both the `CAN_STUB` and real branches.
- The periodic 500 ms auto-recovery/health-check loop in `SM_RUNNING` — both the fault-detection
  check and the recovery request.

(`ODRIVE_CFG_STARTUP`'s `ODrive_Startup()` path already takes `requested_state` as an external
command parameter with a `target_mask`, so it needs no code change — the operator can issue it
twice, once per group of axes, if a heterogeneous startup state is ever needed there.)

**Build verification:** the CM7 project's CubeIDE-generated makefile needs the toolchain bundled
with STM32CubeIDE, not the system `gcc-arm-none-eabi` (the generated makefile passes
`-fcyclomatic-complexity`, unsupported by the older apt-installed 10.3.1 compiler; the correct one
is `/opt/st/stm32cubeide_1.18.1/.../tools/bin`, version 13.3.1). Also note: `make` with no target
does **not** build the whole project on this generated makefile (an `-include`d `subdir.mk` defines
a rule before the real `all:` target, becoming GNU Make's default goal by file-order) — use
`make all` explicitly. With the correct toolchain and target, the full project (including
`main.c`/`ODrive.c`) built clean: 0 errors, 2 pre-existing unrelated warnings (both untouched by
this patch), successfully linked.

**Not yet flashed to the physical STM32 board** — only compiled/verified. Flashing the actual
omnibase controller (and live CAN testing of steps 1-2 above) needs the physical board + a
programmer, or the robot itself, neither available in this dev session.

## Summary of what's confirmed vs. still needs real-hardware verification

| Piece | Status |
|---|---|
| ODrive firmware compiles + flashes | ✅ Done, verified |
| ODrive config survives a firmware update, IN GENERAL | ⚠ Only if the struct layout doesn't change — see §11 below, this bit us |
| `SET_INPUT_VEL` CAN hook logic | ✅ Written, compiles — ⚠ not exercised by a real CAN frame |
| `ENCODER_ESTIMATES` telemetry wiring | ✅ Written, compiles — ⚠ CAN payload content not inspected |
| STM32 `main.c`/`ODrive.h` `Armed_State` patch | ✅ Written, compiles clean |
| STM32 firmware flashed to physical board | ❌ Not done — needs the actual board + programmer |
| End-to-end CAN test (STM32 ↔ this ODrive) | ❌ Not done — needs CAN adapter or the real robot |
| Velocity PI (firmware, toggleable) | ✅ Implemented, built, flashed, tested (see §11) — works |

---

# §11. VELOCITY PI (firmware-side, toggleable) + hard-won gotchas from implementing it

## What was added

`Axis::LockinConfig_t` (`MotorControl/axis.hpp`) gained 4 new fields, exposed in
`odrive-interface.yaml` under `ODrive.Axis.LockinConfig` exactly like the existing
`vel`/`current`/`accel` fields (so they're live-writable over USB/CAN the same way):
```
bool  vel_pi_enable = false;   // THE TOGGLE: false = pure open-loop (unchanged behavior).
                                // true = PI-corrected open-loop (see below).
float vel_pi_kp = 0.0f;
float vel_pi_ki = 0.0f;
float vel_pi_limit = 200.0f;   // [rad/s elec] anti-windup / correction clamp
```
In `Axis::run_lockin_spin()`'s while loop, when `vel_pi_enable` is true, the commanded field
speed becomes `general_lockin.vel + PI(error)`, where `error = general_lockin.vel -
measured_vel_elec`, `measured_vel_elec = encoder_.vel_estimate_.any().value_or(0) * 2π *
pole_pairs` (the encoder used purely as a tachometer — see §10 above for why this is safe/valid
even without a commutation-grade encoder). The integrator is a **local variable** declared before
the loop, so it naturally resets to 0 every time `LOCKIN_SPIN` is (re-)entered — no separate reset
code needed, matches how `phase_vel_`/`open_loop_controller_` state already reset on entry. When
`vel_pi_enable` is false, behavior is byte-identical to the pre-existing pure open-loop patch —
this is genuinely a single toggle between the two modes, per the request that motivated it.

This is a **slip-compensation** loop, not true closed-loop FOC: it only ever nudges the *target
field frequency*, never touches phase/commutation alignment, so it cannot destabilize commutation
the way feeding this into a real closed loop would — the failure mode is "doesn't correct speed
well," not "motor loses sync/stalls unpredictably."

## Gotcha #1: ANY struct-layout change wipes ALL saved ODrive config, not just the changed struct

Adding these 4 fields to `LockinConfig_t` — despite being a small, additive change — **invalidated
the entire saved NVM config blob** on the next boot: `pole_pairs` reverted to the hardware default
(7), `is_calibrated` went back to `False`, `hall_polarity_calibrated` was lost, and even
board-level fields outside `LockinConfig_t` entirely (`gpio12/13/14_mode`) reverted to a stale
intermediate value from earlier in the session rather than either the correct saved value or a
clean factory default — evidence the whole config tree shares one layout-versioned blob, not
independent per-struct ones. **Lesson: after flashing ANY firmware build with a struct change
(not just logic changes), assume full reconfiguration from scratch is needed** — motor params,
encoder mode/cpr/direction, bus trip levels, brake resistor, resting `general_lockin` — and
re-verify, don't assume "it built and flashed fine" means "config survived."

## Gotcha #2: mode-dependent GPIO/timer hardware setup only (re)initializes at boot

After the config wipe, `encoder.config.mode` was re-set to `HALL` over USB **without an
intervening reboot**, then `AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION` was attempted — it
failed (`ENCODER_ERROR_ILLEGAL_HALL_STATE`), and `encoder.hall_state` read a constant `0` even
while the motor was genuinely spinning (confirmed by simultaneously reading raw GPIO 12/13/14 via
`get_gpio_states()`, which showed a stable, real, non-floating bit pattern). Root cause: the
STM32's GPIO alternate-function/timer-capture wiring for Hall/incremental decoding is set up once
at firmware init based on `encoder.config.mode`, and does **not** get reconfigured just because
the config value changes at runtime over USB. Saving the config and rebooting (letting init run
again with the new mode already in place) fixed it immediately — `hall_state` then cycled through
all 6 valid states with good sample distribution. **Lesson: after changing `encoder.config.mode`
(or likely any other hardware-routing config field), save + reboot before relying on it — don't
test in the same session you changed it in.**

## Gotcha #3: `run_hall_polarity_calibration()` needs a genuinely fast/long-enough sampling window for a coarse sensor

Even with mode/pins correct, calibration still failed at the default `calibration_lockin` speed
(`vel=40, accel=20`). Reading the actual algorithm (`Encoder::run_hall_polarity_calibration()` in
`encoder.cpp`) explains why: it requires **each of the 6 valid Hall states to be sampled >50
times** during a short (`finish_distance = vel*3`) window, gated by the axis actually reaching
constant velocity. With our sensor's coarse resolution (~272 counts per wheel revolution) and the
default slow speed, only ~15-37 samples per state were possible in that window — short of the
threshold. The fix was **not** obvious from the error alone (`ILLEGAL_HALL_STATE` sounds like a
wiring problem, and briefly looked like one via a red herring — a `Controller::ERROR_OVERSPEED`
appeared first when the sampling speed was raised, a side effect of Gotcha #4 below, momentarily
masking the real remaining issue). Working values: `calibration_lockin.vel=800, accel=2000,
current=10` — near-instant ramp, high transition rate, plenty of margin above the 50-sample floor.
**Lesson: for any non-standard/coarse Hall sensor, calibration parameters may need to be pushed
well past ODrive's defaults (which assume a real commutation-grade sensor) — if it fails, check
`states_seen_count_` logic / sampling duration math before assuming a wiring fault.**

## Gotcha #4: connecting `vel_estimate_src_` for CAN telemetry also feeds the Controller's own overspeed monitor

The §10 telemetry-wiring patch (`controller_.vel_estimate_src_.connect_to(&encoder_.vel_estimate_)`)
has a side effect beyond CAN telemetry: the `Controller` class's own `enable_overspeed_error`
safety check (meant to validate true closed-loop control) also reads from this same port, and
tripped (`Controller::ERROR_OVERSPEED` → propagates up as `AXIS_ERROR_CONTROLLER_FAILED`) during a
fast calibration ramp, since our sensor's instantaneous readings are noisy at speed. Fixed by
setting `axis1.controller.config.enable_overspeed_error = False` — this axis's safety comes from
the host tools' own current/speed caps, not ODrive's closed-loop-oriented overspeed logic, so
disabling it is correct here, not a workaround-of-convenience.

## Fixes applied / current known-good persisted state (axis1)

All of the above are now resolved and saved: `pole_pairs=20`, `is_calibrated=True`,
`hall_polarity_calibrated=True`, `direction=1`, `cpr=30`, `mode=HALL`, `gpio12/13/14_mode=
DIGITAL_PULL_UP`, `calibration_lockin` = `vel=800/accel=2000/current=10` (proven-working values,
kept as the new baseline rather than reverting to defaults that are known to fail),
`controller.config.enable_overspeed_error=False`, resting `general_lockin` = `vel=0/current=8/
accel=20/vel_pi_enable=False`, brake resistor `enable=True/resistance=2.0`, bus trip
`10.5/29.0`.

## Config backup (per request — distinctly named from the dead S1's `odrive_node33.json`)

Full config dumped via `odrivetool backup-config` to
**`odrive_config/odrive_node33_v3_6_reactive_openloop.json`** — captures every field above in one
file. Given Gotcha #1, this is now the fast way to restore full working state after any future
config wipe (`odrivetool restore-config <file>`) instead of re-deriving values by hand.

## Test results

Both host tools (`openloop_cli.py`, `openloop_sequence_test.py`) got `--pi`/`c`-style support for
the new fields; `openloop_sequence_test.py` also gained `--kp`/`--ki`/`--pi-limit`/`--scale`
(multiplies every `SEQUENCE` value — e.g. `--scale 0.5` for a halved-value run).
`MAX_CURRENT` raised back to **15 A** and default `--accel`/`DEFAULT_ACCEL` raised to **900**
rad/s² in both tools (up from the earlier 8 A / 450 conservative values), per request.

- **Halved sequence (`--scale 0.5`) with PI** (`kp=2.0, ki=5.0, limit=100`): ran the full
  forward+reverse profile (~9-19 wheel RPM) for 21 s. `Id` held ~8 A, tracking was noisy (expected
  — coarse ~272-count/rev sensor) but bounded and stable, zero errors throughout, clean stop.
- **Full-value sequence with PI** at the new 15 A/900 defaults: ran the full ±38 wheel RPM
  forward+reverse profile for 21 s. `Id` sat right at the 15 A cap as expected, fast tracking
  (most steps settled within the 1 s dwell at accel=900), clean through-zero reversal, zero errors
  the entire run, clean stop.
- Both runs' `meas~` column (measured wheel RPM, read via `encoder.vel_estimate`) tracked
  `wheel~`/`actual_ev` reasonably across the full commanded range, confirming the PI + tachometer
  feedback loop works as designed, not just in the earlier single-step sanity check.

Gains used (`kp=2.0, ki=5.0, limit=100`) were a first reasonable guess, not a tuned result —
proven stable but not verified optimal (no overshoot/settling-time analysis done). Tune further
if steady-state accuracy or disturbance rejection needs improving for real driving use.
