# Patched ODrive v3.6 firmware — reactive open-loop velocity + CAN + PI

Custom build of ODrive firmware **fw-v0.5.6** for the **v3.6-56V** board (node 33 in the omnibase
system — this ODrive has no commutation-grade encoder, only an external low-resolution Hall
tachometer, so it runs forced-commutation `LOCKIN_SPIN` instead of true `CLOSED_LOOP_CONTROL`).
Full write-up, including WHY each patch exists and several hard-won gotchas, lives in
`../../ODriveV3_6_Test.md` (sections "CUSTOM FIRMWARE", "CAN INTEGRATION", and "§11. VELOCITY PI").

## Source / license
Base firmware is upstream **odriverobotics/ODrive**, open source, MIT-licensed:
https://github.com/odriverobotics/ODrive (tag `fw-v0.5.6`). Obtained via:
```
git clone --branch fw-v0.5.6 --depth 1 --recurse-submodules https://github.com/odriverobotics/ODrive
```
The four `*.patched` files and `reactive_openloop_full.patch`/`reactive_openloop_lockin.patch`
in this folder are custom modifications on top of that unmodified upstream checkout — not an
upstream release. See "What this firmware does differently from stock 0.5.6" below for exactly
what was changed and why.

## What this firmware does differently from stock 0.5.6

1. **Reactive open-loop velocity.** Stock reads `general_lockin.vel` only once at `LOCKIN_SPIN`
   entry; live writes are ignored, so every speed change needs IDLE→re-enter (a jerk, restart from
   0). Patched to re-read `general_lockin.vel/current/accel` every control iteration — write it
   any time while spinning and it ramps smoothly from the current speed, including sign flips
   (reversal through zero), no re-entry.
2. **CAN `SET_INPUT_VEL` → drives this axis.** Stock only writes `controller_.input_vel_`, which
   nothing reads while in `LOCKIN_SPIN`. Patched `can_simple.cpp` to redirect it into
   `general_lockin.vel` (unit conversion: turns/s → electrical rad/s via `×2π×pole_pairs`) so this
   axis responds to the exact same CAN message the other (closed-loop) omnibase wheels use.
3. **CAN `ENCODER_ESTIMATES` telemetry works for this axis too.** Stock only wires
   `controller_.pos_estimate_linear_src_`/`vel_estimate_src_` to the encoder inside
   `start_closed_loop_control()`, which `LOCKIN_SPIN` never runs — so telemetry would report a
   constant 0.0 without this. Patched to do the same wiring on `LOCKIN_SPIN` entry (pure data
   routing, doesn't affect control).
4. **Optional velocity PI** (`general_lockin.vel_pi_enable`, `vel_pi_kp`, `vel_pi_ki`,
   `vel_pi_limit`). When `vel_pi_enable=false` (default), behavior is byte-identical to plain
   open-loop. When `true`, the encoder (used purely as a tachometer) feeds a slip-compensation
   loop that trims the commanded field frequency toward the real measured speed — corrects for
   load/friction/slip while commutation itself stays fully open-loop (can't destabilize FOC the
   way feeding this into a true closed loop would).

## Files here
- **`ODriveFirmware-v0.5.6-reactive-can-pi.bin`/`.hex`** — the current flashable image (load
  address `0x08000000` for the `.bin`). **This is the exact image currently on the board.** sha256:
  `c9a578bfd1de505edcf2030e01ddcc23ba7f0331f5d317cf631f9fea7d7c5a2e`
- `reactive_openloop_full.patch` — combined diff of all changes (`axis.cpp`, `axis.hpp`,
  `can_simple.cpp`, `odrive-interface.yaml`) vs. a clean `fw-v0.5.6` checkout.
- `axis.cpp.patched` / `axis.hpp.patched` / `can_simple.cpp.patched` / `odrive-interface.yaml.patched`
  — full patched source files, for reference.
- `tup.config` — the build config used (`CONFIG_BOARD_VERSION=v3.6-56V`).
- `ODriveFirmware-v0.5.6-reactive.bin`/`.hex` + `reactive_openloop_lockin.patch` — the **earlier**,
  superseded image (reactive open-loop only, no CAN/PI). Kept for reference/rollback; not what's on
  the board now.

> NOTE: this firmware still reports version **0.5.6** over USB — indistinguishable from stock by
> version alone, but behaves differently. On stock 0.5.6 the reactive CLI would appear "frozen,"
> and this axis would not respond to `SET_INPUT_VEL` over CAN at all.

## ⚠ Before reflashing: config gets wiped by ANY struct-layout change

Adding fields to `LockinConfig_t` (for the PI feature) **invalidated the entire saved config
blob** on next boot — not just the changed struct. `pole_pairs`, calibration state, encoder mode,
even unrelated GPIO pin config all reverted. **After flashing any future rebuild that changes a
struct's layout, assume full reconfiguration is needed.** The fastest path back:
```
odrivetool restore-config ../../odrive_config/odrive_node33_v3_6_reactive_openloop.json
```
(backs up via `odrivetool backup-config <file>` — do this again after any future config change).
See `ODriveV3_6_Test.md` §11 "Gotcha #1" for the full story, and Gotchas #2-4 for other
non-obvious issues hit while bringing this board's config back after that wipe (mode-dependent
hardware needing a reboot to take effect, Hall calibration needing much higher speed than
defaults for this sensor, and the CAN-telemetry patch also feeding the Controller's own overspeed
monitor).

## Reflash (fastest — use the prebuilt .bin)
1. Put board in DFU: with it connected and running, `python3 -c "import odrive; odrive.find_any().enter_dfu_mode()"`
   (or set the physical DFU DIP switch to DFU and power-cycle). Confirm: `dfu-util -l` shows `0483:df11`.
2. Flash:
   ```
   dfu-util -a 0 -s 0x08000000:leave -D ODriveFirmware-v0.5.6-reactive-can-pi.bin
   ```
3. Set the DFU switch back to RUN and power-cycle.
4. **Re-check config immediately** (see warning above) — restore from the JSON backup if needed.

## Rebuild from source (if you need to change a patch)
Requires: `gcc-arm-none-eabi` (tested 10.3.1), `tup`, `dfu-util`, and `pip install --user PyYAML jinja2 jsonschema`.
```
git clone --branch fw-v0.5.6 --depth 1 --recurse-submodules https://github.com/odriverobotics/ODrive
cd ODrive/Firmware
git apply /path/to/reactive_openloop_full.patch      # or copy the four *.patched files over
cp /path/to/tup.config ./tup.config
tup init && make                                        # -> build/ODriveFirmware.elf
arm-none-eabi-objcopy -O binary build/ODriveFirmware.elf build/ODriveFirmware.bin
```
(The `create_can_dbc.py` step at the end of `make` fails without the `cantools` pip module — that
is a post-build CAN-dbc generator, unrelated to the firmware image; ignore it.)

## Test it (reactive open-loop CLI)
`../openloop_cli.py` streams `general_lockin.vel` live. Wheels OFF the ground; motor on M1 (axis1).
```
python3 ../openloop_cli.py
```
Then type wheel-RPM targets and watch smooth, live response (no jerk between commands):
```
10        # ramp to +10 wheel RPM
25        # smoothly ramp up to +25 (from current speed, not from 0)
-8        # smoothly reverse through zero to -8
c 7       # change torque/current live (A)
a 300     # change accel live (rad/s^2, snappier)
0         # ramp to standstill and hold (still armed)
coast     # disarm / freewheel
q         # stop and exit
```
CLI defaults: `current=15 A`, `accel=900 rad/s^2`, `MAX_WHEEL_RPM=40`, `MAX_CURRENT=15` (hard cap).

## Automated reactivity/stress test + PI test (`../openloop_sequence_test.py`)
Streams a fixed sequence of wheel-RPM setpoints, holding each for `--dwell` seconds, looping
perpetually. Wheels OFF the ground; motor on M1 (axis1).
```
python3 ../openloop_sequence_test.py                                   # pure open-loop, defaults
python3 ../openloop_sequence_test.py --pi --kp 2.0 --ki 5.0            # with velocity PI enabled
python3 ../openloop_sequence_test.py --scale 0.5                       # halve every sequence value
python3 ../openloop_sequence_test.py -d 0.5 --accel 600                # faster command cadence
```
Params: `--dwell`/`-d` (s between commands), `--accel` (rad/s² elec ramp), `--current` (A, capped
at `MAX_CURRENT=15`), `--sample` (print interval), `--scale` (multiplies every `SEQUENCE` value),
`--pi` (enable the firmware velocity PI — off by default = pure open-loop), `--kp`/`--ki`
(PI gains), `--pi-limit` (correction clamp, default 200 rad/s elec). Telemetry columns: `cmd`
(commanded wheel RPM), `target_ev`/`actual_ev` (commanded vs. live electrical velocity), `wheel~`
(actual wheel RPM from `phase_vel`), `meas~` (measured wheel RPM from the encoder/tachometer —
what the PI actually sees), `Id`, `vbus`, `st` (9=spinning), `err`. **Ctrl-C or `kill` (SIGTERM)**
both stop it and drop the motor to IDLE cleanly. The looped sequence is defined by `SEQUENCE` near
the top of the script.

Verified (see `ODriveV3_6_Test.md` §11 for full results): both halved-value and full-value
(±38 wheel RPM) sequences ran cleanly with `--pi --kp 2.0 --ki 5.0 --pi-limit 100` — stable,
bounded tracking (noisy given the ~272-count/wheel-rev sensor resolution), zero errors, clean
stops. Gains are a first reasonable guess, not tuned for optimal response.

Higher `--accel` (with enough `--current` for torque headroom) makes it hit each setpoint within
the dwell instead of chasing; too high and the open-loop rotor can lose sync (stutters/stalls) —
open-loop commutation itself still has no feedback to catch that, PI or not. 15 A is real winding
heat; watch motor temp on long runs.
