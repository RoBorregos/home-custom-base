#!/usr/bin/env python3
"""
Reactive open-loop STRESS/REACTIVITY test for ODrive v3.6 / PATCHED fw 0.5.6.

Streams a fixed sequence of WHEEL-RPM setpoints to axis1 (motor on M1), holding each
for DWELL seconds, looping perpetually. Writes general_lockin.vel LIVE (no IDLE / no
re-enter) so you can watch how the open-loop rotor tracks a rapidly changing setpoint.

Run in your own terminal; Ctrl-C to stop (drops motor to IDLE cleanly):
    python3 odrive_config/openloop_sequence_test.py
Optional overrides:
    python3 odrive_config/openloop_sequence_test.py --dwell 1.5 --accel 300 --current 6

Requires the PATCHED firmware (see firmware/odrive_3_6_custom_firmware/). Wheels OFF the ground.
NOTE: at accel=150 a big step may not fully settle within 1.5 s -- that is expected and
is exactly what this test shows. Raise --accel (and --current for torque headroom) to
track faster; too high and the open-loop rotor loses sync (stutters/stalls).
"""
import sys
import time
import math
import signal
import argparse

import odrive
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_LOCKIN_SPIN

# --- mechanics ---
POLE_PAIRS = 20
REDUCTION = 9.0
ELEC_PER_WHEEL_RPM = (POLE_PAIRS * REDUCTION) * (2.0 * math.pi / 60.0)   # ~18.85
MAX_WHEEL_RPM = 40.0
MAX_CURRENT = 15.0          # HARD CAP on lockin current [A]. Open-loop (and open-loop+PI) forces
                            # this continuously regardless of load -> pure winding heat, ~20-30W
                            # continuous @15A. Raised back up from 8A per request.

# --- the test sequence (wheel RPM) ---
# Positive profile, then the same profile negated (reverse direction). Looped
# perpetually this runs: forward sweep -> reverse sweep -> forward -> ... , so it
# reverses smoothly through zero at each block boundary (the +22 -> -18 and, on
# loop, -22 -> +18 transitions exercise through-zero reversal).
_BASE = [18, 20, 22, 18, 16, 12, 16, 22, 26, 30, 34, 38, 35, 33, 28, 25, 22]
SEQUENCE = _BASE + [-v for v in _BASE]


def wheel_rpm_to_vel(rpm):
    return rpm * ELEC_PER_WHEEL_RPM


def vel_to_wheel_rpm(vel):
    return vel / ELEC_PER_WHEEL_RPM


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def preflight(odrv, ax):
    problems = []
    if not ax.motor.is_calibrated:
        problems.append("motor.is_calibrated is False")
    if ax.encoder.config.direction == 0:
        problems.append("encoder.config.direction == 0 (set to 1)")
    if odrv.config.enable_brake_resistor and not odrv.brake_resistor_armed:
        problems.append("enable_brake_resistor=True but resistor not armed")
    return problems


def enter_spin(odrv, ax, current, accel, pi_enable=False, kp=0.0, ki=0.0, pi_limit=200.0):
    gl = ax.config.general_lockin
    ax.requested_state = AXIS_STATE_IDLE
    time.sleep(0.05)
    gl.accel = accel
    gl.current = current
    gl.vel = 0.0
    gl.finish_on_vel = False
    gl.finish_on_distance = False
    gl.finish_on_enc_idx = False
    gl.vel_pi_enable = pi_enable
    gl.vel_pi_kp = kp
    gl.vel_pi_ki = ki
    gl.vel_pi_limit = pi_limit
    odrv.clear_errors()
    ax.requested_state = AXIS_STATE_LOCKIN_SPIN
    time.sleep(0.2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dwell", "-d", type=float, default=1.5,
                   help="TIME BETWEEN COMMANDS: seconds to hold each setpoint (default 1.5)")
    p.add_argument("--accel", type=float, default=900.0, help="rad/s^2 electrical ramp rate (default 900)")
    p.add_argument("--current", type=float, default=15.0,
                   help=f"lockin current [A] (default 15, hard-capped at {MAX_CURRENT})")
    p.add_argument("--sample", type=float, default=0.5, help="telemetry print interval [s]")
    p.add_argument("--scale", type=float, default=1.0,
                   help="multiply every value in SEQUENCE by this factor (e.g. 0.5 = half speed)")
    p.add_argument("--pi", action="store_true",
                   help="enable the firmware velocity PI (vel_pi_enable). Off by default = pure open-loop.")
    p.add_argument("--kp", type=float, default=0.0, help="velocity PI proportional gain (only used with --pi)")
    p.add_argument("--ki", type=float, default=0.0, help="velocity PI integral gain (only used with --pi)")
    p.add_argument("--pi-limit", type=float, default=200.0,
                   help="max PI correction magnitude [rad/s elec], anti-windup clamp (default 200)")
    args = p.parse_args()
    if args.current > MAX_CURRENT:
        print(f"(capping current {args.current} -> {MAX_CURRENT} A)")
        args.current = MAX_CURRENT
    args.current = max(0.0, args.current)

    # live output even when piped, and clean shutdown on `kill` (SIGTERM) as well as Ctrl-C
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    print("Connecting to ODrive ...")
    odrv = odrive.find_any(timeout=20)
    ax = odrv.axis1
    print(f"Connected. fw {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}, "
          f"vbus {odrv.vbus_voltage:.1f} V, axis1.")

    problems = preflight(odrv, ax)
    if problems:
        print("NOT READY:")
        for pr in problems:
            print("  - " + pr)
        return

    gl = ax.config.general_lockin
    enter_spin(odrv, ax, args.current, args.accel,
               pi_enable=args.pi, kp=args.kp, ki=args.ki, pi_limit=args.pi_limit)
    if ax.current_state != 9:
        print(f"Failed to enter LOCKIN_SPIN (state={ax.current_state}). Aborting.")
        ax.requested_state = AXIS_STATE_IDLE
        return

    sequence = [v * args.scale for v in SEQUENCE]
    mode_str = f"PI ENABLED (kp={args.kp} ki={args.ki} limit={args.pi_limit})" if args.pi else "pure open-loop (PI disabled)"
    print(f"\nLIVE stress test. dwell={args.dwell}s  accel={args.accel} rad/s^2  "
          f"current={args.current} A  scale={args.scale}  mode={mode_str}")
    print(f"sequence (wheel RPM): {sequence}")
    print("Ctrl-C to stop.\n")
    print(f"{'t':>7} {'cmd':>6} {'target_ev':>9} {'actual_ev':>9} {'wheel~':>7} "
          f"{'meas~':>7} {'Id':>6} {'vbus':>6} {'st':>3} {'err':>4}")

    t_start = time.time()
    try:
        cycle = 0
        while True:
            cycle += 1
            for target in sequence:
                tgt = clamp(target, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
                # robustness: if something disarmed us, transparently re-enter
                if ax.current_state != 9:
                    print("   [re-entering LOCKIN_SPIN after disarm]")
                    enter_spin(odrv, ax, args.current, args.accel,
                               pi_enable=args.pi, kp=args.kp, ki=args.ki, pi_limit=args.pi_limit)
                gl.vel = wheel_rpm_to_vel(tgt)              # <-- LIVE write, no re-enter
                target_ev = wheel_rpm_to_vel(tgt)
                t_cmd = time.time()
                # sample telemetry across the dwell; always honor --dwell exactly, even
                # when dwell < sample (never let the sleep overshoot the command cadence),
                # and always print at least one row per command.
                while True:
                    cc = ax.motor.current_control
                    ev = cc.phase_vel
                    # measured velocity from the encoder (same tachometer the firmware PI
                    # uses internally), converted to electrical rad/s for comparison with ev
                    meas_ev = ax.encoder.vel_estimate * 2.0 * math.pi * POLE_PAIRS
                    print(f"{time.time()-t_start:7.1f} {tgt:6.1f} {target_ev:9.1f} "
                          f"{ev:9.1f} {vel_to_wheel_rpm(ev):7.1f} "
                          f"{vel_to_wheel_rpm(meas_ev):7.1f} "
                          f"{cc.Id_measured:6.2f} {odrv.vbus_voltage:6.2f} "
                          f"{ax.current_state:3d} {ax.error:4d}")
                    remaining = args.dwell - (time.time() - t_cmd)
                    if remaining <= 0:
                        break
                    time.sleep(min(args.sample, remaining))
    except KeyboardInterrupt:
        print("\n^C stopping")
    finally:
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.1)
        print(f"Motor -> IDLE (coasting). state={ax.current_state} err={ax.error}")


if __name__ == "__main__":
    main()
