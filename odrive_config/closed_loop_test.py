#!/usr/bin/env python3
"""
Interactive closed-loop test tool for the AS5600/ESP32-bridged ODrive axis.

Unlike openloop_cli.py / openloop_sequence_test.py (forced-commutation LOCKIN_SPIN,
no encoder feedback), this drives the axis through real AXIS_STATE_CLOSED_LOOP_CONTROL
using the AS5600 (via the ESP32 quadrature bridge, see
firmware/ESP32_AS5600_EncoderBridge/) as a true commutation-grade encoder. See
ODriveV3_6_Test.md for the full story of how this axis got here.

AXIS_NUM = 0 because the motor + AS5600 are currently wired to M0/axis0 for bench
testing, NOT M1/axis1 where the wheel will ultimately live (see ODriveV3_6_Test.md).
Change AXIS_NUM below if you move the wiring back to M1.

Gains default to the ODRIVE STOCK values (vel_gain=0.1667, vel_integrator_gain=0.3333).
The project's tuned gains from main.c's ODrive_Startup() (vel_integrator_gain=5.658,
~17x stock) were EMPIRICALLY CONFIRMED to destabilize this bench setup: every
CURRENT_LIMIT_VIOLATION trip during bring-up traced back to them, on BOTH encoder
variants (quadrature and SPI-abs), while stock gains ran clean. Those gains were
tuned for the wheel mounted in the real robot (different inertia/load) -- use
--tuned-gains to opt in when testing in that configuration.

On startup this tries AXIS_STATE_ENCODER_INDEX_SEARCH first (fast when it works),
but always falls back to a full AXIS_STATE_ENCODER_OFFSET_CALIBRATION if the index
isn't found. In testing here, index search using the ESP32's software-emulated Z
pulse succeeded 1 out of 3 attempts -- not reliable enough to trust alone, so don't
remove the fallback even though encoder.config.pre_calibrated does persist now.

Usage:
    python3 odrive_config/closed_loop_test.py
Menu:
    1) sequence    -- replays openloop_sequence_test.py's wheel-RPM SEQUENCE at
                       1/4 scale (~3-9.5 RPM), through real closed-loop velocity control
    2) manual vel  -- type wheel RPM values, live (closed-loop velocity, ramped)
    3) position    -- type +/- wheel REVOLUTIONS for a relative move
    q) quit (drops to IDLE)
Ctrl-C inside a mode returns to the menu; Ctrl-C/SIGTERM at the menu exits to IDLE.
"""
import sys
import time
import signal
import argparse

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_VEL_RAMP,
    INPUT_MODE_PASSTHROUGH,
    INPUT_MODE_TRAP_TRAJ,
)

AXIS_NUM = 0  # M0 for now -- see module docstring

# --- mechanics (matches openloop_sequence_test.py / openloop_cli.py) ---
REDUCTION = 9.0
MAX_WHEEL_RPM = 40.0

# --- gains: ODrive stock defaults (proven stable on this bench setup) ---
VEL_GAIN = 0.1666666716337204
VEL_INTEGRATOR_GAIN = 0.3333333432674408
# --- tuned gains from main.c ODrive_Startup(), opt-in via --tuned-gains ---
# (destabilize the bench setup -- see module docstring. For robot-mounted use.)
TUNED_VEL_GAIN = 0.3333
TUNED_VEL_INTEGRATOR_GAIN = 5.658
POS_GAIN = 20.0
VEL_LIMIT = 25.0          # turns/s, controller safety clamp -- not a target speed
VEL_RAMP_RATE = 40.0      # turns/s^2, used with INPUT_MODE_VEL_RAMP

# --- trapezoidal trajectory limits for position mode (INPUT_MODE_TRAP_TRAJ) ---
# Position moves MUST use trap-traj, not passthrough: a passthrough position step
# of N turns demands pos_gain*N turns/s instantly (saturating vel_limit), then
# brakes at max current at the target -- empirically tripped
# CURRENT_LIMIT_VIOLATION at -18/-21 A on a mere 0.5-wheel-rev move.
TRAP_VEL = 2.0            # turns/s cruise (~13 wheel RPM)
TRAP_ACCEL = 4.0          # turns/s^2
TRAP_DECEL = 4.0          # turns/s^2

# same profile shape as openloop_sequence_test.py (forward sweep then the same
# sweep negated, reversing smoothly through zero at loop boundaries), at 1/4 the
# open-loop test's velocities: bench-safe range ~3-9.5 wheel RPM.
_BASE = [18, 20, 22, 18, 16, 12, 16, 22, 26, 30, 34, 38, 35, 33, 28, 25, 22]
SEQUENCE = [v / 4.0 for v in _BASE] + [-v / 4.0 for v in _BASE]


def wheel_rpm_to_turns_s(rpm):
    return rpm * REDUCTION / 60.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_gains(ax, tuned=False):
    ax.controller.config.pos_gain = POS_GAIN
    ax.controller.config.vel_gain = TUNED_VEL_GAIN if tuned else VEL_GAIN
    ax.controller.config.vel_integrator_gain = TUNED_VEL_INTEGRATOR_GAIN if tuned else VEL_INTEGRATOR_GAIN
    ax.controller.config.vel_limit = VEL_LIMIT
    ax.controller.config.vel_ramp_rate = VEL_RAMP_RATE
    ax.controller.config.enable_vel_limit = True


def wait_for_idle(ax, timeout):
    t0 = time.time()
    while ax.current_state != AXIS_STATE_IDLE and time.time() - t0 < timeout:
        time.sleep(0.1)


def ensure_calibrated_and_ready(odrv, ax):
    if not ax.motor.is_calibrated:
        print("motor not calibrated -- running AXIS_STATE_MOTOR_CALIBRATION...")
        odrv.clear_errors()
        ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        wait_for_idle(ax, 15)
        if ax.motor.error != 0:
            print(f"  motor calibration FAILED, motor.error={ax.motor.error}")
            return False

    if ax.encoder.config.use_index:
        print("trying index search (fast path)...")
        odrv.clear_errors()
        ax.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        wait_for_idle(ax, 20)
        print(f"  index_found={ax.encoder.index_found}")

    if not ax.encoder.is_ready:
        print("running full encoder offset calibration (will spin the shaft)...")
        odrv.clear_errors()
        ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        wait_for_idle(ax, 20)
        # is_ready reads stale (False) for a moment right after the state
        # transition -- observed repeatedly on this board. Poll instead of
        # trusting a single read.
        t0 = time.time()
        while not ax.encoder.is_ready and time.time() - t0 < 3.0:
            time.sleep(0.3)

    ok = ax.encoder.is_ready and ax.error == 0 and ax.motor.error == 0 and ax.encoder.error == 0
    print(f"ready={ax.encoder.is_ready} axis.err={ax.error} motor.err={ax.motor.error} enc.err={ax.encoder.error}")
    return ok


def status_line(ax):
    cc = ax.motor.current_control
    return (f"vel_est={ax.encoder.vel_estimate:+6.2f}turn/s "
            f"wheel~{ax.encoder.vel_estimate * 60 / REDUCTION:+6.1f}RPM "
            f"pos={ax.encoder.pos_estimate:+8.2f}turn Iq={cc.Iq_measured:+5.2f}A "
            f"err={ax.error} merr={ax.motor.error}")


def enter_closed_loop_velocity(odrv, ax, ramped=True):
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_VEL_RAMP if ramped else INPUT_MODE_PASSTHROUGH
    ax.controller.input_vel = 0.0
    odrv.clear_errors()
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    return ax.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and ax.error == 0


def enter_closed_loop_position(odrv, ax):
    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ   # NOT passthrough -- see TRAP_* comment
    ax.trap_traj.config.vel_limit = TRAP_VEL
    ax.trap_traj.config.accel_limit = TRAP_ACCEL
    ax.trap_traj.config.decel_limit = TRAP_DECEL
    ax.controller.input_pos = ax.encoder.pos_estimate
    odrv.clear_errors()
    ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.3)
    return ax.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and ax.error == 0


def mode_sequence(odrv, ax):
    if not enter_closed_loop_velocity(odrv, ax, ramped=True):
        print(f"failed to enter closed loop: axis.err={ax.error} motor.err={ax.motor.error}")
        return
    dwell = 1.5
    print(f"\nSEQUENCE mode (closed-loop, ramped @ {VEL_RAMP_RATE} turn/s^2). Ctrl-C to return to menu.\n")
    try:
        while True:
            for target in SEQUENCE:
                tgt = clamp(target, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
                ax.controller.input_vel = wheel_rpm_to_turns_s(tgt)
                t_cmd = time.time()
                while time.time() - t_cmd < dwell:
                    print(f"cmd={tgt:+6.1f}RPM  " + status_line(ax))
                    if ax.error != 0 or ax.motor.error != 0:
                        raise RuntimeError(f"axis error {ax.error} motor error {ax.motor.error}")
                    time.sleep(0.3)
    except RuntimeError as e:
        print(f"\nABORTED: {e}")
    except KeyboardInterrupt:
        print("\n^C -- back to menu")
    finally:
        ax.controller.input_vel = 0.0
        time.sleep(0.3)
        ax.requested_state = AXIS_STATE_IDLE


def mode_manual_velocity(odrv, ax):
    if not enter_closed_loop_velocity(odrv, ax, ramped=True):
        print(f"failed to enter closed loop: axis.err={ax.error} motor.err={ax.motor.error}")
        return
    print(f"\nMANUAL VELOCITY mode (closed-loop, ramped @ {VEL_RAMP_RATE} turn/s^2). "
          f"Type wheel RPM (+/-{MAX_WHEEL_RPM} max), 's'/0 to stop, 'm' to return to menu.\n")
    try:
        for raw in sys.stdin:
            cmd = raw.strip().lower()
            if cmd == "":
                continue
            if cmd == "m":
                break
            if cmd in ("s", "stop"):
                ax.controller.input_vel = 0.0
                print("-> 0 RPM (holding)")
                continue
            try:
                rpm = float(cmd)
            except ValueError:
                print("?? number (wheel RPM), 's'/0 stop, 'm' menu")
                continue
            applied = clamp(rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
            ax.controller.input_vel = wheel_rpm_to_turns_s(applied)
            if applied != rpm:
                print(f"(clamped {rpm} -> {applied})")
            time.sleep(0.2)
            print("   " + status_line(ax))
            if ax.error != 0 or ax.motor.error != 0:
                print(f"AXIS ERROR {ax.error} / MOTOR ERROR {ax.motor.error} -- returning to menu")
                break
    except KeyboardInterrupt:
        print("\n^C -- back to menu")
    finally:
        ax.controller.input_vel = 0.0
        time.sleep(0.3)
        ax.requested_state = AXIS_STATE_IDLE


def mode_position(odrv, ax):
    if not enter_closed_loop_position(odrv, ax):
        print(f"failed to enter closed loop: axis.err={ax.error} motor.err={ax.motor.error}")
        return
    target_pos = ax.encoder.pos_estimate
    print("\nPOSITION mode (closed-loop). Type +/- wheel REVOLUTIONS for a relative move "
          "(e.g. 2 or -0.5), 'm' to return to menu.\n")
    try:
        for raw in sys.stdin:
            cmd = raw.strip().lower()
            if cmd == "":
                continue
            if cmd == "m":
                break
            try:
                wheel_revs = float(cmd)
            except ValueError:
                print("?? number (wheel revolutions, +/-), 'm' menu")
                continue
            target_pos += wheel_revs * REDUCTION
            ax.controller.input_pos = target_pos
            print(f"-> move {wheel_revs:+.2f} wheel rev (target={target_pos:+.2f} motor turns)")
            # generous timeout: trajectory time at TRAP_VEL cruise + margin
            move_timeout = 3.0 + abs(wheel_revs) * REDUCTION / TRAP_VEL
            t0 = time.time()
            while time.time() - t0 < move_timeout:
                print("   " + status_line(ax))
                if ax.error != 0 or ax.motor.error != 0:
                    break
                if abs(ax.encoder.pos_estimate - target_pos) < 0.02 and abs(ax.encoder.vel_estimate) < 0.02:
                    break
                time.sleep(0.2)
            if ax.error != 0 or ax.motor.error != 0:
                print(f"AXIS ERROR {ax.error} / MOTOR ERROR {ax.motor.error} -- returning to menu")
                break
    except KeyboardInterrupt:
        print("\n^C -- back to menu")
    finally:
        ax.requested_state = AXIS_STATE_IDLE


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tuned-gains", action="store_true",
                   help="use main.c's tuned gains (vel_integrator_gain=5.658) instead of ODrive "
                        "stock defaults. WARNING: destabilizes the bench setup (current-limit "
                        "trips) -- only for the wheel mounted in the real robot.")
    args = p.parse_args()

    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    print("Connecting to ODrive ...")
    odrv = odrive.find_any(timeout=20)
    ax = getattr(odrv, f"axis{AXIS_NUM}")
    print(f"Connected. fw {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}, "
          f"vbus {odrv.vbus_voltage:.1f} V, axis{AXIS_NUM}.")

    apply_gains(ax, tuned=args.tuned_gains)
    print(f"gains: {'TUNED (main.c)' if args.tuned_gains else 'ODrive stock defaults'}")

    if not ensure_calibrated_and_ready(odrv, ax):
        print("NOT READY -- aborting.")
        ax.requested_state = AXIS_STATE_IDLE
        return

    try:
        while True:
            print("\n1) sequence test   2) manual velocity   3) position (revolutions)   q) quit")
            try:
                choice = input("> ").strip().lower()
            except EOFError:
                break
            if choice == "1":
                mode_sequence(odrv, ax)
            elif choice == "2":
                mode_manual_velocity(odrv, ax)
            elif choice == "3":
                mode_position(odrv, ax)
            elif choice in ("q", "quit"):
                break
            else:
                print("choose 1, 2, 3 or q")
    except KeyboardInterrupt:
        print("\n^C")
    finally:
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.1)
        print(f"Motor -> IDLE. state={ax.current_state} err={ax.error}")


if __name__ == "__main__":
    main()
