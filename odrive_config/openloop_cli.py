#!/usr/bin/env python3
"""
Reactive open-loop velocity interface for ODrive v3.6 / PATCHED fw 0.5.6.

Motor on M1 => axis1. Pure forced commutation (LOCKIN_SPIN), no encoder feedback.

Requires the PATCHED firmware (run_lockin_spin re-reads general_lockin.vel/current/
accel every loop). With it, we enter LOCKIN_SPIN ONCE and then just write
general_lockin.vel live -- phase_vel ramps smoothly from the current speed to the
new target at general_lockin.accel. No IDLE, no re-enter, no restart-from-0, no jerk.
Reversal (sign flip) ramps smoothly through zero. (On STOCK 0.5.6 live writes are
ignored; this tool would then appear frozen at the entry speed -- use patched fw.)

Usage (run in your OWN terminal so stdin is interactive):
    python3 odrive_config/openloop_cli.py
Then type, one per line:
    <number>    target WHEEL speed in RPM (negative = reverse), e.g.  12  or  -8
    0 / s       ramp to standstill and hold (stays armed)
    c <amps>    change lockin current / torque knob live, e.g.  c 7
    a <rate>    change accel (rad/s^2 elec) live, e.g.  a 300
    coast       disarm and freewheel (AXIS_STATE_IDLE)
    ?           print status
    q / quit    stop and exit

Safety: wheels OFF the ground. NO feedback -- a stalled/jammed wheel is not
detected. Ctrl-C or 'q' always drops the motor to IDLE.
"""
import sys
import time
import math

import odrive
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_LOCKIN_SPIN

# --- mechanics ---
POLE_PAIRS = 20
REDUCTION = 9.0
# electrical rad/s per 1 wheel-RPM:  (pole_pairs*reduction) * (2*pi/60)
ELEC_PER_WHEEL_RPM = (POLE_PAIRS * REDUCTION) * (2.0 * math.pi / 60.0)   # ~18.85

# --- limits / defaults ---
MAX_WHEEL_RPM = 40.0        # refuse commands beyond this (open-loop can lose sync).
                            # 40 RPM -> ~37.7 rad/s mech, back-EMF ~3.5V (vs 26V bus) and
                            # ~120Hz elec (< 1000 rad/s current-loop BW) -- both fine; the
                            # limiting factor at higher speed is sync loss, not voltage.
MAX_CURRENT = 15.0          # HARD CAP on lockin current [A]. Open-loop (and open-loop+PI) forces
                            # this continuously regardless of load -- pure winding heat at
                            # ~20-30W continuous @15A. Raised back up from 8A per request.
DEFAULT_CURRENT = 15.0      # A, lockin torque knob (clamped to MAX_CURRENT)
DEFAULT_ACCEL = 900.0       # rad/s^2 electrical (snappier tracking; adjustable live via 'a <rate>')


def wheel_rpm_to_vel(rpm):
    return rpm * ELEC_PER_WHEEL_RPM


def vel_to_wheel_rpm(vel):
    return vel / ELEC_PER_WHEEL_RPM


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def preflight(odrv, ax):
    problems = []
    if not ax.motor.is_calibrated:
        problems.append("motor.is_calibrated is False (need pre_calibrated=True + valid R/L, then save)")
    if ax.encoder.config.direction == 0:
        problems.append("encoder.config.direction == 0 (set it to 1 -- gates LOCKIN_SPIN)")
    if odrv.config.enable_brake_resistor and not odrv.brake_resistor_armed:
        problems.append("enable_brake_resistor=True but resistor not armed (set False if none connected)")
    return problems


def status_line(odrv, ax):
    cc = ax.motor.current_control
    return (f"state={ax.current_state} armed={ax.motor.is_armed} "
            f"wheel~{vel_to_wheel_rpm(cc.phase_vel):+5.1f}RPM "
            f"(phase_vel={cc.phase_vel:+7.1f} elec) "
            f"Id={cc.Id_measured:+.2f}A vbus={odrv.vbus_voltage:.1f}V "
            f"aerr={ax.error} merr={ax.motor.error}")


def enter_spin(odrv, ax, current, accel):
    """Enter LOCKIN_SPIN once at vel=0 (holds), then we stream vel live."""
    gl = ax.config.general_lockin
    ax.requested_state = AXIS_STATE_IDLE
    time.sleep(0.05)
    gl.accel = accel
    gl.current = current
    gl.vel = 0.0
    gl.finish_on_vel = False
    gl.finish_on_distance = False
    gl.finish_on_enc_idx = False
    odrv.clear_errors()
    ax.requested_state = AXIS_STATE_LOCKIN_SPIN
    time.sleep(0.2)


def main():
    print("Connecting to ODrive ...")
    odrv = odrive.find_any(timeout=20)
    ax = odrv.axis1
    print(f"Connected. fw {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}, "
          f"vbus {odrv.vbus_voltage:.1f} V, axis1.")

    problems = preflight(odrv, ax)
    if problems:
        print("\nNOT READY to spin -- fix these first:")
        for p in problems:
            print("  - " + p)
        return

    current = DEFAULT_CURRENT
    accel = DEFAULT_ACCEL
    gl = ax.config.general_lockin

    enter_spin(odrv, ax, current, accel)
    if ax.current_state != 9:
        print(f"Failed to enter LOCKIN_SPIN (state={ax.current_state}, err={ax.error}). Aborting.")
        ax.requested_state = AXIS_STATE_IDLE
        return

    print(f"\nLIVE. current={current} A, accel={accel} rad/s^2, max +/-{MAX_WHEEL_RPM} wheel RPM.")
    print("Commands: <rpm> | 0/s | c <amps> | a <rate> | coast | ? | q\n")
    print("   " + status_line(odrv, ax))

    try:
        for raw in sys.stdin:
            cmd = raw.strip().lower()
            if cmd == "":
                continue
            if cmd in ("q", "quit", "exit"):
                break
            if cmd == "coast":
                ax.requested_state = AXIS_STATE_IDLE
                time.sleep(0.1)
                print("COAST (disarmed) -> " + status_line(odrv, ax))
                continue
            if cmd in ("s", "stop"):
                gl.vel = 0.0
                print("-> ramping to 0 (holding)")
                time.sleep(0.3)
                print("   " + status_line(odrv, ax))
                continue
            if cmd == "?":
                print("   " + status_line(odrv, ax))
                continue
            if cmd.startswith("c"):
                try:
                    req = float(cmd[1:].strip())
                    current = clamp(req, 0.0, MAX_CURRENT)
                    gl.current = current
                    note = f" (capped from {req} to {MAX_CURRENT})" if current != req else ""
                    print(f"lockin current -> {current} A (live){note}")
                except ValueError:
                    print("usage: c <amps>")
                continue
            if cmd.startswith("a"):
                try:
                    accel = float(cmd[1:].strip())
                    gl.accel = accel
                    print(f"accel -> {accel} rad/s^2 (live)")
                except ValueError:
                    print("usage: a <rate>")
                continue
            try:
                rpm = float(cmd)
            except ValueError:
                print("?? number (wheel RPM), 's'/0 stop, 'c <amps>', 'a <rate>', 'coast', 'q'")
                continue
            # if we had coasted (IDLE), re-enter the spin transparently
            if ax.current_state != 9:
                enter_spin(odrv, ax, current, accel)
            applied = clamp(rpm, -MAX_WHEEL_RPM, MAX_WHEEL_RPM)
            gl.vel = wheel_rpm_to_vel(applied)   # <-- LIVE, no re-enter (patched fw)
            if applied != rpm:
                print(f"(clamped {rpm} -> {applied})")
            print(f"-> {applied:+.1f} wheel RPM (vel={wheel_rpm_to_vel(applied):+.0f} elec)")
            time.sleep(0.3)
            print("   " + status_line(odrv, ax))
    except KeyboardInterrupt:
        print("\n^C")
    finally:
        ax.requested_state = AXIS_STATE_IDLE
        time.sleep(0.1)
        print("Motor -> IDLE (coasting). " + status_line(odrv, ax))


if __name__ == "__main__":
    main()
