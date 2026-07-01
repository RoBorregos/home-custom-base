# home-custom-base

RoBorregos custom omnidirectional base — firmware and ROS2 interface for a four-wheel mecanum-drive robot.

---

## Current Status

**Platform**: STM32H743 (CM7) · FreeRTOS · four ODrive S1 (CAN, fw v0.6.12) · BNO085 IMU (I2C) · ESP32 BT (USART2) · ROS computer (USART3 @ 230,400 baud)

| Feature | Status |
|---|---|
| Mecanum drive (velocity control) | Working |
| ODrive CAN (FDCAN + TJA1051 @ 500 kbps) | Working |
| BNO085 IMU — 6-axis quaternion (GAME_ROTATION_VECTOR), 50 Hz | Working |
| PS5 / Dualshock4 via ESP32 Bluetooth | Working |
| Telemetry CSV to ROS computer @ ~50/10 Hz (slim/fat lines) | Working |
| On-MCU 6-state EKF (wheel odom + IMU fusion), replaces robot_localization for omnibase | Working* |
| `odom -> base_link` TF published directly by the dashboard | Working |
| Firmware error reporting (queued, dashboard "Firmware errors" panel) | Working |
| Web dashboard (`localhost:5000`) with link status monitoring | Working |
| Arbitrary ODrive parameter writes over CAN (RxSdo) | Working |
| FDCAN bus-off auto-recovery | Working |
| UART overrun self-healing | Working |
| Velocity-command watchdog (500 ms, any source) | Working |
| BT pairing persistence across reboots | Working |
| ESTOP hardware input with separate press/release debounce + re-arm hold | Working |
| Stack overflow detection (`configCHECK_FOR_STACK_OVERFLOW=2`) + periodic watermark check | Working |
| ControlTask (heading controller / path planner) | **Pending** — scaffolded, body empty |
| home2 `omnidriver` dashboard EKF compatibility | **Pending**\*\* |
| IMU staleness watchdog | **Pending** — silent fallback to dead-reckoning if BNO085 stops producing samples |

\* see `firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md` §11

\*\* see `firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md` §12

---

## Quick Start

1. Connect STM32H7 to the host computer via USB (USART3, **230,400 baud** 8N1).
2. Power ODrives at **10.5 to 29 V DC** before booting the STM32 (CAN TX timeout otherwise).
3. Build and source `omnibase_ws`, then run:

```bash
cd omnibase_ws
colcon build --packages-select odrive_comm
source install/setup.bash
ros2 run odrive_comm odrive_dashboard
```

Dashboard: `http://host_ip:5000`

The dashboard shows live odometry, IMU, ODrive axis states, robot velocity, and a **Links** panel with ESP32→STM32 and STM32→PC connection health (OK / WARN / LOST).

---

## ODrive Node IDs

| Axis | CAN Node ID |
|---|---|
| Front-Left  | 36 |
| Front-Right | 34 |
| Rear-Left   | 33 |
| Rear-Right  | 40 |

Node IDs are pre-assigned in firmware at boot before FDCAN RX is activated. Config dumps are in `odrive_config/`, which also has `flat_endpoints.json` for ODrive firmware v0.6.12 — use it to look up the `endpoint_id` for any `axis0.controller.config.*` (or other) parameter you want to set via the dashboard's "Advanced param (RxSdo)" box (see `omnibase_documentation.md` §10.2).

---

## Safety

- **Velocity watchdog**: if no `SET_VEL` command is received from any source (ROS or BT) for 500 ms, all four motors are zeroed automatically. Re-arms on next command.
- **FDCAN bus-off recovery**: if the CAN bus enters bus-off state, the firmware restarts the peripheral automatically without rebooting.
- **UART self-healing**: overrun errors on USART2 (ESP32) and USART3 (ROS PC) are cleared and RX is re-armed automatically.
- **Hardware ESTOP**: normally-closed button on PE2. Press/release uses a ~90 ms sample-after-delay debounce; release additionally requires a separate `ESTOP_RELEASE_HOLD_MS` (150 ms) of continuous release before re-arming, so a noisy release edge can't immediately re-engage the motors.
- **Stack overflow protection**: `configCHECK_FOR_STACK_OVERFLOW=2` (pattern-checking) plus a periodic `uxTaskGetStackHighWaterMark()` sweep across all tasks, both reported through the firmware error queue.

---

## Known Limitations

- `printf()` is not mutex-protected, but this is now enforced by convention instead: **only `Start_UART_TX_Task` calls `printf()`** — every other task routes diagnostics through `FirmwareError_Push()` / the error queue instead, so concurrent-printf corruption can't occur as long as that rule holds.
- No I2C bus-reset recovery — BNO085 brownout mid-transaction freezes IMU data until reboot.
- No IMU staleness watchdog — if the BNO085 stops producing samples, `ekf_correct_imu` silently stops applying corrections forever; the EKF falls back to wheel-only dead-reckoning with no error pushed and no dashboard indicator (unlike the ESP32/PC link rows, which do track age/status explicitly).
- `DefaultTask` is created but gutted to a one-line idle loop (kept only to avoid CubeMX regen conflicts).
- BT-only watchdog (`BT_OVERRIDE_TIMEOUT_MS`) is now redundant — superseded by the general velocity watchdog. Pending removal after testing.
- home2's `omnidriver` dashboard fork is not yet updated to consume the on-MCU EKF firmware correctly (baud rate, yaw inversion, EKF quaternion/covariance, TF) — see `omnibase_documentation.md` §12. Don't flash this EKF firmware against an un-updated `omnidriver` without checking that first.
- Arbitrary CAN parameter writes (RxSdo, used for e.g. `vel_ramp_rate`) are fire-and-forget — no TxSdo read-back is implemented, so a write isn't confirmed to have landed without checking via `odrivetool`/USB.
- The dashboard's **Axis Lab tab** (per-motor inspection/control/calibration) is **untested** — has not been validated against real hardware.

---

## RTOS Task Summary

| Task | Priority | Period |
|---|---|---|
| UART_RX_Task | High | ~200 Hz (5 ms) |
| BT_RX_Task | High | ~200 Hz (5 ms) |
| UART_TX_Task | Normal | blocks on CAN→UTX queue; drains firmware error queue + stack watermark check |
| ODriveTask | AboveNormal | ~1 kHz setpoints, slim telemetry every cycle, fat telemetry every 5th, EKF update each cycle |
| IMU_Task | AboveNormal | ~500 Hz poll (2 ms) |
| ControlTask | AboveNormal | empty — 10 ms delay only |

---

## Firmware Error Reporting

Tasks other than `Start_UART_TX_Task` cannot call `printf()`. Instead they call `FirmwareError_Push(code, axis, detail)`, which queues a `FirmwareError` struct (16-deep). `Start_UART_TX_Task` drains it each cycle and emits `E=<code>,<axis>,<detail>\r\n` per error, or `ELOST=<n>\r\n` if the queue overflowed. `axis` is `0-3` for per-axis errors or `0xFF` for system-level. Error codes are defined in `main.h` (`FERR_*`) and described in `odrive_dashboard.py`'s `FERR_DESCRIPTIONS`; the dashboard shows them in a dedicated "Firmware errors" panel, separate from the general activity log.

---

## EKF and TF

The firmware runs a 6-state EKF (`firmware/.../ekf.c`) fusing wheel odometry with BNO085 IMU yaw/omega_z, replacing the previous Euler dead-reckoning. `odrive_dashboard.py` consumes the EKF's quaternion and full pose/twist covariance and publishes the `odom -> base_link` TF directly (`publish_tf` parameter, default `True`) — this fully replaces `robot_localization` for the omnibase configuration. See `firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md` §11 for the full design write-up, replaceability analysis vs. `robot_localization`, and known gaps (§12 covers the pending home2 migration).

---

## Troubleshooting

See `firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md` §13 for known issues:
- **§13.1** — I2C init failures / system not booting (FreeRTOS stack overflow)
- **§13.2** — ODrive stuck in INITIALIZING and won't clear

## Developers

- Rogelio Ruiz — [joserogelioruiz12@outlook.com](mailto:joserogelioruiz12@outlook.com)
