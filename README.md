# home-custom-base

RoBorregos custom omnidirectional base — firmware and ROS2 interface for a four-wheel mecanum-drive robot.

---

## Current Status

**Platform**: STM32H743 (CM7) · FreeRTOS · four ODrive S1 (CAN) · BNO085 IMU (I2C) · ESP32 BT (USART2) · ROS computer (USART3 @ 921,600 baud)

| Feature | Status |
|---|---|
| Mecanum drive (velocity control) | Working |
| ODrive CAN (FDCAN + TJA1051 @ 500 kbps) | Working |
| BNO085 IMU — 6-axis quaternion (GAME_ROTATION_VECTOR), 50 Hz | Working |
| PS5 / Dualshock4 via ESP32 Bluetooth | Working |
| Telemetry CSV to ROS computer @ 100 Hz | Working |
| Dead-reckoning odometry @ 100 Hz | Working |
| Web dashboard (`localhost:5000`) with link status monitoring | Working |
| FDCAN bus-off auto-recovery | Working |
| UART overrun self-healing | Working |
| Velocity-command watchdog (500 ms, any source) | Working |
| BT pairing persistence across reboots | Working |
| ControlTask (heading controller / path planner) | **Pending** — scaffolded, body empty |

---

## Quick Start

1. Connect STM32H7 to the host computer via USB (USART3, **921,600 baud** 8N1).
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

Node IDs are pre-assigned in firmware at boot before FDCAN RX is activated. Config dumps are in `odrive_config_dump/`.

---

## Safety

- **Velocity watchdog**: if no `SET_VEL` command is received from any source (ROS or BT) for 500 ms, all four motors are zeroed automatically. Re-arms on next command.
- **FDCAN bus-off recovery**: if the CAN bus enters bus-off state, the firmware restarts the peripheral automatically without rebooting.
- **UART self-healing**: overrun errors on USART2 (ESP32) and USART3 (ROS PC) are cleared and RX is re-armed automatically.

---

## Known Limitations

- `printf()` is not mutex-protected — add `MutexUART_DataHandle` guards before adding any new debug-heavy task.
- No stack-overflow detection — set `configCHECK_FOR_STACK_OVERFLOW = 2` in `FreeRTOSConfig.h` during development.
- No I2C bus-reset recovery — BNO085 brownout mid-transaction freezes IMU data until reboot.
- `DefaultTask` declared but never created; its body contains a CAN smoke-test with conflicting node IDs.
- BT-only watchdog (`BT_OVERRIDE_TIMEOUT_MS`) is now redundant — superseded by the general velocity watchdog. Pending removal after testing.

---

## RTOS Task Summary

| Task | Priority | Period |
|---|---|---|
| UART_RX_Task | High | ~200 Hz (5 ms) |
| BT_RX_Task | High | ~200 Hz (5 ms) |
| UART_TX_Task | Normal | blocks on CAN→UTX queue |
| ODriveTask | AboveNormal | ~1 kHz setpoints, 100 Hz telemetry |
| IMU_Task | AboveNormal | ~500 Hz poll (2 ms) |
| ControlTask | AboveNormal | empty — 10 ms delay only |

---

## Developers

- Rogelio Ruiz — [joserogelioruiz12@outlook.com](mailto:joserogelioruiz12@outlook.com)
