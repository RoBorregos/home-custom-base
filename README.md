# home-custom-base

RoBorregos custom omnidirectional base — firmware and ROS2 interface for a four-wheel mecanum-drive robot.

---

## Current Status

**Platform**: STM32H743 (CM7) · FreeRTOS · four ODrive S1 (CAN) · BNO085 IMU (I2C) · ESP32 BT (USART2) · ROS computer (USART3)

| Feature | Status |
|---|---|
| Mecanum drive (velocity control) | Working |
| ODrive CAN (FDCAN + TJA1051 @ 500 kbps) | Working |
| BNO085 IMU — quaternion output, 50 Hz | Working |
| Dualshock4 via ESP32 Bluetooth | Working |
| Telemetry CSV to ROS computer @ 100 Hz | Working |
| Dead-reckoning odometry @ 100 Hz | Working |
| Web dashboard (`localhost:5000/offline`) | Working |
| ControlTask (heading controller / path planner) | **Pending** — scaffolded, body empty |
| BT watchdog (1 s timeout) | **Pending** — commented out |

---

## Quick Start

1. Connect STM32H7 to the host computer via USB (USART3, 115,200 baud 8N1).
2. Power ODrives at **10.5 to 29 V DC** booting the STM32 (CAN TX timeout otherwise).
3. In the repo, build and source `omnibase_ws`, then run:

```bash
ros2 run odrive_comm odrive_dashboard
```

Dashboard: `http://host_ip:5000`

---

## Known Limitations

- `printf()` is not mutex-protected — add `MutexUART_DataHandle` guards before adding any new debug-heavy task.
- No stack-overflow detection — set `configCHECK_FOR_STACK_OVERFLOW = 2` in `FreeRTOSConfig.h` during development.
- No I2C bus-reset recovery — BNO085 brownout mid-transaction freezes IMU data until reboot.
- BNO055 and MCP2515 source files still compiled (dead code, ~2–4 KB flash wasted).
- `DefaultTask` declared but never created; its body contains a CAN smoke-test with conflicting node IDs.

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
