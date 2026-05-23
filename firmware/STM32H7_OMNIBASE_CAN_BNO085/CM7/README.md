# STM32H7 OMNIBASE — firmware quickstart

Four-wheel mecanum-drive robot controller running on an
**STM32H755ZI** (Cortex-M7 core, FreeRTOS). The MCU drives four
**ODrives** over **FDCAN1**, fuses encoder odometry with a **BNO085**
IMU through an on-board 6-state Extended Kalman Filter, takes velocity
commands from a ROS computer on **USART3** or an ESP32 / Bluetooth
module on **USART2**, and streams CSV telemetry back to the host at
100 Hz.

For the deep dive (PDF gaps, full EKF derivation, queue / RTOS detail)
see [`docs_update.md`](docs_update.md). For the complete technical
reference of every task and subsystem see
[`omnibase_documentation.txt`](omnibase_documentation.txt).

---

## 1. Hardware at a glance

### Pinout

| Pin   | Function                                                      |
| ----- | ------------------------------------------------------------- |
| PB6   | I²C1 SCL — BNO085 (4.7 kΩ pull-up to 3V3)                     |
| PB7   | I²C1 SDA — BNO085 (4.7 kΩ pull-up to 3V3)                     |
| PD14  | BNO085 INT — active-LOW input, polled by `sh2_service()`      |
| PD15  | BNO085 RST — active-LOW output, pulsed for 10 ms at boot      |
| PD5   | USART2 TX — ESP32 / Bluetooth                                 |
| PD6   | USART2 RX — ESP32 / Bluetooth                                 |
| PD8   | USART3 TX — ROS computer                                      |
| PD9   | USART3 RX — ROS computer                                      |
| FDCAN1| ODrive bus, 500 kbps, classic CAN mode (TJA1051 transceiver)  |

BNO085 7-bit I²C address `0x4A` (selected by `PS0 = PS1 = GND`).
Both UARTs run at **115 200 baud, 8N1**.

### ODrive node IDs and wheel signs

| Index | Node ID | Gear ratio | Wheel sign |
| ----- | ------- | ---------- | ---------- |
| 0     | 36      | 9          | −1         |
| 1     | 34      | 9          | +1         |
| 2     | 33      | 9          | −1         |
| 3     | 40      | 9          | +1         |

Robot geometry (live values in
[`main.c`](Core/Src/main.c)):
`x_offset = 0.195 m`, `y_offset = 0.195 m`, `wheel radius = 0.0762 m`.

> Physical FL / BL / FR / BR mapping is still TBD — confirm on the
> bench and annotate `StartODriveTask`.

### Power sequencing

ODrives must be powered (12 V DC) **before** the STM32 boots. If the
STM32 finishes its 3-second boot window with the CAN bus dead, the
`SM_BOOT → SM_STARTUP` transition fails with `CAN TX timeout` messages
on USART3 and the task retries.

---

## 2. Build & flash

### Toolchain

- **STM32CubeIDE 1.13+** (or stand-alone `arm-none-eabi-gcc` driven by
  the Eclipse-managed Makefile under `Debug/`).
- Target: STM32H755ZITx, Cortex-M7 core. The CM4 project
  (`../CM4/`) is also built but is currently empty.

### How to build inside the IDE

1. Open `firmware/STM32H7_OMNIBASE_CAN_BNO085` as an existing Eclipse
   project. Both `CM7` and `CM4` sub-projects appear in the Project
   Explorer.
2. Select the `CM7` project → **Project → Build**.
3. Flash with **Run → Debug** (uses the .launch config in this folder)
   or the ST-Link CLI.

`Core/Src/*.c` and `Core/Inc/*.h` are auto-discovered by the project,
so new source files (e.g. [`ekf.c`](Core/Src/ekf.c) /
[`ekf.h`](Core/Inc/ekf.h)) compile without editing
`.cproject` or `CMakeLists`.

### If include paths break after pulling new headers

Eclipse caches indexer state. If you get `No such file or directory`
for headers that visibly exist on disk:

1. Close and reopen the project (or restart the IDE).
2. If the error persists, right-click the project → **Properties →
   C/C++ Build → Settings → Tool Settings → MCU GCC Compiler →
   Include paths → Add**:

       ${workspace_loc:/${ProjName}/Core/Inc/sh2}

   Repeat for the **MCU GCC Assembler → Include paths**.
3. **Project → Clean → Build All** (full clean, not incremental).

---

## 3. First-flash verification

Open the host-side USART3 terminal at **115 200 baud** before flashing.

| # | Check                | Expected                                                                  |
| - | -------------------- | ------------------------------------------------------------------------- |
| 1 | BNO085 init          | Within ~1 s: `IMU_Task: starting BNO085 init` then `IMU_Task: BNO085 running at 50 Hz`. Failure prints `sh2_open failed rc=X — halting`; check PD14/PD15 wiring and the 4.7 kΩ I²C pull-ups. |
| 2 | IMU values in telemetry | Hold the board flat — `IMU_yaw` drifts slowly, `IMU_roll` / `IMU_pitch` ≈ 0. Rotate it; all three respond. They should not freeze or jump wildly. |
| 3 | Telemetry rate       | Telemetry line emitted every ~10 ms (≈100 Hz). IMU fields are not stuck at 0.0. |
| 4 | Filtered odometry    | `ODOM_phi`, `ODOM_x`, `ODOM_y`, `ODOM_qx..qw` populate. Pushing the robot one wheel-rev forward should advance `ODOM_x` by ~`2π·0.0762 m ≈ 0.48 m`. `ODOM_var_*` values stay finite and shrink as the filter converges. |
| 5 | ODrive control        | With CAN_STUB = 0 and ODrives connected, send a type-1 command (see § 4): wheel velocities track the setpoint, no recurring `CAN TX timeout`. |
| 6 | I²C lockup           | Leave the firmware running >10 minutes; the telemetry line keeps printing with fresh IMU values. A frozen `IMU_yaw` indicates a stuck `HAL_I2C_Master_Receive`; raise `BNO085_I2C_TIMEOUT_MS` or add bus-reset logic in `hal_read()`. |
| 7 | Sensor reset recovery | Power-cycle the BNO085 in isolation. Console prints `IMU_Task: BNO085 reset detected — re-configuring` and IMU data resumes within ~500 ms. |

The EKF makes step 4 the new sanity test for both the wheel encoders
and the IMU: if either input stalls, the corresponding `ODOM_*`
component freezes and `ODOM_var_*` grows.

---

## 4. UART command protocol

USART3 (ROS) and USART2 (ESP32) both at 115 200 baud, 8N1. Lines
terminated with `\r\n`.

| Type | From  | Format                                       | Effect                                            |
| ---- | ----- | -------------------------------------------- | ------------------------------------------------- |
| 1    | ROS   | `1 <vx> <vy> <wz>`                           | Velocity command for the whole robot              |
| 2    | ROS   | `2 <sub> <mask_hex> [params]`                | ODrive configuration sub-command (see below)      |
| 3    | ESP32 | `3 <vx> <vy> <wz> <buttons_hex>`             | BT velocity command; bit `0x04` = e-stop          |

`<vx>`, `<vy>` in m/s; `<wz>` in rad/s. BT packets have a 500 ms
priority window over ROS.

Type-2 sub-commands (one per line — see
[`docs_update.md`](docs_update.md#5-uart-command-protocol-usart3--115-200-baud)
for the full param list):

| `<sub>` | Action                                                       |
| ------- | ------------------------------------------------------------ |
| 20      | clear errors                                                 |
| 21      | set axis state (8 = CLOSED_LOOP_CONTROL)                     |
| 22      | set controller + input mode (2 = VEL, 1 = PASSTHROUGH)       |
| 23      | set velocity + current limits                                |
| 24      | set position gain                                            |
| 25      | set velocity gain + integrator gain                          |
| 26      | full startup sequence                                        |
| 27      | reboot ODrive                                                |
| 28      | set input torque                                             |
| 29      | stop (vel → 0, axes → IDLE)                                  |
| 30      | set input position with vel/torque feedforwards              |

---

## 5. Telemetry output (USART3 TX, 100 Hz)

One comma-separated line every 10 ms. Field groups:

```
CMD_*                  last commanded robot twist
IMU_yaw,roll,pitch     BNO085 Euler in DEGREES (legacy)
IMU_q*, IMU_w*, IMU_a* quaternion / gyro / linear accel (rad, rad/s, m/s²)
IK_u0..u3              IK wheel speeds (rad/s)
ODOM_phi,x,y,z         EKF pose (z always 0)
ODOM_q{x,y,z,w}        pose quaternion (from EKF yaw)
ODOM_w,vx,vy           world-frame velocities (legacy q_dot triplet)
ODOM_vxb,vyb           body-frame linear velocities from the EKF
ODOM_var_x,y,yaw       pose covariance diagonal
ODOM_var_vx,vy,wz      twist covariance diagonal
N0..U3                 per-axis ODrive state (node id → updated flag)
BT_active, BT_vx,vy,wz Bluetooth override state
ESP32_age_ms           ms since last valid BT packet (-1 = never)
```

Full layout, including the per-axis ODrive fields, is in
[`docs_update.md`](docs_update.md#6-telemetry-output-format-usart3-tx--100-hz).

The full ROS-style row-major 6×6 `pose_covariance` and
`twist_covariance` arrays live in the firmware-side `OdomData` struct
but are not emitted over UART — only the diagonal entries are echoed
to keep the line within the 115 200 baud budget.

---

## 6. Odometry — on-board 6-state EKF

The firmware no longer dead-reckons. Every telemetry tick (10 ms)
`ODrive_UpdateTelemetryAndOdometry` runs one full predict + correct
cycle of a six-state Extended Kalman Filter:

```
state =  [ x, y, theta, vx_body, vy_body, omega ]
inputs:
    wheel encoders × 4   →  mecanum FK  →  (vx, vy, omega) measurement
    BNO085 quaternion    →  yaw         measurement (gated, boot-zeroed)
    BNO085 gyro.z        →  yaw rate    measurement
    BNO085 lin-accel     →  optional predict-step control input (off by default)

output  →  OdomData (legacy fields + z, qx/qy/qz/qw, body twist, full
                     6×6 ROS-shaped pose / twist covariance)
```

State transitions are propagated through `ekf_predict` (Jacobian-based,
with optional `0.5·a·dt²` position correction when acceleration is on),
and each measurement is fused via the **Joseph-form** Kalman update so
`P` stays symmetric and positive-semidefinite.

The `EKFParams` struct exposes the noise covariances, outlier gates,
and signs — see
[`Core/Inc/ekf.h`](Core/Inc/ekf.h) and `ekf_params_defaults()` in
[`Core/Src/ekf.c`](Core/Src/ekf.c). Full derivation, matrix shapes and
tuning notes are in
[`docs_update.md` § 7](docs_update.md#7-ekf--6-state-mecanum-odometry-filter-new).

A pose-reset API (`ekf_reset_pose`) is implemented but not yet wired
to any UART command — drop a new type-2 sub-command into
`UART_RX_ParseLine` to expose it.

---

## 7. FreeRTOS tasks

| Task              | Priority           | Stack | Role                                                                |
| ----------------- | ------------------ | ----- | ------------------------------------------------------------------- |
| `UART_RX_Task`    | osPriorityHigh        | 4 KB  | USART3 ring buffer → line parser → command queue                    |
| `BT_RX_Task`      | osPriorityHigh        | 4 KB  | USART2 ring buffer → type-3 parser → command queue                  |
| `UART_TX_Task`    | osPriorityNormal      | 4 KB  | Telemetry CSV printer (blocks on the CAN-2-UTX queue)               |
| `ODriveTask`      | osPriorityAboveNormal | 4 KB  | CAN-control state machine + EKF cycle + telemetry assembly @ 10 ms  |
| `IMU_Task`        | osPriorityAboveNormal | 4 KB  | BNO085 `sh2_service()` loop @ 50 Hz outputs                         |
| `ControlTask`     | osPriorityAboveNormal | 1 KB  | scaffolded, body empty                                              |
| `StartDefaultTask`| osPriorityNormal      | 512 B | declared, never started                                             |

Tick: 1 kHz. Heap: 32 768 B (`heap_4`). HAL timebase: TIM6.

For the per-task narrative (why `IMU_Task` is separate, queue
sizing, etc.) see [`docs_update.md` § 3-4](docs_update.md#3-rtos--full-task-table).

---

## 8. Where things live

```
Core/Src/main.c            All tasks, kinematics, BNO085 callbacks, peripheral init
Core/Src/ODrive.c          Low-level CAN-TX wrappers
Core/Inc/main.h            Project structs (OdomData, ODriveTelemetryMsg, …)
Core/Src/ekf.c             6-state EKF implementation
Core/Inc/ekf.h             EKF API
Core/Src/sh2_hal_impl.c    BNO085 HAL (I²C + INT + RST)
Core/Src/sh2/*             CEVA SH2 protocol library (do not modify)
Core/Src/myprintf.c        printf retarget to USART3 (polling, not thread-safe)
odrive_task_config.c       Design reference only — not in the build
```

Legacy still-compiled drivers (`bno055.c`, `mcp2515.c`) cost ~2-4 KB of
flash and can be dropped from the build at any time — they're never
called.

---

## 9. Further reading

- [`docs_update.md`](docs_update.md) — PDF gaps and corrections, full
  EKF derivation, queue / IPC details, kinematics equations, known
  limitations, and status snapshot.
- [`omnibase_documentation.txt`](omnibase_documentation.txt) —
  exhaustive technical reference (1000+ lines covering every task,
  callback, and data path).
- `Omnibase STM32 Docs.pdf` — original public-facing doc; parts are
  stale (see `docs_update.md` § 1 for the corrections).
