# STM32H7 OMNIBASE — Complete Technical Documentation

**Project**: STM32H7_OMNIBASE_CAN_BNO085
**Target**: STM32H743 (CM7 core), FreeRTOS + CMSIS-RTOS2
**Date**: 2026-06-20 (converted from `omnibase_documentation.txt`, updated for EKF + firmware error reporting + ESTOP + CAN parameter access)

> This file documents the firmware as of branch `merged_major_update_ekf`. For the EKF specifically, see Section 11 (design write-up + `robot_localization` replaceability analysis) and Section 12 (pending home2 `omnidriver` migration). For a dated changelog, see `CM7/status.txt`.

---

## Section 1 — Overall Project Behavior

### 1.1 What the project does

This firmware controls a four-wheeled mecanum-drive robot. The four wheels are each driven by an independent brushless motor through a 9:1 gearbox. Each motor has its own ODrive S1 motor controller (firmware v0.6.12), connected over CAN (FDCAN1, 500 kbps). The MCU acts as the CAN master: it receives high-level robot velocity commands from two sources (a ROS computer over USART3 and an ESP32 Bluetooth module over USART2), converts them to individual wheel speed setpoints using mecanum inverse kinematics, and transmits those setpoints to each ODrive every cycle.

At the same time the MCU continuously polls the BNO085 9-DOF IMU over I2C to obtain the robot's orientation and angular velocity. Wheel odometry and IMU data are fused on-MCU by a 6-state EKF, and the result — plus per-axis ODrive status — is bundled into telemetry and sent back to the ROS computer at up to ~50 Hz (slim line) / ~10 Hz (fat line).

In summary the system does five things simultaneously:

1. Receive and parse robot twist commands (vx, vy, wz) from ROS or BT.
2. Run mecanum inverse kinematics and send velocity setpoints over CAN.
3. Receive encoder and status feedback from the ODrives over CAN.
4. Fuse wheel odometry + BNO085 orientation/angular-velocity in an on-MCU EKF.
5. Report firmware-level diagnostics (CAN faults, ODrive faults, IMU faults, watchdogs, stack health) through a queued error-reporting pipeline instead of ad-hoc debug prints.

### 1.2 How the subsystems interact

```
ROS (USART3 RX) ──► UART_RX_Task ────────────────────────┐
                                                          │  ODriveCmdMsg
ESP32 (USART2 RX) ► BT_RX_Task  ────────────────────────►│
                                                          ▼
BNO085 (I2C1) ──► IMU_Task ──► g_bno085_yaw/wz/qx../etc ►│
                                                          │
                           ┌──────────────────────────────┘
                           ▼
                      ODriveTask  ◄──► FDCAN1 ◄──► ODrives (×4)
                           │  (EKF runs here, fusing odom + IMU)
                           │  ODriveTelemetryMsg
                           ▼
                      UART_TX_Task ──► USART3 TX ──► ROS
                           ▲
                           │  drains ERR_QueueHandle every cycle
                      (any task) ── FirmwareError_Push() ──► ERR_QueueHandle
```

All inter-task communication uses FreeRTOS message queues, **except**:
- The BNO085 data, shared via global variables (justified: 32-bit aligned writes are atomic on Cortex-M7, and the IMU task produces data faster than it's consumed — a queue would only add latency).
- `odrives[]` (per-axis CAN feedback), written directly from `HAL_FDCAN_RxFifo0Callback` (ISR context) and read by `ODriveTask`.

### 1.3 BNO055 → BNO085 migration (historical)

The previous firmware version used a BNO055 IMU (simple register-read, synchronous, ~1ms per call). The BNO085 uses the SHTP (Sensor Hub Transport Protocol) over I2C — the sensor pushes reports asynchronously whenever data is ready (signalled by the INT pin going LOW), and the host must continuously service the transport layer (`sh2_service()`) to drain packets before the sensor's internal buffer overflows.

Key practical consequences:
- `sh2_service()` must be called every few milliseconds, not once per control cycle.
- `sh2_open()` blocks for ~310 ms during the hardware reset sequence.
- The SH2 library is not re-entrant — must be called from exactly one task.

These constraints are why a dedicated `IMU_Task` exists (Section 2.6).

---

## Section 2 — FreeRTOS Task Structure

### 2.1 Task table

| Task | Priority | Stack | Period / blocking behaviour |
|---|---|---|---|
| `UART_RX_Task` | High | 4 KB | `osDelay(5)` → ~200 Hz |
| `BT_RX_Task` | High | 4 KB | `osDelay(5)` → ~200 Hz |
| `UART_TX_Task` | Normal | 6 KB (1536×4) | blocks on `CAN_2_UTX_QueueHandle`; also drains the firmware error queue and runs a periodic stack-watermark check |
| `ODriveTask` | AboveNormal | 6 KB (1536×4) | `osDelay(1)`; slim telemetry every cycle, fat telemetry every 5th cycle; EKF update runs every cycle |
| `IMU_Task` | AboveNormal | 4 KB | `osDelay(2)` → ~500 Hz poll rate; entire body skipped via `#if !IMU_ENABLED` when the IMU is disabled at compile time |
| `StartDefaultTask` | Normal | 512 B | created, but gutted to a one-line idle loop (`for(;;) osDelay(1000);`) — kept only to avoid CubeMX regen conflicts, not a real task |
| `ControlTask` | AboveNormal | 1 KB | created, body empty (`osDelay(10)` loop only) |

FreeRTOS heap: `configTOTAL_HEAP_SIZE = 49152` bytes (heap_4 allocator) — raised from 32768 to accommodate the EKF's working memory.
FreeRTOS tick: `configTICK_RATE_HZ = 1000` (1 ms tick).
HAL timebase: TIM6 (separate from SysTick, avoids FreeRTOS conflict).
Stack overflow detection: `configCHECK_FOR_STACK_OVERFLOW = 2` (pattern-checking) — newly enabled; see Section 7.

### 2.2 UART_RX_Task (`start_UART_RX_Task`)

Receives text commands from the ROS computer on USART3 (**230,400 baud**, PD8=TX, PD9=RX — must match `odrive_dashboard.py`'s `baud_rate` parameter).

Interrupt + ring-buffer pattern: `HAL_UART_Receive_IT` arms one-byte reception; `HAL_UART_RxCpltCallback` stores each byte in `rx_buf[]` and re-arms. The task drains the ring buffer every 5 ms, accumulates a line, and calls `UART_RX_ParseLine()` on `\n`/`\r`.

### 2.3 BT_RX_Task (`start_BT_RX_Task`)

Receives velocity commands from the ESP32 Bluetooth module on USART2 (115,200 baud, PD5=TX, PD6=RX). Identical ring-buffer pattern using `rx_buf2[]`/`rx_char2`.

Parses `"3 <vx> <vy> <wz> <buttons_hex>\r\n"` lines, applies dead-zones, handles the BT toggle button, and pushes an `ODriveCmdMsg` (source = `CMD_SOURCE_BT`) into `URX_2_CAN_QueueHandle`. Failed queue puts (queue full) now call `FirmwareError_Push(FERR_BT_QUEUE_FULL, ...)` instead of a printf. See Section 7 for why this used to flood on every boot, and the fix.

### 2.4 UART_TX_Task (`Start_UART_TX_Task`)

The **only** task allowed to call `printf()` (see Section 7 for why). On each wake-up (blocks on `CAN_2_UTX_QueueHandle`):

1. Receives the latest `ODriveTelemetryMsg`.
2. Non-blocking get from `UART_QueueHandle` for the last ROS command (echoed in telemetry only).
3. Emits the **slim** telemetry line every cycle (EKF pose/twist + raw yaw/omega — the two EKF inputs at high rate, for the host's `/odrive/imu` + `/odrive/odom`).
4. Every 5th cycle, emits the **fat** telemetry line (full per-axis diagnostic snapshot — CMD echo, full IMU, IK speeds, EKF pose+quat+covariance, all 4 axes' N/E/S/C/P/V/Sh/CPR/Vbus/Ibus/IqSet/IqMeas/U, BT status, ESP32 link age, SM state).
5. Drains `ERR_QueueHandle`, emitting `E=<code>,<axis>,<detail>\r\n` per entry, or `ELOST=<n>\r\n` if `g_errors_lost > 0`.
6. Every ~250 cycles, checks `uxTaskGetStackHighWaterMark()` across the 5 known tasks and pushes `FERR_STACK_LOW` if any is below 128 words.
7. `osDelay(10)`.

See Section 9 (Quick Reference: Telemetry Output Format) for the exact field layout.

### 2.5 ODriveTask (`StartODriveTask`)

Runs the ODrive state machine, IK, CAN setpoints, the EKF update, and telemetry assembly.

ODrive configuration:

```
odrives[0].NODE_ID = 36   odrives[1].NODE_ID = 34
odrives[2].NODE_ID = 33   odrives[3].NODE_ID = 40
gear_ratio = 9 (all four)
wheel_sign = {-1, +1, -1, +1}
x_offset = 0.195 m, y_offset = 0.195 m, wheel radius = 0.0762 m
```

State machine (`ODriveSMState`): `SM_BOOT → SM_STARTUP → SM_RUNNING`, plus `SM_IDLE` (commanded stop) and `SM_ESTOP` (hardware button held) — see Section 8 for the ESTOP details.

- **SM_BOOT**: waits up to 3000 ms for either an explicit `CFG_STARTUP` command, or the timeout to expire. Before arming, waits (bounded, up to ~3s) for the first heartbeat from every axis — and **drains `URX_2_CAN_QueueHandle` every 10 ms tick during that wait** (fixed this session; see Section 7) so an active BT stream doesn't flood the error queue. Pushes the tuned `vel_ramp_rate` (via RxSdo, Section 10) and `Set_Vel_Gains` before arming each axis with `ODrive_ArmAxisConfirmed(..., VEL_RAMP, 5, i)` — default input mode is now `VEL_RAMP`, not `PASSTHROUGH` (changed this session to smooth velocity setpoint jumps).
- **SM_RUNNING**: per iteration (~1ms): read IMU globals, try a command from `URX_2_CAN_QueueHandle` (2ms wait), process it, run `ODrive_UpdateTelemetryAndOdometry()` (EKF update + telemetry assembly) and push telemetry. Per-axis diagnostics (fault transitions, heartbeat timeout, auto-rearm) are pushed through `FirmwareError_Push()`. A command watchdog zeros all motors if no `SET_VEL` arrives from any source within `CMD_WATCHDOG_TIMEOUT_MS` (500ms).
- **SM_IDLE**: all axes commanded IDLE; waits for `CFG_STARTUP` to return to `SM_RUNNING`.
- **SM_ESTOP**: see Section 8.

BT priority override: a `CMD_SOURCE_BT` command sets `bt_override_active=1`; ROS commands are discarded for the next 500ms unless renewed by another BT command.

### 2.6 IMU_Task (`StartIMUTask`)

Exclusively owns the BNO085 and services the SH2/SHTP protocol stack. **Entire body is skipped** (`for(;;) osDelay(1000);`) when `IMU_ENABLED=0` at compile time — added this session to stop spurious `FERR_IMU_REPORT_CFG` errors that fired every boot in stub/bench-test mode (the SH2 calls were hitting an I2C IO error since no hardware was present).

When enabled, startup sequence: `sh2_open()` → drain SHTP advertisement → register sensor callback → `imu_enable_all_reports()` (game rotation vector, gyro, linear acceleration — see Section 5.3) → main loop: `sh2_service()` + `osDelay(2)` forever, re-configuring on a detected sensor reset.

Data lands in `g_bno085_yaw/roll/pitch` (legacy Euler, degrees-on-read) plus `g_bno085_qx/qy/qz/qw`, `g_bno085_wx/wy/wz`, `g_bno085_ax/ay/az` (quaternion, angular velocity, linear acceleration — feeds `sensor_msgs/Imu` downstream). `g_bno085_seq` increments on every fresh sample; the EKF uses it to dedup corrections (Section 8).

### 2.7 Why the BNO085 must have its own task

(Unchanged from the original integration — still accurate.) Five reasons: (1) SHTP requires continuous polling or packets are overwritten; (2) `sh2_open()` blocks ~310ms, which would stall CAN comms if run inline; (3) the SH2 library is not re-entrant; (4) the IMU's 50Hz rate has no natural sync with the ~1kHz CAN loop — the shared-global approach decouples them cleanly; (5) a previous inline-read integration attempt was unreliable for exactly reason (1).

### 2.8 Per-task flow diagrams

Each diagram below shows one full iteration of the task's main loop. Cross-reference Sections 2.2–2.6 above for the prose explanation of each step.

#### UART_RX_Task

```
┌─────────────────────────────────────────────────────────────┐
│  start_UART_RX_Task()                                       │
│                                                               │
│  HAL_UART_Receive_IT(&huart3, &rx_char, 1)  ── armed once    │
│                                                               │
│  loop (osDelay(5) ≈ 200 Hz):                                 │
│    ┌─────────────────────────────────────────────────────┐  │
│    │ drain rx_buf[] (filled by ISR) until head == tail   │  │
│    │   for each byte:                                    │  │
│    │     accumulate into line_buf[]                      │  │
│    │     if byte == '\n' or '\r':                        │  │
│    │         UART_RX_ParseLine(line_buf, ...)             │  │
│    │         reset line_buf                               │  │
│    └─────────────────────────────────────────────────────┘  │
│    osDelay(5)                                                │
└─────────────────────────────────────────────────────────────┘
        ▲
        │ byte arrives
┌───────┴─────────────────────────────────────────────────────┐
│ HAL_UART_RxCpltCallback (ISR, USART3)                        │
│   rx_buf[head++] = rx_char                                   │
│   HAL_UART_Receive_IT(&huart3, &rx_char, 1)  ── re-arm        │
└───────────────────────────────────────────────────────────────┘
```

`UART_RX_ParseLine` (called from the task, not the ISR) dispatches on the leading integer: `1` → `ODRIVE_CMD_SET_VEL` into both `UART_QueueHandle` (telemetry echo) and `URX_2_CAN_QueueHandle` (control path); `2` → one of the `ODRIVE_CFG_*` sub-types (Section 9) into `URX_2_CAN_QueueHandle` only.

#### BT_RX_Task

```
┌─────────────────────────────────────────────────────────────┐
│  start_BT_RX_Task()                                          │
│  loop (osDelay(5) ≈ 200 Hz):                                 │
│    drain rx_buf2[] (filled by USART2 ISR, same ring pattern) │
│    for each completed line "3 <vx> <vy> <wz> <buttons>":     │
│        apply dead-zones (|vx|,|vy| < 0.05, |wz| < 0.05)      │
│        if buttons & BT_UART_TOGGLE_BUTTON:                   │
│            BT_active = 0; queue a zero-vel stop cmd; continue │
│        else:                                                  │
│            BT_active = (controll_state from ESP32 line)       │
│            on 2→<2 transition: queue one stop cmd             │
│              (releases BT override immediately, doesn't wait  │
│               for the 500ms watchdog)                         │
│            if BT_active == 2:                                 │
│                build ODriveCmdMsg{source=CMD_SOURCE_BT, ...}  │
│                osMessageQueuePut(URX_2_CAN_QueueHandle, ...)  │
│                  on fail → FirmwareError_Push(FERR_BT_QUEUE_FULL)│
│        on any parse failure →                                 │
│            FirmwareError_Push(FERR_BT_PARSE_FAIL)              │
│    BT watchdog: if no byte for BT_WATCHDOG_MS and UART2 RX     │
│      state != READY, self-heal (clear OREFLAG, re-arm IT)      │
│    osDelay(5)                                                  │
└─────────────────────────────────────────────────────────────┘
```

#### UART_TX_Task

```
┌─────────────────────────────────────────────────────────────────┐
│  Start_UART_TX_Task()                                           │
│  loop:                                                            │
│    osMessageQueueGet(CAN_2_UTX_QueueHandle, &telemetryMsg,        │
│                       osWaitForever)        ◄── blocks here       │
│    osMessageQueueGet(UART_QueueHandle, &last_cmd, NULL, 0)        │
│                                              (non-blocking echo)   │
│    telemetryMsg.bt_active = BT_active        (live snapshot)      │
│    esp32_age_ms = HAL_GetTick() - g_bt_last_valid_msg_tick         │
│                                              (-1 if never seen)    │
│    printf(SLIM line)                         ── every cycle       │
│    if (++tx_count % 5 == 0): printf(FAT line) ── every 5th cycle   │
│    drain ERR_QueueHandle:                                          │
│        while queue not empty: printf("E=%u,%u,%u\r\n", ...)        │
│        if g_errors_lost > 0: printf("ELOST=%u\r\n", ...)           │
│    every ~250 cycles: check uxTaskGetStackHighWaterMark() on        │
│        all 5 known tasks → FirmwareError_Push(FERR_STACK_LOW)       │
│        if any task < 128 words remaining                            │
│    osDelay(10)                                                      │
└───────────────────────────────────────────────────────────────────┘
```

This is the **only** task in the whole firmware allowed to call `printf()` — see Section 7.

#### IMU_Task

```
┌───────────────────────────────────────────────────────────────┐
│  StartIMUTask()                                                │
│                                                                  │
│  #if !IMU_ENABLED:                                              │
│      for(;;) osDelay(1000);    ◄── entire body skipped           │
│  #else:                                                          │
│      osDelay(500)                                                │
│      sh2_open(hal, async_cb, NULL)                                │
│        └─ hal_open(): RST pulse (10ms LOW, 300ms wait HIGH)       │
│        on fail → FirmwareError_Push(FERR_IMU_OPEN); halt loop      │
│      imu_service_ms(200)        drain SHTP advertisement           │
│      sh2_setSensorCallback(imu_sensor_data_cb, NULL)                │
│        on fail → FirmwareError_Push(FERR_IMU_SET_CALLBACK)          │
│      imu_service_ms(100)                                            │
│      imu_enable_all_reports()  ── game rotation vector + gyro        │
│                                    + linear accel, 20ms interval      │
│      g_bno085_sensor_ready = 0                                       │
│                                                                       │
│      loop (osDelay(2) ≈ 500 Hz poll):                                 │
│        sh2_service()                                                  │
│          └─ if INT pin (PD14) LOW: hal_read() over I2C1                │
│               └─ decodes SHTP packet → fires imu_sensor_data_cb         │
│                    → writes g_bno085_yaw/qx../wx../ax..                 │
│                    → g_bno085_seq++ (LAST, so a fresh seq implies        │
│                       fresh data — see Section 11.4)                     │
│             else: returns immediately, no I2C transaction                │
│        if g_bno085_sensor_ready (BNO085 sent a reset event):              │
│            FirmwareError_Push(FERR_IMU_RESET)                              │
│            imu_service_ms(200); imu_enable_all_reports()  ── reconfigure   │
│        osDelay(2)                                                          │
│  #endif                                                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

#### ODriveTask — main loop (high level; state machine detail in Section 2.9)

```
┌────────────────────────────────────────────────────────────────────┐
│  StartODriveTask()  — one-time setup: NODE_IDs, gear ratios,        │
│    EKF ekf; ekf_init(&ekf, NULL); ekf.use_imu = EKF_TRUST_IMU;       │
│    sm_state = SM_BOOT                                                │
│                                                                        │
│  loop (osDelay(1) ≈ 1 kHz):                                           │
│    now = osKernelGetTickCount()                                       │
│    ① HARDWARE ESTOP sample-after-delay debounce + release hold         │
│         (Section 2.9 / Section 8 — runs BEFORE the state switch,        │
│          can force sm_state = SM_ESTOP regardless of what follows)      │
│    ② FDCAN_RecoverIfBusOff() — if it just recovered AND sm_state ==      │
│         SM_RUNNING, send one zero-velocity SET_VEL (stale-setpoint guard) │
│    ③ snapshot g_bno085_* into telemetryMsg.imu                            │
│    ④ qst = osMessageQueueGet(URX_2_CAN_QueueHandle, &cmd, NULL, 2)         │
│    ⑤ command-watchdog bookkeeping (refresh/disarm last_vel_cmd_tick)       │
│    ⑥ switch(sm_state): SM_BOOT / SM_STARTUP / SM_RUNNING / SM_IDLE /        │
│         SM_ESTOP(no-op)   ◄── full detail in Section 2.9                    │
│    ⑦ if (now - last_telem_tick) >= telemetry_period (20ms):                 │
│         ODrive_UpdateTelemetryAndOdometry()  — EKF predict+correct,          │
│           fills OdomData (Section 5.2)                                       │
│         ODrive_PushLatestTelemetry(CAN_2_UTX_QueueHandle, &telemetryMsg)      │
│         last_telem_tick = now                                                 │
│    osDelay(1)                                                                 │
└────────────────────────────────────────────────────────────────────────────┘
```

### 2.9 ODriveTask state machine — detailed diagram

Supersedes the simplified diagram that used to live in `odrive_task_config.c` (a design-reference file predating the ESTOP, command watchdog, BT-priority, bus-off recovery, and auto-rearm logic; deleted once its content — this diagram plus the CAN command reference in Section 10.3 — was fully migrated here). This reflects the actual current implementation.

```
                                    power-on / reset
                                           │
                                           ▼
                                    ┌─────────────┐
                              ┌────▶│  SM_BOOT    │
                              │     └─────────────┘
                              │            │
                              │   ┌────────┴─────────────────────────┐
                              │   │                                  │
                              │   ▼ explicit CFG_STARTUP cmd          ▼ boot_delay_ms (3000ms) elapsed, no cmd
                              │  ODrive_ProcessCommand(cmd)            wait up to ~3s for EVERY axis heartbeat
                              │  (uses cmd's ctrl_mode/input_mode/      (CAN_STUB: skipped — loopback has no HB)
                              │   axis_state as given)                 │   while waiting: drain+discard
                              │   │                                    │   URX_2_CAN_QueueHandle every 10ms tick
                              │   │                                    │   (fixed this session — was previously
                              │   │                                    │    undrained, see Section 7.4)
                              │   │                                    ▼
                              │   │                              Set_Vel_Gains() ×4  (ODRIVE_STARTUP_VEL_GAIN/
                              │   │                                                   VEL_INT_GAIN)
                              │   │                              Set_Param_Float() ×4 (vel_ramp_rate=25.0,
                              │   │                                                    endpoint 398, RxSdo)
                              │   │                              ODrive_ArmAxisConfirmed(VELOCITY_CONTROL,
                              │   │                                  VEL_RAMP, 5 retries) ×4
                              │   │                                  (retry loop: Clear_Errors → Set_Controller_
                              │   │                                   Modes → Set_Axis_Requested_State(CLOSED_LOOP)
                              │   │                                   → wait ≤300ms for heartbeat to confirm;
                              │   │                                   on exhaustion → FERR_ARM_TIMEOUT(axis_idx))
                              │   │                                    │
                              │   ▼                                    ▼
                              │  current_ctrl_mode=as commanded   current_ctrl_mode=VELOCITY_CONTROL
                              │  current_input_mode=as commanded  current_input_mode=VEL_RAMP
                              │   │                                    │
                              │   └──────────────┬─────────────────────┘
                              │                  ▼
                              │           ┌─────────────┐
                              │           │ SM_STARTUP  │  (transient — falls through to
                              │           └─────────────┘   SM_RUNNING on the very next loop
                              │                  │           iteration; exists only as a label)
                              │                  ▼
              ┌───────────────────────────┌─────────────┐
              │  CFG_REBOOT received  ◄───│ SM_RUNNING  │◄────────────────────────────┐
              │  (sm_state=SM_BOOT,        └─────────────┘                            │
              │   boot_tick reset)          │  │  │  │                                │
              └────────────────────────────┘  │  │  │                                │
                                               │  │  │                                │
   ┌───────────────────────────────────────────┘  │  └──────────────────────────┐    │
   │ every iteration (independent of cmd):         │                            │    │
   │  • REARM_PERIOD_MS tick: any axis not in       │                            │    │
   │    CLOSED_LOOP_CONTROL (and not UNDEFINED) →    │                            │    │
   │    FERR_AXIS_REARM, Clear_Errors, re-arm,        │                           │    │
   │    Set_Input_Vel(0,0)  — rate-limited auto-heal   │                          │    │
   │  • CMD_WATCHDOG_TIMEOUT_MS (500ms) since last      │                         │    │
   │    SET_VEL from ANY source → FERR_CMD_WATCHDOG,     │                        │    │
   │    latch, periodic zero-vel resend (WD_STOP_RESEND_MS)│                      │    │
   │    until a fresh SET_VEL clears the latch              │                     │    │
   │  • BT_OVERRIDE_TIMEOUT_MS since last BT packet while    │                    │    │
   │    bt_override_active → release override, zero velocity  │                  │    │
   └─────────────────────────────────────────────────────────┐ │                  │    │
                                                              │ │                  │    │
   command arrived this iteration (qst==osOK):                │ │                  │    │
     bit BT_ESTOP_BUTTON set? ──────────────────────────────► STOP_ODRIVES ──► SM_IDLE  │
     cmd.type == CFG_STOP / CMD_STOP_ODRIVES (unconditional,                            │
       even under active BT override — safety) ─────────────────────────────► SM_IDLE  │
     cmd.source == CMD_SOURCE_BT:                                                       │
       BT_active==2 → bt_override_active=1, refresh last_bt_tick                         │
       else → bt_override_active=0 (toggle-off / paired)                                 │
     cmd.source == ROS, bt_override_active && within timeout → SILENTLY DROPPED           │
     cmd.type == CFG_STOP → SM_IDLE (duplicate path, kept for the non-unconditional        │
       check above already having handled it; see main.c for the exact branch order)       │
     cmd.type == CFG_REBOOT → SM_BOOT (boot_tick reset) ───────────────────────────────────┘
     anything else (SET_VEL, gain/limit/param tuning, etc.) → ODrive_ProcessCommand(cmd),
       stays in SM_RUNNING

                              ┌─────────────┐
                              │  SM_IDLE    │  (all axes IDLE; commanded stop)
                              └─────────────┘
                                     │
                                     ▼ CFG_STARTUP received
                              ODrive_ProcessCommand(cmd) → SM_RUNNING


   ═══════════════════════ HARDWARE ESTOP (orthogonal to the switch above) ═══════════════════
   Checked at the TOP of every loop iteration, BEFORE the sm_state switch. Can force a
   transition INTO SM_ESTOP from any state; the switch's SM_ESTOP case is a no-op (the
   telemetry/IMU/bus-off-recovery code below the switch still runs every iteration, so the
   dashboard keeps seeing live data while e-stopped).

   ESTOP_PIN (PE2) edge ──► ISR sets g_estop_sample_pending, g_estop_sample_tick=now
       │
       ▼ task samples after ESTOP_DEBOUNCE_MS (90ms) of no further edges
   pin == HIGH (NC open = pressed)?
       │ YES                                          │ NO (released)
       ▼                                               ▼
   estop_release_pending=0 (cancel any pending      if sm_state==SM_ESTOP:
   re-arm)                                              estop_release_pending=1
   if sm_state != SM_ESTOP:                             estop_release_tick=now
       Set_Input_Vel(0,0) ×4                                  │
       Set_Axis_Requested_State(IDLE) ×4                       ▼
       sm_state = SM_ESTOP                          wait ESTOP_RELEASE_HOLD_MS (150ms)
       g_estop_active = 1                            of CONTINUOUS release (any re-press
       ESTOP_LED ON (PE0 HIGH)                       before this elapses cancels the re-arm)
                                                              │
                                                              ▼ hold elapsed, pin re-confirmed LOW
                                                       Set_Input_Pos(current encoder pos) ×4
                                                         (snapshot — prevents a lurch on re-arm)
                                                       ODrive_ArmAxisConfirmed(current_ctrl_mode,
                                                         current_input_mode, 5 retries) ×4
                                                         (NOT hardcoded — re-arms to whatever
                                                          mode was active before the e-stop, so
                                                          the VEL_RAMP boot default survives)
                                                       sm_state = SM_RUNNING
                                                       g_estop_active = 0
                                                       ESTOP_LED OFF
```

---

## Section 3 — Task Interaction and Data Flow

### 3.1 Producer / consumer summary

| Data | Produced by | Consumed by |
|---|---|---|
| `ODriveCmdMsg` (vel/config command) | `UART_RX_Task` | `ODriveTask` (via `URX_2_CAN_QueueHandle`) |
| `ODriveCmdMsg` (BT command) | `BT_RX_Task` | `ODriveTask` (via `URX_2_CAN_QueueHandle`) |
| `ODriveCmdMsg` (last ROS cmd, echo) | `UART_RX_Task` | `UART_TX_Task` (via `UART_QueueHandle`) |
| `odrives[i]` (encoder/status) | FDCAN1 RX ISR | `ODriveTask` (direct global read) |
| `g_bno085_*` (IMU data) | `IMU_Task` | `ODriveTask` / EKF (direct global read) |
| `ODriveTelemetryMsg` | `ODriveTask` | `UART_TX_Task` (via `CAN_2_UTX_QueueHandle`) |
| `FirmwareError` | any task | `UART_TX_Task` (via `ERR_QueueHandle`) |
| USART3 TX output | `UART_TX_Task` | ROS computer |
| FDCAN1 TX frames | `ODriveTask` | ODrives (via CAN bus) |

---

## Section 4 — Queues and RTOS Elements

| Queue | Depth × size | Writers | Readers | Notes |
|---|---|---|---|---|
| `URX_2_CAN_QueueHandle` | 3 × `ODriveCmdMsg` | `UART_RX_Task`, `BT_RX_Task` | `ODriveTask` (2ms poll) | Carries robot commands to the state machine. Drained-and-discarded during the `SM_BOOT` heartbeat wait (fixed this session — see Section 7). |
| `UART_QueueHandle` | 3 × `ODriveCmdMsg` | `UART_RX_Task` | `UART_TX_Task` (non-blocking) | Telemetry-echo only, not part of the control path. |
| `CAN_2_UTX_QueueHandle` | 3 × `ODriveTelemetryMsg` | `ODriveTask` | `UART_TX_Task` (blocking) | Drop-newest-on-full: stale telemetry is never preferred over fresh. |
| `ERR_QueueHandle` | 16 × `FirmwareError` | any task except `UART_TX_Task` itself | `UART_TX_Task` | New this session — see Section 7. Drop-and-count on full (`g_errors_lost`), never blocks. |

### 4.1 `MutexUART_DataHandle`

Created (`osMutexNew`) but never acquired or released anywhere in the codebase — unchanged from the original integration. Previously this was a real risk (`printf()` is not thread-safe); it's now mitigated **by convention** instead of by using this mutex: only `UART_TX_Task` calls `printf()`. If you ever need a second task to print directly, either wire up this mutex around every `printf()` call, or (preferred) route through `FirmwareError_Push()` instead, matching the existing pattern.

### 4.2 Task priorities

Unchanged rationale from the original integration: `UART_RX_Task`/`BT_RX_Task` at `High` (mostly sleeping, must drain ring buffers before they wrap); `ODriveTask`/`IMU_Task` at `AboveNormal` (time-sensitive); `UART_TX_Task` at `Normal` (tolerates latency, sleeps on queue).

### 4.3 The `g_bno085_*` globals instead of a queue

Unchanged rationale — 32-bit aligned atomic writes, acceptable skew, avoids queue overhead. Now includes the additional quaternion/angular-velocity/linear-acceleration globals feeding the EKF and `sensor_msgs/Imu`, not just yaw/pitch/roll.

---

## Section 5 — Important Functions

### 5.1 BNO085 initialization

Unchanged call chain: `StartIMUTask()` → `sh2_open()` → `hal_open()` (RST pulse, 300ms boot wait) → SHTP advertisement drain → `sh2_setSensorCallback()` → `imu_enable_all_reports()` (configures rotation vector, gyro, linear acceleration reports at 50Hz). See `sh2_hal_impl.c` for the HAL implementation.

### 5.2 `ekf_correct_imu` / on-MCU EKF update (NEW — replaces dead-reckoning)

`ODrive_UpdateTelemetryAndOdometry()` (in `main.c`) no longer dead-reckons `x/y/theta` directly. Instead:

1. Predicts the EKF state forward using wheel-derived body velocity (`bodySpeedsFromUMecanum`).
2. Corrects on the BNO085 yaw + omega_z, gated by `g_bno085_seq` so each fresh IMU sample is consumed at most once (`ekf_correct_imu()`, in `ekf.c`). Honors `ekf->use_imu` (`EKF_TRUST_IMU`, currently 1) and rejects yaw outliers (`|Δyaw| > 40°` between consecutive samples — corrupted-packet protection ported from the old dashboard-side check).
3. Fills `OdomData` (legacy `x_pos/y_pos/phi` fields for backward compatibility, plus EKF-shaped quaternion, body-frame velocity, and full 6×6 pose/twist covariance).

**The EKF struct itself is a local variable inside `StartODriveTask`'s infinite loop** (`ekf_init()` called once when the task starts). It is **not** reset by restarting the ROS dashboard — that only restarts the PC-side parser/cache. To re-zero the EKF for testing, you must reset the STM32 itself (power cycle, NRST, ST-Link reset, or reflash). There is currently no CAN/UART command to reset just the EKF or the MCU at runtime.

See Section 11 for the full design write-up, the math, and known gaps (no pitch/roll, no IMU outlier rejection on omega_z specifically, no on-MCU gyro bias estimation, no IMU staleness watchdog).

### 5.3 `q_to_ypr` (`CM7/Core/Src/sh2/euler.c`)

Unchanged — converts a unit quaternion to yaw/pitch/roll (radians) via the ZYX convention. Note: the BNO085 on this robot reports yaw + omega_z **inverted** vs. the ROS `base_link` convention; this is now corrected once, in firmware, before the EKF integrates anything (`main.c`, just before `ekf_correct_imu`) — previously this correction lived downstream in the dashboard, which is no longer needed/correct once the EKF firmware is flashed (a double-invert risk if both corrections are active — see Section 12.1 item 3 for the home2 `omnidriver` migration note).

### 5.4 CAN message encoding/decoding

Unchanged fixed-command-set mechanics (`Set_TX_Param`, 11-bit ID = `(node_id<<5)|cmd_id`, `FDCAN_WAIT_TX_FREE()` with a 50ms hard timeout — now pushes `FERR_CAN_TX_TIMEOUT` instead of printing). See Section 10 for the new RxSdo arbitrary-parameter-write path, which uses cmd_id `0x004` instead of one of the fixed command IDs.

### 5.5 / 5.6 Mecanum IK (`computeNecessaryWheelSpeedsMecanum` / `bodySpeedsFromUMecanum`)

Unchanged kinematic model. `bodySpeedsFromUMecanum` (renamed from `globalSpeedsFromUMecanum`) now feeds the EKF's predict step instead of directly dead-reckoning a global pose.

### 5.7 `ODrive_ArmAxisConfirmed` (signature changed this session)

```c
static uint8_t ODrive_ArmAxisConfirmed(Axis *axis, FDCAN_TXmsg *msg,
                                       Control_Mode ctrl, Input_Mode in_mode,
                                       uint8_t attempts, uint8_t axis_idx)
```

Retries `Clear_Errors` + `Set_Controller_Modes` + `Set_Axis_Requested_State(CLOSED_LOOP_CONTROL)` up to `attempts` times, waiting up to 300ms per attempt for the heartbeat to confirm. On total failure, pushes `FirmwareError_Push(FERR_ARM_TIMEOUT, axis_idx, ...)`. The `axis_idx` parameter was added this session — previously this logged `axis->NODE_ID` (e.g. 36/34/33/40) instead of the 0-3 index every other error code uses, which made the error log misleading.

### 5.8 `FirmwareError_Push` (NEW)

```c
static void FirmwareError_Push(uint8_t code, uint8_t axis, uint8_t detail) {
    FirmwareError err = {code, axis, detail};
    if (osMessageQueuePut(ERR_QueueHandle, &err, 0, 0) != osOK)
        g_errors_lost++;
}
```

Non-blocking put, drop-and-count on full. Safe to call from any task context. **Not safe to call from ISR context** without verifying the calling IRQ's priority against `configMAX_SYSCALL_INTERRUPT_PRIORITY` — this is why `HAL_FDCAN_RxFifo0Callback`/`HAL_FDCAN_ErrorStatusCallback` only set bits in the `g_can_busoff` flag instead of calling this directly; the actual push happens in task context inside `FDCAN_RecoverIfBusOff()`.

### 5.9 `Set_Param_Float` (NEW, `ODrive.c`)

```c
HAL_StatusTypeDef Set_Param_Float(const Axis *axis, FDCAN_TXmsg *msg,
                                   uint16_t endpoint_id, float value);
```

Builds an 8-byte RxSdo `OPCODE_WRITE` frame (`opcode(1B) + endpoint_id(2B LE) + reserved(1B) + value(4B)`) at cmd_id `0x004` and queues it. See Section 10.

### 5.10 `vApplicationStackOverflowHook` (NEW)

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    g_stack_overflow_detected = 1;
    HAL_GPIO_WritePin(ESTOP_LED_PORT, ESTOP_LED_PIN, GPIO_PIN_SET);
    for (;;);
}
```

Cannot safely call any FreeRTOS API from this hook (it runs in a context where the offending task's own stack is already corrupted). Just sets a flag, drives the ESTOP LED on directly via HAL, and halts — a deliberate fail-stop, not a recovery attempt.

### 5.11 `UART_RX_ParseLine`

Unchanged Type-1/Type-2 dispatch, but Type-2's sub-command switch now includes `sub_type=31` (`ODRIVE_CFG_SET_PARAM_FLOAT`) — see Section 10.

### 5.12 `HAL_FDCAN_RxFifo0Callback` / `Find_ODrive_By_NodeID`

Unchanged RX-ISR mechanics, with one addition: on `HAL_FDCAN_GetRxMessage()` failure, sets `g_can_busoff |= 0x02` (RX-read-fail bit) instead of doing nothing — converted to `FERR_CAN_RX_FAIL` in task context.

---

## Section 6 — Source and Header Files

### 6.1 BNO085 integration files

Unchanged from the original integration: `sh2_hal_impl.c/.h`, `sh2/sh2.c`, `sh2/shtp.c`, `sh2/sh2_SensorValue.c`, `sh2/sh2_util.c`, `sh2/euler.c` and matching headers. See Section 2.6/2.7 for the architectural rationale.

### 6.2 ODrive / robot control files

`CM7/Core/Src/main.c` — the central file. Now also contains: `FirmwareError_Push()`, `vApplicationStackOverflowHook()`, the ESTOP debounce/hold state machine, `Set_Param_Float()` call sites, and the EKF integration inside `ODrive_UpdateTelemetryAndOdometry()`.

`CM7/Core/Src/ODrive.c` + `CM7/Core/Inc/ODrive.h` — low-level CAN TX functions. Gained `Set_Param_Float()` and the `RXSDO_CMD_ID`/`SDO_OPCODE_*` defines this session.

`CM7/Core/Inc/main.h` — project-level type definitions. Gained the `FirmwareError` struct + all `FERR_*` defines, `ODRIVE_CFG_SET_PARAM_FLOAT`, and `param_endpoint_id`/`param_value` fields on `ODriveCmdMsg`. `OdomData` gained the EKF-shaped fields (quaternion, body-frame velocity, full 6×6 pose/twist covariance) — kept alongside the legacy `x_pos/y_pos/phi` fields for backward compatibility.

**`CM7/Core/Inc/ekf.h` + `CM7/Core/Src/ekf.c` (NEW)** — the 6-state EKF implementation: `ekf_init()`, `ekf_predict()`/equivalent, `ekf_correct_wheel_twist()`, `ekf_correct_imu()`. See Section 11 for the math and design rationale.

`odrive_task_config.c` — **deleted this session.** Was a design-reference file (never compiled), predating the ESTOP/watchdog/auto-rearm logic. Its state machine diagram is now Section 2.9, its UART parser sub-type table is reflected in Section 9, and its list of unimplemented ODrive CAN commands (Estop, Get_Error, Get_Temperature, Set_Traj_*, Set_Absolute_Position, Get_Torques, Get_Powers) is now Section 10.3.

### 6.3 General firmware infrastructure

Unchanged: `myprintf.c/.h` (printf retarget to USART3, polling, not thread-safe — see Section 4.1 for how this risk is now mitigated by convention), `stm32h7xx_hal_msp.c`, `stm32h7xx_hal_timebase_tim.c`, `stm32h7xx_it.c`, `syscalls.c`.

`CM7/Core/Inc/FreeRTOSConfig.h` — `configTOTAL_HEAP_SIZE` raised to 49152 (was 32768) for the EKF's working memory; `configCHECK_FOR_STACK_OVERFLOW = 2` newly added.

### 6.4 Legacy / unused files

Unchanged: `bno055.c/.h` (old IMU driver, ~2KB dead flash), `mcp2515.*` (unused SPI CAN driver), `uart_rx.h` (minimal header, real implementation in `main.c`).

---

## Section 7 — Firmware Error Reporting System

### 7.1 Why it exists

Previously, diagnostic messages were scattered `printf()` calls across every task. Since `printf()` is not mutex-protected (Section 4.1), this risked corrupted UART output whenever two tasks printed close together. The fix: **only `Start_UART_TX_Task` may call `printf()`.** Every other task calls `FirmwareError_Push(code, axis, detail)` instead (Section 5.8), and the TX task is the sole consumer that drains and prints them.

### 7.2 Wire format

```
E=<code>,<axis>,<detail>\r\n      one line per queued error, drained each TX cycle
ELOST=<n>\r\n                      emitted only if the queue overflowed since the last drain
```

`axis` is `0-3` for per-axis errors, `0xFF` (`FERR_NO_AXIS`) for system-level errors. `detail` is code-specific (HAL status, SH2 return code, axis state, remaining stack words, etc.).

### 7.3 Error codes (`main.h`)

| Code | Name | Meaning |
|---|---|---|
| `0x01` | `FERR_CAN_TX_TIMEOUT` | `FDCAN_WAIT_TX_FREE()` hit its 50ms hard timeout |
| `0x02` | `FERR_CAN_BUSOFF` | FDCAN bus-off detected and recovered |
| `0x03` | `FERR_CAN_RX_FAIL` | `HAL_FDCAN_GetRxMessage()` failed in the RX ISR |
| `0x10` | `FERR_STARTUP_FAILED` | `ODRIVE_CFG_STARTUP` sequence failed |
| `0x20` | `FERR_ARM_TIMEOUT` | `ODrive_ArmAxisConfirmed()` exhausted all retries |
| `0x21` | `FERR_AXIS_REARM` | axis fell out of `CLOSED_LOOP_CONTROL`, auto-rearm issued |
| `0x30` | `FERR_AXIS_FAULT` | `AXIS_Error` register went non-zero |
| `0x31` | `FERR_CMD_WATCHDOG` | no `SET_VEL` within `CMD_WATCHDOG_TIMEOUT_MS` |
| `0x32` | `FERR_HEARTBEAT_TIMEOUT` | axis stopped sending CAN heartbeats |
| `0x40` | `FERR_IMU_OPEN` | `sh2_open()` failed |
| `0x41` | `FERR_IMU_SET_CALLBACK` | `sh2_setSensorCallback()` failed |
| `0x42` | `FERR_IMU_REPORT_CFG` | `sh2_setSensorConfig()` failed |
| `0x43` | `FERR_IMU_RESET` | BNO085 reset detected, re-configuring |
| `0x50` | `FERR_BT_QUEUE_FULL` | BT command dropped, `URX_2_CAN_QueueHandle` full |
| `0x51` | `FERR_BT_PARSE_FAIL` | malformed Type-3 BT line |
| `0x60` | `FERR_STACK_LOW` | a task's stack high-water mark dropped below 128 words (axis field encodes task index) |
| `0x61` | `FERR_STACK_OVERFLOW` | `vApplicationStackOverflowHook` fired |

The dashboard (`odrive_dashboard.py`'s `FERR_DESCRIPTIONS`) maps every code to a human-readable string and shows them in a dedicated panel separate from the general activity log.

### 7.4 Fixes made this session

- **Boot-time BT-queue flood**: the `SM_BOOT` heartbeat-wait loop (up to 3s) previously left `URX_2_CAN_QueueHandle` (depth 3) completely undrained. An active BT stream during that window filled it almost instantly, flooding `FERR_BT_QUEUE_FULL` and `ELOST` on every boot. Fixed by draining-and-discarding the queue every 10ms tick inside that wait loop — commands during arming are expected to be dropped, so this is a silent discard, not an error.
- **`FERR_ARM_TIMEOUT` axis numbering**: was logging `axis->NODE_ID` (36/34/33/40) instead of the 0-3 index every other code uses. Fixed by adding an `axis_idx` parameter to `ODrive_ArmAxisConfirmed()`.

---

## Section 8 — ESTOP State Machine

Hardware emergency-stop input on **PE2**, normally-closed button wired to GND with the STM32 internal pull-up enabled — resting (closed) reads LOW, pressed (open) reads HIGH. A broken wire also reads HIGH, so the design is fail-safe against open-circuit harness faults. ESTOP status LED on **PE0** (push-pull, active HIGH, on while `SM_ESTOP` is active).

Two independent timers gate the transition:

1. **`ESTOP_DEBOUNCE_MS` (90ms)** — sample-after-delay debounce for both press and release detection. Rejects mechanical bounce on either edge.
2. **`ESTOP_RELEASE_HOLD_MS` (150ms)** — once a release is detected, the firmware additionally requires this much *continuous* release before re-arming. Any re-press during this hold window cancels the pending re-arm (`estop_release_pending = 0`). This exists so a noisy/bouncy release edge can't immediately re-engage the motors.

On entering `SM_ESTOP`: all axes set IDLE (base becomes push-movable), LED on. On confirmed release: each axis gets a `Set_Input_Pos` snapshot of its current encoder position (so a position-mode controller doesn't lurch), then `ODrive_ArmAxisConfirmed()` re-arms with whatever `current_ctrl_mode`/`current_input_mode` was active before the ESTOP — **not** a hardcoded value, so the boot-time `VEL_RAMP` default is preserved across an ESTOP cycle.

---

## Section 9 — Quick Reference: UART Command Protocol

USART3 (ROS, 230,400 baud), USART2 (ESP32, 115,200 baud), 8N1.

**From ROS (USART3):**

```
Type 1 — velocity command:
  "1 <vx> <vy> <wz>\r\n"          vx, vy in m/s; wz in rad/s. All four axes.

Type 2 — configuration:
  "2 <sub> <mask_hex> [params]\r\n"
  sub=20 0x0F             clear errors on all axes
  sub=21 0x0F <state>     set axis state (e.g. 8=CLOSED_LOOP)
  sub=22 0x0F <cm> <im>   set controller mode (cm=2=VEL) + input mode
  sub=23 0x0F <vl> <cl>   set velocity limit + current limit
  sub=24 0x0F <pg>        set position gain
  sub=25 0x0F <vg> <vi>   set velocity gain + integrator gain
  sub=26 0x0F [cm im st]  startup sequence (default: cm=2 im=2(VEL_RAMP) st=8)
  sub=27 0x0F             reboot ODrive
  sub=28 0x0F <tq>        set input torque (all axes same torque)
  sub=29 0x0F             stop (vel→0 then IDLE state)
  sub=30 0x0F <pos> <vff> <tff>          set input position with feedforwards
  sub=31 0x0F <endpoint_id> <value>      NEW: arbitrary float32 parameter
                                          write over CAN (RxSdo). endpoint_id
                                          is firmware-build-specific — see
                                          Section 10.
```

**From ESP32/BT (USART2):**

```
Type 3 — BT velocity command:
  "3 <vx> <vy> <wz> <buttons_hex>\r\n"
  buttons bit 1 (0x02) = toggle BT off and queue a stop command.
  Any other valid type-3 packet activates BT_active=1.
  BT commands have 500 ms override priority over ROS commands.
```

---

## Section 10 — Quick Reference: Telemetry Output Format & CAN Parameter Access

### 10.1 Telemetry lines

USART3 TX, 230,400 baud. Two line shapes, sharing a single index map (see the comment block directly above the printfs in `Start_UART_TX_Task`):

**Slim line** (every cycle, ~50 Hz): `IMU_yaw`, `IMU_wz`, `ODOM_phi/x/y/z`, `ODOM_qx/qy/qz/qw`, `ODOM_w/vx/vy`, `ODOM_vxb/vyb`, `ODOM_var_x/y/yaw`, `ODOM_var_vx/vy/wz`, `SM_state`. Carries exactly the EKF outputs the host needs for `/odrive/imu` + `/odrive/odom` at high rate.

**Fat line** (every 5th cycle, ~10 Hz): everything in the slim line, plus `CMD_vx/vy/wz`, `IMU_roll/pitch`, `IMU_qx/qy/qz/qw`, `IMU_wx/wy/wz`, `IMU_ax/ay/az`, `IK_u0..u3`, per-axis (×4) `N/E/S/C/P/V/Sh/CPR/Vbus/Ibus/IqSet/IqMeas/U`, `BT_active`, `BT_vx/vy/wz`, `ESP32_age_ms`.

**Error lines** (interleaved, as they occur): `E=<code>,<axis>,<detail>` / `ELOST=<n>` — see Section 7.

### 10.2 Arbitrary CAN parameter access (RxSdo/TxSdo)

ODrive's CANSimple protocol (v0.6.x+) supports generic parameter read/write on top of the fixed command set, via `RxSdo` (host→ODrive) and `TxSdo` (ODrive→host) messages at cmd_id `0x004`/`0x005`. This firmware implements the **write** side only (`Set_Param_Float`, Section 5.9):

```
Byte layout (8 bytes, fits one CAN frame):
  [0]    opcode      = SDO_OPCODE_WRITE (0x01)
  [1-2]  endpoint_id  (uint16, little-endian)
  [3]    reserved     = 0
  [4-7]  value        (float32, little-endian)
```

`endpoint_id` is **firmware-build-specific** — look it up in the `flat_endpoints.json` shipped with the ODrive's exact firmware release. **`odrive_config/flat_endpoints.json` in this repo is the fw v0.6.12 map, matching these 4 ODrives** — search it for any `axis0.controller.config.*` (or other) parameter name to get its numeric `endpoint_id`, then set it from the dashboard's "Advanced param (RxSdo)" box (mask + endpoint ID + value), not just `vel_ramp_rate`. Currently `vel_ramp_rate` (endpoint `398`) is the only one pushed automatically at boot (`25.0`). No TxSdo read-back is implemented — like every other CAN write in this firmware, it's fire-and-forget. Confirm a write actually landed via `odrivetool`/USB if certainty matters. Re-download/replace this file if any ODrive is ever reflashed to a different firmware version — endpoint IDs are not guaranteed stable across releases.

### 10.3 ODrive CAN commands defined but NOT implemented (reference for future work)

These are real ODrive CANSimple fixed-command-set IDs that have **no corresponding function in `ODrive.c`/`.h`** — i.e. calling them today requires writing the helper first. Listed here (migrated from the now-deleted `odrive_task_config.c` design-reference file) so the cmd_id allocation is documented in one place and isn't rediscovered from scratch later.

| cmd_id | Name | Frame type | Payload |
|---|---|---|---|
| `0x002` | `Estop` | 0-byte data | none — immediately disarms the axis |
| `0x003` | `Get_Error` | RTR | replies `[active_errors u32][disarm_reason u32]` |
| `0x011` | `Set_Traj_Vel_Limit` | data, 4B | `[traj_vel_limit f32]` |
| `0x012` | `Set_Traj_Accel_Limits` | data, 8B | `[accel_limit f32][decel_limit f32]` |
| `0x013` | `Set_Traj_Inertia` | data, 4B | `[inertia f32]` |
| `0x015` | `Get_Temperature` | RTR | replies `[FET_temp f32][Motor_temp f32]` |
| `0x019` | `Set_Absolute_Position` | data, 4B | `[position f32]` |
| `0x01C` | `Get_Torques` | RTR | replies `[torque_setpoint f32][torque_estimate f32]` |
| `0x01D` | `Get_Powers` | RTR | replies `[electrical_power f32][mechanical_power f32]` |

Note `RXSDO_CMD_ID` (`0x004`) and the would-be `TxSdo` response (`0x005`) are **already implemented** (write side only — Section 10.2) and are listed here in `ODrive.h`, unlike the rest of this table.

Implementing any of these follows the existing pattern in `ODrive.c` exactly: `Set_TX_Param(&msg->header, axis->NODE_ID, <cmd_id>, FDCAN_STANDARD_ID, <FDCAN_DATA_FRAME or FDCAN_REMOTE_FRAME>, <DLC>)`, pack any payload floats with `pack_f32`, call `HAL_FDCAN_AddMessageToTxFifoQ`. For the four `Get_*`/`Get_Error` RTR commands, the response also needs a new `case` in `HAL_FDCAN_RxFifo0Callback`'s/`ODrive_RX_CallBack`'s `switch(cmd_id)` (unpacking into new `Axis` struct fields — e.g. `AXIS_FET_Temperature`, `AXIS_Motor_Temperature`, `AXIS_Torque_Setpoint`, `AXIS_Torque_Estimate`, `AXIS_Electrical_Power`, `AXIS_Mechanical_Power`, `AXIS_Disarm_Reason` — none of which exist on `Axis` yet).

---

## Section 11 — On-MCU EKF: Design Detail

This section folds in the EKF design write-up (previously a standalone `ekf_addition.md` at the repo root — merged here since the EKF is now tested and in regular use; this is the persistent reference for it going forward).

### 11.1 Why the EKF moved on-MCU

Previously, two fusion stages ran in series: the firmware dead-reckoned `(x, y, phi)` from wheel kinematics alone (no IMU), then ROS's `robot_localization` re-fused that with the IMU into `/odometry/filtered`. That meant heading drift was only corrected after a round-trip through the ROS stage, and there was no single source of truth for pose until both stages had run.

Moving the EKF onto the MCU means: one source of truth available from boot (no waiting on a ROS node), a tighter fusion loop (wheel + IMU corrections at 50Hz with no UART/DDS hop between them), live covariance instead of static placeholders, and the robot keeps a coherent pose estimate even if the ROS stack goes down.

### 11.2 State, prediction, and correction

State vector (6 elements), updated every 20ms tick inside `ODrive_UpdateTelemetryAndOdometry`:

```
x = [ px, py, theta, vx_body, vy_body, omega ]^T
    (m)  (m)  (rad)   (m/s)    (m/s)    (rad/s)
```

`theta` wraps to `(-π, π]`. The EKF carries body-frame velocities (what mecanum kinematics gives natively) and owns the world↔body rotation internally via `theta`.

**Prediction** (constant-twist motion model), for `dt` seconds elapsed:

```
px_{k+1}      = px + dt * ( cos(theta)*vx_body - sin(theta)*vy_body )
py_{k+1}      = py + dt * ( sin(theta)*vx_body + cos(theta)*vy_body )
theta_{k+1}   = wrap( theta + dt * omega )
vx_body_{k+1} = vx_body   ← random-walk: "stays the same"
vy_body_{k+1} = vy_body
omega_{k+1}   = omega
```

Jacobian `F = ∂f/∂x` is computed analytically (`ekf.c::ekf_predict`); covariance update is `P ← F P F^T + Q·dt`.

**Correction 1 — wheel-derived body twist**: observation `z = (vx_body, vy_body, omega)` from `bodySpeedsFromUMecanum()` (Step 1 of mecanum FK only — no world rotation). `H` picks states 3,4,5 (identity on the last three rows). `R = diag(σ_vx², σ_vy², σ_omega²)`.

**Correction 2 — IMU yaw + omega_z**: observation `z = (yaw, omega_z)` from the BNO085, as two separate corrections back-to-back (`H=[0,0,1,0,0,0]` for yaw → state 2, `H=[0,0,0,0,0,1]` for omega → state 5), sharing the same `seq` argument so they advance the dedup counter together.

**Joseph form** for every correction:

```
S = H P H^T + R
K = P H^T S^-1
x ← x + K·(z - h(x))
P ← (I - K H) P (I - K H)^T + K R K^T   ← Joseph form (symmetric, numerically stable)
```

The naive form `P ← (I-KH)P` loses positive-definiteness after enough corrections due to float rounding — for an MCU running 24/7 without restart, that matters. The 6×6 inverse for `S^-1` is an in-place Gauss-Jordan solve (`gj_inverse`); it bails on `|pivot| < 1e-12` and the caller silently drops that correction rather than poisoning `P` with NaN.

### 11.3 Tuning — matched to `robot_localization`'s old omnibase config

The home2 omnibase launch used `process_noise_covariance`/`initial_estimate_covariance` defaults at `frequency: 30.0`. `robot_localization` adds `Q` once per cycle, so its per-second variance injection is `Q_rl · 30`. The on-MCU EKF uses a `Q·dt` convention (each predict step adds `q²·dt`), so integrated over one second: `q²·dt·rate = q²·(1/rate)·rate = q²` — **rate-independent**. Matching `r_l` means `q = sqrt(30 · Q_rl)`, true regardless of whether the firmware runs the EKF at 50Hz, 100Hz, or anything else.

| Knob | r_l variance @30Hz | EKF std-dev (Q·dt convention) | Notes |
|---|---|---|---|
| `q_x` / `q_y` | 0.05 m² | 1.2247 m·√s⁻¹ | matched |
| `q_theta` | 0.06 rad² | 1.3416 rad·√s⁻¹ | matched |
| `q_vx` / `q_vy` | 0.025 m²·s⁻² | 0.8660 m·s⁻¹·√s⁻¹ | matched |
| `q_omega` | 0.02 rad²·s⁻² | 0.7746 rad·s⁻¹·√s⁻¹ | matched |
| `r_imu_yaw` | message default | 0.03 rad (σ) | typical BNO085 short-term yaw |
| `r_imu_omega` | message default | 0.03 rad/s (σ) | typical BNO085 gyro |
| `r_wheel_vx`/`vy` | message default | 0.05 m/s (σ) | trusted |
| `r_wheel_omega` | not used (`config[11]=False`) | **1000 rad/s (σ²=1e6)** | deliberately ignored — see below |

**Why `r_wheel_omega` is huge**: mecanum rollers slip, so wheel-derived `omega` is systematically corrupted by roller slip + alignment error. `robot_localization` handled this by not fusing vyaw from odom at all (`odom0_config[11]=False`); the EKF gets the same effect by setting `σ²=10^6` on that observation, which collapses the Kalman gain on that row to ≈0 — the `omega` state ends up owned entirely by the BNO085.

**`imu_relative` (yaw zeroing at boot)**: mirrors `robot_localization`'s `imu0_relative: true`. On the first `ekf_correct_imu` call, the incoming yaw is latched as `imu_yaw_offset`; from then on the correction uses `(yaw - offset)`, so the robot boots at heading 0 regardless of absolute heading at power-on. The first sample produces a zero-innovation update (covariance tightens, mean doesn't move).

**IMU yaw outlier rejection**: after the `imu_relative` offset is applied, `ekf_correct_imu` rejects the correction if `|effective_yaw - x[2]| > 0.70 rad` (~40°) — ported from a check the home2 dashboard used to do at the UART-line level (dropping whole lines on a >40° jump between consecutive 50Hz samples, the "yaw flash" symptom of a corrupted SH2 packet). `last_imu_seq` still advances so a rejected sample isn't re-evaluated next tick.

### 11.4 IMU sequence gating and sign inversion

`g_bno085_seq` increments **last** inside `imu_sensor_data_cb` (after the yaw/quaternion/gyro globals are written), so a reader observing a fresh `seq` is guaranteed to see the corresponding fresh values, not a half-written previous sample. `ekf_correct_imu(ekf, yaw, omega_z, seq)` guards on `seq == e->last_imu_seq` — already-consumed samples return immediately, so a delayed/duplicate sample from a transient I2C glitch isn't applied twice.

The BNO085 on this robot reports yaw and gyro-z **inverted** relative to ROS `base_link` convention (rotating the robot +90° CCW reads roughly -90° raw) — rooted in `sh2/euler.c::q_to_ypr` using a non-ROS quaternion-to-yaw formula. Both yaw and gyro-z flip together since they share the same physical z-axis. This is now corrected **in firmware**, once, right before the EKF consumes it:

```c
const double yaw_imu   = -(double)g_bno085_yaw;
const double omega_imu = -(double)g_bno085_wz;
ekf_correct_imu(ekf, yaw_imu, omega_imu, seq);
```

The raw `g_bno085_yaw`/`g_bno085_wz` globals are left untouched (so the legacy `IMU_yaw`/`IMU_wz` UART fields stay backward-compatible with any consumer that has its own sign fix) — which is exactly why a downstream consumer that *also* negates these fields will double-invert. See Section 12 for why this matters for home2's `omnidriver`. A cleaner fix would rewrite `q_to_ypr` itself to use the ROS yaw formula, but that has wider blast radius (multiple consumers, mounting assumptions) — the one-line negation at the EKF input is surgical and easy to undo later.

`EKF_TRUST_IMU` (compile-time `#define`, currently `1`) sets `ekf.use_imu` at boot. Setting it to `0` makes `ekf_correct_imu` a no-op — wheel-only kinematic prediction + correction, functionally similar to the old dead-reckoning but with proper covariance accounting.

### 11.5 Telemetry bandwidth budget

Slim line ~220 chars at 50Hz, fat line ~920 chars at 10Hz ⇒ aggregate ≈ 20,200 chars/s ≈ 20kB/s at the UART symbol level (10-bit framing per byte):

| Baud rate | Capacity | Utilization | Verdict |
|---|---|---|---|
| 115,200 | 11,520 B/s | 175% | impossible |
| 230,400 | 23,040 B/s | 88% | tight — current setting, no burst headroom |
| 460,800 | 46,080 B/s | 44% | sustainable, comfortable |
| 921,600 | 92,160 B/s | 22% | luxurious |

Firmware currently runs at **230,400** (`MX_USART3_UART_Init`). At ~88% utilization there's no burst headroom — if more fields are added or slim cadence is raised, bump to 460,800 first.

### 11.6 Replaceability vs. `robot_localization` (for the omnibase config)

| `robot_localization` knob | STM32 EKF equivalent | Match |
|---|---|---|
| `frequency: 30.0` | telemetry tick at 50Hz | ✓ (faster is fine) |
| `sensor_timeout: 0.2` | predict-only without correction grows P | ✓ |
| `two_d_mode: True` | inherent — only 2D states tracked | ✓ |
| `publish_tf: True` | dashboard's `TransformBroadcaster` + `publish_tf` param | ✓ |
| `world_frame: odom` | `odom_frame_id` parameter (default `odom`) | ✓ |
| `odom0_config: [vx, vy]` | `ekf_correct_wheel_twist` with `r_wheel_omega=1000` | ✓ (omega effectively ignored) |
| `imu0_config: [yaw, vyaw]` | `ekf_correct_imu(yaw, omega_z)` | ✓ |
| `imu0_relative: True` | `EKF.imu_relative=1` (default) | ✓ |
| `process_noise_covariance` default | `default_params()` rescaled per Section 11.3 | ✓ |
| `initial_estimate_covariance` default (1e-9) | `init_cov_pos=0.10` etc. | slightly looser at boot, irrelevant after a few corrections |

What this does **not** replace: `map → odom` TF (still comes from SLAM — `slam_toolbox`/RTABMap, not the local EKF) and full 15-state semantics (`robot_localization` carries z/roll/pitch/accelerations; the EKF only tracks the 6 states an omnibase with `two_d_mode: True` was actually using anyway). Dashgo's separate EKF (differential drive, own IMU fusion config) is unaffected — only the omnibase launch path changes.

### 11.7 EKF known gaps (in addition to Known Limitations below)

- **Pitch/roll not estimated** — the EKF is planar by design. A mounting tilt on the BNO085 would manifest as systematic position drift. `OdomData` already carries the slots; extending the EKF to fuse gravity-aligned roll/pitch is mechanical, not done.
- **No IMU outlier rejection on omega_z specifically** — yaw is filtered (Section 11.3), gyro-z isn't. `r_imu_omega=0.03` is small, so a corrupted omega sample would update the filter cleanly. Symmetry would argue for a similar `|Δω| > ω_max_phys` guard.
- **No on-MCU gyro bias estimation** — `imu_relative` zeroes the initial offset but not the drift rate. Long-duration navigation without SLAM corrections would need a bias state (grows the filter to 7-8 states).
- **No fusion of commanded velocity** — `last_cmd.robot_twist` is observable and could serve as a third correction source (encoder-disagreement check). Recorded as a possibility, not implemented.
- **Initial covariance is loose** (`init_cov_pos=0.10` vs. `r_l`'s `1e-9` default) — means downstream consumers see "unknown pose" for the first few EKF ticks after boot. Irrelevant once a few corrections land.

---

## Section 12 — Pending: home2 `omnidriver` EKF Migration

The home-custom-base `odrive_comm` dashboard (this repo) is fully updated to consume the on-MCU EKF (Section 6, `odrive_dashboard.py`). The home2 `omnidriver` package is a **separate, downstream fork** of the same dashboard, hardened for production but **not yet updated** — this is still open work, tracked here (previously tracked in a standalone `ekf_addition.md`/`ekf_home2_update_prompt.txt`, merged into this doc since the EKF itself is tested and in use; only the home2 migration remains outstanding).

### 12.1 Required changes (~60 lines across `_parse_and_publish`, `_publish_imu_odom`, `__init__`, the parameter callback)

1. Bump `baud_rate` default from 230,400 to 460,800 or 921,600 (must match firmware's 230,400 *or* the firmware must be rebumped to match — baud is non-negotiable, host and firmware must agree).
2. Drop the manual world→body un-rotation block in both `_publish_imu_odom()` (slim path) and `_parse_and_publish()` (fat path) — the firmware now emits `ODOM_vxb`/`ODOM_vyb` directly. Keep `linear_scale_x`/`linear_scale_y` (empirical wheel-radius corrections, still apply).
3. Drop the `-math.radians(imu_y)` / `-imu_wz` yaw inversion in the `/odrive/imu` build path — the firmware now negates internally (Section 11.4). Leaving the dashboard's own inversion in while this firmware is flashed double-inverts the heading.
4. Prefer `ODOM_q*` for orientation when present in the parsed dict; fall back to `_yaw_to_quaternion(o_phi)` only for legacy firmware.
5. Prefer `ODOM_var_*` for covariance (with the `1e9` "unknown" convention on un-estimated slots); fall back to the existing tuned placeholder values for legacy firmware only.
6. Add a `TransformBroadcaster` + `publish_tf` parameter (default `True`); emit `odom → base_link` after the `nav_msgs/Odometry` publish in **both** the fast path and the fat path.
7. Verify `tf2_ros` is in `package.xml` (likely already present, for the `robot_localization` it's replacing).

### 12.2 Keep unchanged

The FAST PATH (`_publish_imu_odom`) for the slim line — it's a real perf win, just update its field set. The web-socket disconnect safety (zeroing commanded velocity). The IMU yaw outlier rejection in the parser (now redundant with the EKF's own rejection, Section 11.3, but harmless as defence-in-depth). The `linear_scale_x`/`linear_scale_y` calibration knobs.

### 12.3 Launch-file change

In `omni_basics.launch.py`: remove the `robot_localization` `Node(...)` entry. Remap downstream `/odometry/filtered` consumers (slam_toolbox/Nav2) to `/odrive/odom`, or rename the dashboard's publisher topic to `/odometry/filtered` instead — same outcome, pick the lower-blast-radius option for whatever launch structure exists at migration time. `imu_frame_id: 'base_link'` can stay as-is (skips a TF lookup). Do **not** touch the Dashgo path — it keeps its own `robot_localization` (different fusion config, unrelated base).

### 12.4 Staged rollout order (to avoid a double-filtering gap)

1. **Firmware first.** Flash the EKF firmware. `omnidriver`'s old inversion/un-rotation will double-correct at this point — don't bring up Nav2 yet.
2. **`omnidriver` second.** Apply Section 12.1's edits, rebuild. Now `/odrive/odom` is correct and TF flows from the dashboard; `robot_localization` is still running and double-filtering on top, but the two now agree — useful as a sanity check, not harmful.
3. **Drop `robot_localization`.** Remove the launch-file `Node`. Verify slam_toolbox/Nav2 still see what they need.
4. **(Optional)** Confirm the firmware's outlier rejection catches the "yaw flash" condition before removing the dashboard's now-redundant pre-filter.

### 12.5 Verification checklist

- `colcon build --packages-select omnidriver nav_main`
- `ros2 run tf2_ros tf2_echo odom base_link` → live transform at ~50Hz, quaternion changes when rotated by hand, exactly one publisher (no jitter between two filters).
- `ros2 node list | grep robot_localization` → empty for the omnibase.
- `ros2 topic echo /odrive/odom --once` → `pose.pose.orientation` is the EKF quaternion (not fixed identity); covariance diagonals are non-constant EKF values, not the legacy `0.02`/`0.08`/`0.05` placeholders; un-estimated slots carry `1e9`.
- Rotate the robot ~90° CCW by hand: yaw extracted from the quaternion should grow **positive**. If it grows negative, a sign-inversion was left in by mistake (Section 11.4).

---

## Section 13 — Troubleshooting

### 13.1 Stack size — I2C init errors or nothing works at boot

If I2C initialization errors show up or the system just doesn't come up at all, it might be because a task's stack filled up. This happened to me many times. Check the stack sizes in `freertos.c` and increase whichever task you think is the culprit.

---

### 13.2 ODrive stuck in INITIALIZING forever

One of the wheels gets stuck in `INITIALIZING` and won't come out no matter how many times you clear errors. Reasons still unknown.

Per the [ODrive docs](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error.INITIALIZING), this is expected briefly on power-up or after changing a config like `current_hard_max`. But if it stays like that for more than 1–3 seconds, it might be a hardware issue. The important thing to know: **you cannot clear this error by spamming `Clear_Errors`** — it won't go away while the axis is still initializing.

At RoboCup 2026 this happened and even connecting the ODrive to https://gui.odriverobotics.com/dashboard didn't help — couldn't clear the error from there either, and trying to reconfigure it from the GUI locked it up. Also tried: disconnecting the STM32 so it wouldn't spam CAN, connecting the ODrive alone without the battery to the motors, powering off and back on — it was still stuck in INITIALIZING. Then randomly it went away and everything worked. No idea why.

**Best solution found so far:** turn everything off, wait a few seconds (probably has something to do with the capacitors discharging), then power back on. That's it. It's the only thing that consistently worked.

---

## Known Limitations and Future Work

1. **No IMU staleness watchdog.** `g_bno085_seq` simply stops incrementing if the BNO085 goes dark (I2C fault, sensor hang); `ekf_correct_imu()` then silently no-ops forever and the EKF falls back to wheel-only dead-reckoning, with no error pushed and no dashboard indicator — unlike the ESP32/PC link rows, which do track age/status explicitly. Candidate fix: track `g_bno085_seq` staleness against a timeout in `ODriveTask` and push a new `FERR_IMU_STALE` code.
2. **No I2C bus-reset recovery.** Unchanged from the original BNO085 integration — a brownout mid-transaction still freezes IMU data until reboot.
3. **RxSdo writes are fire-and-forget.** No TxSdo read-back implemented (Section 10.2).
4. **`vApplicationStackOverflowHook` halts, doesn't recover.** By design — a corrupted stack can't safely run recovery code.
5. **home2 `omnidriver` dashboard not yet updated for this firmware.** See Section 12 for the ~60-line migration checklist (baud rate, drop manual world→body un-rotation, drop yaw inversion, prefer EKF quaternion/covariance, add TF). Don't flash this firmware against an un-updated `omnidriver` — yaw/velocity will double-invert and TF won't publish.
6. **BT-only watchdog is redundant.** Superseded by the general command watchdog; candidate for removal after further testing.
7. **`MutexUART_DataHandle` still unused.** Acceptable as long as the "only `UART_TX_Task` calls `printf()`" convention holds (Section 4.1) — revisit if that rule is ever broken.
8. **Recurring CAN bus-off / axis-fault / mass-arm-timeout observed in one test run.** Likely a wiring/termination/connector issue, possibly vibration-induced given the arm-vibration tuning context this firmware is being tested under — not a firmware bug, but flagged here since it would corrupt PID-tuning data collected during an affected run. Investigate hardware (connector seating, termination resistors) before trusting such a run's data.
