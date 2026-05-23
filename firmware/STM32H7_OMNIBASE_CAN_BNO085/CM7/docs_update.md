# STM32H7 OMNIBASE — Documentation Update

_Last updated: 2026-05-20_

The PDF in this folder (`Omnibase STM32 Docs.pdf`) is the public-facing
document. `omnibase_documentation.txt` is the deep technical reference.
This file lists gaps, errors, and additions found by comparing the two —
and now also captures the on-firmware EKF that fuses wheel odometry with
the BNO085 IMU to produce filtered odometry.

---

## 1. Critical errors to fix in the PDF

### 1.1 PINOUT — BNO055 → BNO085

The PDF lists `PB6 / PB7` as I²C lines for the BNO055. The project moved to
the **BNO085** months ago. Update the labels and add two new rows:

| Pin   | Function                                                     |
| ----- | ------------------------------------------------------------ |
| PB6   | I²C1 SCL — **BNO085**                                        |
| PB7   | I²C1 SDA — **BNO085**                                        |
| PD14  | BNO085 INT — active LOW, polled by `sh2_service()`           |
| PD15  | BNO085 RST — active LOW, pulsed for 10 ms by `hal_open()`    |

### 1.2 PINOUT — UART labels

The PDF mislabels USART3 as USART2. Correct mapping:

| Pin | Function                            |
| --- | ----------------------------------- |
| PD5 | USART2 TX (ESP32 / Bluetooth)       |
| PD6 | USART2 RX (ESP32 / Bluetooth)       |
| PD8 | USART3 TX (ROS computer)            |
| PD9 | USART3 RX (ROS computer)            |

Both run at **115 200 baud, 8N1**.

### 1.3 "Current State of Project" placeholder

Page 1 of the PDF still says `asdfasdfasdfasdf`. Replace with:

> Four-wheel mecanum-drive robot controller. STM32H755ZI (CM7 core) running
> FreeRTOS. Four ODrives on FDCAN1, BNO085 IMU on I²C1, ESP32 BT module on
> USART2, ROS computer on USART3. **Velocity control + 6-state EKF
> odometry operational at 100 Hz.** Two onboard fusion paths: mecanum
> wheel forward-kinematics and BNO085 yaw/yaw-rate. `ControlTask` is
> scaffolded but empty.

---

## 2. BNO085 — replace the PDF's BNO055 section entirely

### 2.1 Why the sensor was changed

- **BNO055**: single-register read, synchronous, ~1 ms per call.
- **BNO085**: SHTP (Sensor Hub Transport Protocol) over I²C, asynchronous.
  The sensor runs its own fusion and pushes quaternion / gyro / linear-accel
  packets at a configurable rate (currently 50 Hz, every 20 ms). The host
  drains them via `sh2_service()`; there is no "read the angle" register.

### 2.2 Why a dedicated IMU_Task is required

1. SHTP needs polling every few ms or packets are lost. `ODriveTask` can
   stall for >40 ms during startup / CAN bursts.
2. `sh2_open()` blocks ~310 ms (RST pulse + boot wait). Running it inside
   `ODriveTask` would trigger the ODrive CAN watchdog.
3. The SH2 library is not re-entrant — exactly one task may call it.
4. Decoupled rates: IMU at 50 Hz, CAN setpoints ~1 kHz. Shared volatile
   globals eliminate coupling.
5. Empirically, inline BNO085 reads (no dedicated task) caused the SHTP
   session to silently die within seconds.

### 2.3 IMU_Task startup sequence

1. `osDelay(500)` — avoids printf lock contention at boot.
2. `sh2_open()` → `hal_open()`: RST LOW 10 ms, RST HIGH, `osDelay(300)`.
3. `imu_service_ms(200)` — drains SHTP advertisement packets.
4. `sh2_setSensorCallback(imu_sensor_data_cb, NULL)`.
5. `imu_service_ms(100)` — drains startup control packets.
6. Three `sh2_setSensorConfig` calls at 20 000 µs (50 Hz):
   - `SH2_ROTATION_VECTOR`
   - `SH2_GYROSCOPE_CALIBRATED`
   - `SH2_LINEAR_ACCELERATION`
7. Main loop: `sh2_service()` + `osDelay(2)`, forever.

### 2.4 Data path: INT pin → callback → globals

```
PD14 LOW (every ~20 ms)
   ↓
sh2_service() → hal_read() (4-byte header, then full packet, addr 0x94)
   ↓
sh2_decodeSensorEvent() → imu_sensor_data_cb()
   ↓
volatile floats: g_bno085_{qx,qy,qz,qw,wx,wy,wz,ax,ay,az,yaw,pitch,roll}
volatile uint32_t g_bno085_seq   (incremented on every sample)
```

### 2.5 I²C hardware

- Peripheral: `I2C1`, clock 400 kHz, timing reg `0x00B03FDB`.
- Pins: PB6 (SCL), PB7 (SDA), 4.7 kΩ pull-ups to 3V3.
- BNO085 address: `0x4A` (7-bit) / `0x94` (8-bit write).
- INT: PD14 (input, pull-up, active-LOW, polled — not an MCU interrupt).
- RST: PD15 (output, idle HIGH, active-LOW).
- PS0 = PS1 = GND → selects I²C mode.

---

## 3. RTOS — full task table

| Task             | Priority           | Stack | Period / behaviour                |
| ---------------- | ------------------ | ----- | --------------------------------- |
| UART_RX_Task     | osPriorityHigh        | 4 KB  | `osDelay(5)` → ~200 Hz            |
| BT_RX_Task       | osPriorityHigh        | 4 KB  | `osDelay(5)` → ~200 Hz            |
| UART_TX_Task     | osPriorityNormal      | 4 KB  | blocks on `CAN_2_UTX_QueueHandle` |
| ODriveTask       | osPriorityAboveNormal | 4 KB  | `osDelay(1)`, telemetry @ 10 ms   |
| IMU_Task         | osPriorityAboveNormal | 4 KB  | `osDelay(2)` → ~500 Hz poll       |
| StartDefaultTask | osPriorityNormal      | 512 B | declared, never created           |
| ControlTask      | osPriorityAboveNormal | 1 KB  | created, body is empty            |

FreeRTOS heap: 32 768 B (heap_4). Tick: 1 kHz. HAL timebase: TIM6
(SysTick reserved for the scheduler).

### 3.1 UART_RX_Task

USART3 @ 115 200 baud, PD8=TX / PD9=RX (ROS link). Interrupt + 256-byte
ring buffer. Drains every 5 ms, parses on `\n` / `\r` via
`UART_RX_ParseLine()`.

### 3.2 BT_RX_Task

USART2 @ 115 200 baud, PD5=TX / PD6=RX (ESP32). Same ring-buffer pattern.
Parses `"3 <vx> <vy> <wz> <buttons_hex>"`. Dead-zone 0.05 m/s on linear
components and 0.05 rad/s on yaw rate. Button bit `0x04` is the
emergency-stop (`BT_ESTOP_BUTTON`).

### 3.3 ODriveTask — state machine

- **SM_BOOT**: waits up to 3 s for `CFG_STARTUP` or auto-starts:
  `Clear_Errors` → `Set_Controller_Modes(VELOCITY, PASSTHROUGH)` →
  `Set_Axis_Requested_State(CLOSED_LOOP)`.
- **SM_RUNNING**: every ~1 ms reads IMU globals, polls command queue,
  runs IK, sends CAN setpoints. Every 10 ms runs the EKF cycle (see
  Section 7) and pushes telemetry.
- **SM_IDLE**: zero-vel + IDLE state, waits for `CFG_STARTUP`.

BT priority override: once a BT command arrives, ROS is silenced for
500 ms (`BT_OVERRIDE_TIMEOUT_MS`).

**Note**: IMU yaw is not currently fed into IK (`phi=0.0` is hardcoded in
the IK call). The IMU **is** fused into the published odometry via the
EKF — but the controllers themselves still treat the body frame as the
world frame.

### 3.4 Safety watchdogs

`ODriveTask` carries two independent 500 ms timeouts. They are
idempotent — when both fire on the same disconnect the second zero-vel
SET_VEL is harmless.

#### a) Command-velocity watchdog (`CMD_WATCHDOG_TIMEOUT_MS`)

Locals at the top of `StartODriveTask`:

```c
const uint32_t CMD_WATCHDOG_TIMEOUT_MS = 500;
uint32_t last_vel_cmd_tick = osKernelGetTickCount();
uint8_t  cmd_watchdog_fired = 0;     // latch — clears on next SET_VEL
```

Refreshed by **any** incoming `ODRIVE_CMD_SET_VEL` regardless of source
(ROS or BT). If no `SET_VEL` arrives for 500 ms while in `SM_RUNNING`,
the firmware sends one zero-velocity command to all four axes
(`target_mask = 0x0F`), prints `CMD watchdog: no SET_VEL for 500 ms —
stopping motors` to USART3, latches, and also clears
`bt_override_active` so ROS can immediately reclaim control as soon as
packets resume.

The timestamp is also refreshed while `sm_state != SM_RUNNING`, so a
BOOT/STARTUP/IDLE → RUNNING transition never fires the watchdog
immediately.

Purpose: catch link failures that the BT-override path does not cover
(ROS node crashed, USB cable unplugged, host froze) when BT was never
active. Cost: one 32-bit subtract per loop iteration.

#### b) BT override release (`BT_OVERRIDE_TIMEOUT_MS`)

Older, narrower watchdog. Only relevant when BT had been actively
driving the robot (`bt_override_active == 1`). After 500 ms of BT
silence it sends a zero `SET_VEL` and clears the override flag so the
ROS path can resume.

Both timeouts share the same 500 ms horizon by design — the
host-side dashboard widget labels link state "LOST" at the same
threshold, see [§ 6.1](#61-link-status-monitoring-esp32_age_ms).

### 3.5 ODrive node-ID → wheel mapping

```c
odrives[0].NODE_ID = 36;  gear_ratio = 9;  wheel_sign = -1
odrives[1].NODE_ID = 34;  gear_ratio = 9;  wheel_sign = +1
odrives[2].NODE_ID = 33;  gear_ratio = 9;  wheel_sign = -1
odrives[3].NODE_ID = 40;  gear_ratio = 9;  wheel_sign = +1
```

Robot geometry (live values in `main.c`):
`x_offset = 0.195 m`, `y_offset = 0.195 m`, `wheel radius = 0.0762 m`.
(FL/BL/FR/BR physical labels still TBD — confirm on the bench.)

---

## 4. FreeRTOS queues and IPC

### 4.1 `URX_2_CAN_QueueHandle`
Slots: 3 × `ODriveCmdMsg`. Writers: UART_RX_Task (type-1, type-2),
BT_RX_Task (type-3). Reader: ODriveTask (2 ms timeout).

### 4.2 `UART_QueueHandle`
Slots: 3 × `ODriveCmdMsg`. Writer: UART_RX_Task (type-1 only).
Reader: UART_TX_Task (telemetry echo only).

### 4.3 `CAN_2_UTX_QueueHandle`
Slots: 3 × `ODriveTelemetryMsg` (now ~1.1 KB each — extended by the EKF
covariance fields). Writer: ODriveTask (every 10 ms). Reader:
UART_TX_Task (blocking). Full-queue policy: drop-oldest.

### 4.4 `MutexUART_DataHandle`
Created but never acquired. Intended as printf protection. Add to any new
task that prints frequently.

### 4.5 IMU data uses volatile globals

`g_bno085_*` are volatile `float`s — single-instruction atomic writes on
Cortex-M7. Worst-case skew: one component lags one sample (20 ms).
Acceptable for telemetry and the EKF. **A `volatile uint32_t g_bno085_seq`
is incremented on every SH2 callback** so the ODrive task can detect
fresh data and avoid double-counting it in the EKF.

---

## 5. UART command protocol (USART3 @ 115 200 baud)

### Type 1 — velocity command (from ROS)
```
1 <vx> <vy> <wz>\r\n
```
`vx`, `vy` in m/s, `wz` in rad/s. Drives all four axes.

### Type 2 — configuration (from ROS)
```
2 <sub> <mask_hex> [params]\r\n
```
| `<sub>` | Effect                                                        |
| ------- | ------------------------------------------------------------- |
| 20      | clear errors                                                  |
| 21      | set axis state (8 = CLOSED_LOOP)                              |
| 22      | set controller + input mode (2 = VEL, 1 = PASSTHROUGH)        |
| 23      | set velocity + current limits                                 |
| 24      | set position gain                                             |
| 25      | set velocity gain + integrator gain                           |
| 26      | full startup sequence (defaults: cm=2, im=1, st=8)            |
| 27      | reboot ODrive                                                 |
| 28      | set input torque                                              |
| 29      | stop (vel → 0 then IDLE)                                      |
| 30      | set input position with vel/torque feedforwards               |

### Type 3 — Bluetooth (from ESP32, USART2)
```
3 <vx> <vy> <wz> <buttons_hex>\r\n
```
Bits: `0x02` = BT toggle off + queue stop; `0x04` = emergency stop. Any
valid type-3 sets `BT_active=1`. BT has 500 ms override priority.

---

## 6. Telemetry output format (USART3 TX @ 100 Hz)

One CSV-style line per 10 ms. The full field list is now:

```
CMD_vx, CMD_vy, CMD_wz                      # last commanded robot twist
IMU_yaw, IMU_roll, IMU_pitch                # BNO085 Euler, DEGREES
IMU_qx, IMU_qy, IMU_qz, IMU_qw              # BNO085 quaternion (unit-norm)
IMU_wx, IMU_wy, IMU_wz                      # angular velocity, rad/s
IMU_ax, IMU_ay, IMU_az                      # linear acceleration, m/s^2
IK_u0..u3                                   # IK wheel speeds, rad/s
ODOM_phi, ODOM_x, ODOM_y, ODOM_z            # EKF pose (z always 0)
ODOM_qx, ODOM_qy, ODOM_qz, ODOM_qw          # pose quaternion (from yaw)
ODOM_w, ODOM_vx, ODOM_vy                    # world-frame velocities
ODOM_vxb, ODOM_vyb                          # body-frame linear velocities
ODOM_var_x, ODOM_var_y, ODOM_var_yaw        # pose covariance diagonal
ODOM_var_vx, ODOM_var_vy, ODOM_var_wz       # twist covariance diagonal
N0..N3, E0..E3, S0..S3, C0..C3              # ODrive node IDs, errors, axis state, controller status
P0..P3, V0..V3                              # position est (turns), velocity est (turns/s motor)
Sh0..Sh3, CPR0..CPR3                        # encoder shadow + CPR
Vbus0..Vbus3, Ibus0..Ibus3                  # bus voltage / current
IqSet0..IqSet3, IqMeas0..IqMeas3            # Iq setpoint / measured
U0..U3                                      # CAN-RX updated flag
BT_active, BT_vx, BT_vy, BT_wz              # Bluetooth override state
ESP32_age_ms                                # ms since last valid BT packet (-1 = never)
```

The firmware-side `OdomData` struct holds the full ROS-style row-major
6×6 `pose_covariance` and `twist_covariance` arrays (see Section 7.6). The
UART line carries only the diagonal entries to stay within bandwidth; the
remaining covariance cells are available in `OdomData` if a downstream
consumer ever switches to a binary transport.

### 6.1 Link-status monitoring (`ESP32_age_ms`)

`ESP32_age_ms` is the age, in milliseconds, of the last successfully
parsed Type-3 line from the ESP32 (USART2). The data path:

```
BT_RX_Task : on a clean sscanf (parsed >= 4)
             g_bt_last_valid_msg_tick = HAL_GetTick();
UART_TX_Task: snapshot once per telemetry packet,
              esp32_age_ms = (long)(HAL_GetTick() - snap);
              ESP32_age_ms = -1 when snap == UINT32_MAX (never received).
```

A heartbeat in PAIRED / INACTIVE state counts as "valid" — the field
tracks **link liveness**, not driving activity. The 1 kHz `HAL_GetTick()`
wraps every ~49.7 days; unsigned subtraction handles wrap-around
transparently. The `UINT32_MAX` sentinel only collides with a real tick
~49.7 days into a session, at which point the "first-message" check is
moot anyway.

The host (`odrive_dashboard.py`) interprets `ESP32_age_ms` for the
ESP32 → STM32 leg and uses its own `time.monotonic()` for the
STM32 → PC leg. The two values appear side-by-side in the dashboard
"Links" card with the following thresholds:

| Bucket | Range            | Pill                          |
| ------ | ---------------- | ----------------------------- |
| OK     | age < 500 ms     | green (`.pill.run`)           |
| WARN   | 500 ms ≤ age < 2 s | amber (inline style)        |
| LOST   | age ≥ 2 s        | red (`.pill.err`)             |
| UNKNOWN| firmware sentinel `-1` or missing field | muted          |

Rationale for the thresholds:

- STM32 → PC: `UART_TX_Task` emits at ~100 Hz. 500 ms = 50 missed
  packets — unambiguous degradation.
- ESP32 → STM32: BT streams Type-3 packets at ~50 Hz when ACTIVE and
  heartbeats well above 2 Hz when PAIRED. 500 ms covers both.
- 2 s LOST matches the BT-override + command-velocity watchdogs in
  [§ 3.4](#34-safety-watchdogs), so the dashboard flips to LOST at the
  same moment the firmware itself stops trusting the link.

**During an STM32 disconnect**, the firmware can't update its own
`ESP32_age_ms`, so the dashboard freezes that value at the last
reported number. The PC must **not** extrapolate it — the STM32 → PC
LOST pill is the authoritative "you can't trust the ESP32 number any
more" signal. Dashboard widget details live in
`omnibase_ws/src/odrive_comm/README.md`.

---

## 7. EKF — 6-state mecanum odometry filter (NEW)

The firmware now runs an Extended Kalman Filter on-board that fuses wheel
encoders with the BNO085 IMU and replaces the previous raw dead-reckoning.
The result is published in the same telemetry stream (`ODOM_*` fields).

**Files**: [`Core/Inc/ekf.h`](Core/Inc/ekf.h), [`Core/Src/ekf.c`](Core/Src/ekf.c).
**Origin**: ported in plain C from the ROS 2 node
`omnibase_ws/src/ekf_optimized/src/ekf_node.cpp`, with the state vector
expanded from 5 to 6 to expose mecanum lateral velocity.

### 7.1 State vector

The filter carries six scalars:

```
x  = [ x, y, theta, vx, vy, omega ]^T
```

| Symbol  | Meaning                       | Units |
| ------- | ----------------------------- | ----- |
| `x`     | position, odom frame, x axis  | m     |
| `y`     | position, odom frame, y axis  | m     |
| `theta` | yaw heading (wrapped ±π)      | rad   |
| `vx`    | linear velocity, **body** x   | m/s   |
| `vy`    | linear velocity, **body** y   | m/s   |
| `omega` | yaw rate (body = world)       | rad/s |

Six states (not the C++ node's five) so that a mecanum chassis can carry
a non-zero lateral body velocity. Z, roll, pitch are not estimated —
the base is planar.

### 7.2 Inputs

| Source                              | What the EKF takes                                |
| ----------------------------------- | ------------------------------------------------- |
| Wheel encoders (via 4 ODrives, CAN) | `u[0..3]` → mecanum FK → `(vx_b, vy_b, omega_b)`  |
| BNO085 `SH2_ROTATION_VECTOR`        | yaw extracted from quaternion                     |
| BNO085 `SH2_GYROSCOPE_CALIBRATED.z` | yaw rate `wz`                                     |
| BNO085 `SH2_LINEAR_ACCELERATION`    | `(ax, ay)` body frame (optional, see Section 7.5) |

The body-frame wheel-FK is in `bodySpeedsFromUMecanum()`:

```
v_body[0] = (r/4)   * ( u0 + u1 + u2 + u3)             // vx_body
v_body[1] = (r/4)   * (-u0 + u1 + u2 - u3)             // vy_body
v_body[2] = (r/(4L))* (-u0 + u1 - u2 + u3),  L = x_off+y_off  // omega_body
```

### 7.3 Predict step

Called once per telemetry tick (10 ms) inside
`ODrive_UpdateTelemetryAndOdometry`. Motion model (with optional linear
acceleration, `ax`/`ay` body-frame):

```
x'     = x + (cos θ · vx − sin θ · vy) · dt
            (+ 0.5 (cos θ · ax − sin θ · ay) · dt²)
y'     = y + (sin θ · vx + cos θ · vy) · dt
            (+ 0.5 (sin θ · ax + cos θ · ay) · dt²)
θ'     = wrap(θ + ω · dt)
vx'    = vx     (+ ax · dt)
vy'    = vy     (+ ay · dt)
ω'     = ω
```

The acceleration terms are only applied when
`ekf_params.use_imu_acceleration == true` (off by default — see Section 7.5).

Jacobian `F = ∂f/∂x` (6 × 6, identity except for the cross terms):

```
F[0][2] = −sin θ · vx · dt − cos θ · vy · dt
F[0][3] =  cos θ · dt
F[0][4] = −sin θ · dt
F[1][2] =  cos θ · vx · dt − sin θ · vy · dt
F[1][3] =  sin θ · dt
F[1][4] =  cos θ · dt
F[2][5] =  dt
```

Plus the analogous accel terms in row 0/1 column 2 when acceleration is on.

Covariance propagation:

```
P  ←  F · P · Fᵀ  +  Q · dt
```

with `Q = diag(q_x, q_y, q_θ, q_vx, q_vy, q_ω)` (tunable in
`EKFParams.process_noise_*`).

### 7.4 Correction steps

Each tick runs three corrections in order, only if their input passes
sanity gates (`isfinite`, value bounded by `max_odom_*` / `max_imu_yaw_jump`).
All corrections share a single Joseph-form update:

```
y      =  z − H · x                                 # innovation (angles wrapped)
S      =  H · P · Hᵀ + R
K      =  P · Hᵀ · S⁻¹
x      ←  x + K · y                                 (θ re-wrapped to ±π)
P      ←  (I − K H) · P · (I − K H)ᵀ + K · R · Kᵀ
```

The Joseph form is used (not the Kalman shortcut `P ← (I−KH) P`) to keep
`P` symmetric and positive-semidefinite under numerical noise.

**S⁻¹** is computed by in-place Gauss-Jordan with partial pivoting — `m`
is at most 3 (the odom-twist correction), so this is cheap and stable.

#### a) Wheel-FK twist correction (m = 3)

```
z  = [ vx_body_meas , vy_body_meas , omega_meas ]ᵀ
H  = [ 0 0 0 1 0 0
       0 0 0 0 1 0
       0 0 0 0 0 1 ]
R  = diag(σ²_vx, σ²_vy, σ²_ω)        # fallback variances from params
```

#### b) IMU yaw correction (m = 1)

The BNO085 quaternion → yaw conversion:

```
yaw_imu = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))
```

If `zero_imu_yaw_on_boot` is set (default `true`) the first valid
`yaw_imu` is captured as `yaw_boot` and all subsequent corrections are
applied relative to it — i.e. the EKF treats "BNO085 yaw at boot" as the
zero-of-yaw of the odom frame. This avoids a 1-time pose jump when the
IMU finishes its self-calibration.

```
z  = [ yaw_imu ]
H  = [ 0 0 1 0 0 0 ]
R  = σ²_yaw
```
Innovation is wrapped to ±π; correction is **skipped** if
`|innovation| > max_imu_yaw_jump` (default 1.0 rad) — that gate kills
single-sample glitches from the sensor without poisoning the filter.

#### c) IMU yaw-rate correction (m = 1)

```
z  = [ g_bno085_wz ]
H  = [ 0 0 0 0 0 1 ]
R  = σ²_ωz
```

Corrections (b) and (c) only fire when `g_bno085_seq` (incremented in the
SH2 callback on every new sample) has advanced since the last cycle, so
a single IMU sample is never injected twice — important because the EKF
runs at 100 Hz and the BNO085 only emits at 50 Hz.

### 7.5 Tuning knobs (defaults in `ekf_params_defaults`)

| Group              | Defaults                                                       |
| ------------------ | -------------------------------------------------------------- |
| sample time        | `0.010` s                                                      |
| process noise      | `[x,y,θ]=0.02`; `[vx,vy,ω]=0.20` (variance per second)         |
| odom meas variance | `[vx,vy,ω]=0.05`                                               |
| IMU meas variance  | `yaw=0.03`, `ωz=0.03`                                          |
| initial cov diag   | `[x,y,θ]=0.10`; `[vx,vy,ω]=0.20`                               |
| outlier gates      | `max_imu_yaw_jump=1.0 rad`, `max_odom_v*=3 m/s`, `max_ω=6 rad/s`|
| signs              | `imu_yaw_sign=+1`, `imu_omega_sign=+1`, `imu_ax/ay_sign=+1`    |
| flags              | `use_imu_yaw=use_imu_omega=true`; `use_imu_acceleration=false` |

`use_imu_acceleration` is off by default because the BNO085 reports
**linear** acceleration with gravity already removed in its **own** frame.
Confirm the body-frame axis convention against the chassis before
turning it on — otherwise the predict step will fight the wheel-FK
measurement.

### 7.6 Output packaging

After the predict + 3-corrections cycle, the filter state is copied into
`OdomData` (`Core/Inc/main.h`). The struct is now ROS-`nav_msgs/Odometry`-shaped:

```c
typedef struct {
    /* Legacy — kept for the existing UART/dashboard parser. */
    double x_pos, y_pos, phi;
    double q_dot[3];     // world-frame [phi_dot, x_dot, y_dot]

    /* Planar pose. */
    double z_pos;        // always 0
    float  qx, qy, qz, qw;   // from EKF yaw (qx = qy = 0)

    /* Body-frame twist. */
    double vx_body, vy_body, vz_body;     // vz_body always 0
    double wx, wy, wz;                    // wx = wy = 0; wz from EKF

    /* ROS row-major 6×6 covariance.
     *   pose axes  : (x, y, z, roll, pitch, yaw)
     *   twist axes : (vx, vy, vz, wx, wy, wz)
     * Unobservable planar-base axes carry 1e6 as a "no info" sentinel. */
    double pose_covariance[36];
    double twist_covariance[36];
} OdomData;
```

The filter's 3×3 pose covariance (`x`, `y`, `θ` block of `P`) is mapped
into the ROS axes `0, 1, 5` of `pose_covariance`; the twist 3×3
(`vx`, `vy`, `ω` block) maps into `0, 1, 5` of `twist_covariance`. Z,
roll, pitch and the vz, wx, wy diagonals carry `1e6` so any consumer
treating the planar base as 3D knows those dimensions are unknown.

The full struct goes into `ODriveTelemetryMsg.odom` and is pushed every
10 ms to `UART_TX_Task` via `CAN_2_UTX_QueueHandle`. The UART writer
prints the new fields described in Section 6.

### 7.7 Pose reset

`ekf_reset_pose(&ekf, x, y, theta)` is available for wiring a future
dashboard "zero pose" command. It re-seeds position and yaw, clears
velocities, and resets the 3×3 pose covariance block to the initial-cov
parameters. Not currently exposed over UART — drop a new sub-type into
`UART_RX_ParseLine()` if/when it's needed.

---

## 8. Kinematics — typed equations

Replace the "NOT YET TESTED" notebook photo in the PDF.

### 8.1 Inverse Kinematics — `computeNecessaryWheelSpeedsMecanum`

Inputs: `vx = x_dot`, `vy = y_dot`, `wz = phi_dot`, `phi` heading
(currently always 0.0 — IMU not yet plumbed into IK).

```
u0 = ( vx(cos φ + sin φ) − vy(cos φ − sin φ) − wz·L ) / r
u1 = ( vx(cos φ − sin φ) + vy(cos φ + sin φ) + wz·L ) / r
u2 = ( vx(cos φ − sin φ) + vy(cos φ + sin φ) − wz·L ) / r
u3 = ( vx(cos φ + sin φ) − vy(cos φ − sin φ) + wz·L ) / r
                                            with  L = x_off + y_off
```

ODrive velocity (motor turns/s):
`odrive_vel[i] = wheel_sign[i] · u[i] · gear_ratio / (2π)`.

### 8.2 Forward Kinematics

Two variants now live in `main.c`:

- `bodySpeedsFromUMecanum(x_off, y_off, r, u[4], v_body[3])` — body
  frame. Used by the EKF as the wheel-FK measurement.
- `globalSpeedsFromUMecanum(phi, x_off, y_off, r, u[4], q_dot[3])` —
  world frame. Kept for backwards compatibility; not called by the
  active code path (the EKF carries its own `theta`).

### 8.3 Odometry integration

Replaced by the EKF. The old raw `x += q_dot[1]·dt` integration is gone;
`ODrive_UpdateTelemetryAndOdometry` calls `ekf_predict` /
`ekf_correct_*` and copies the filter state into `OdomData`.

---

## 9. ODrive CAN protocol — message encoding

```
CAN ID  =  (NODE_ID << 5) | COMMAND_ID            // 11-bit standard
```

Example — `Set_Input_Vel` (CMD `0x00D`):

```
ID       :  (node_id << 5) | 0x00D
Bytes 0-3:  velocity (float32, little-endian)
Bytes 4-7:  torque feedforward (float32, little-endian)
```

TX safety: `FDCAN_WAIT_TX_FREE()` polls TX FIFO level with a 50 ms hard
timeout. A missing ODrive logs `CAN TX timeout` instead of blocking
indefinitely.

RX filter: accepts all standard IDs. `HAL_FDCAN_RxFifo0Callback()` routes
each frame to the right `Axis` struct via `node_id = id >> 5`.

---

## 10. Source-file inventory

```
Core/Src/main.c            All tasks, ODrive helpers, kinematics,
                           BNO085 SH2 callbacks, peripheral init
Core/Src/ODrive.c          Low-level CAN-TX wrappers for the ODrive protocol
Core/Inc/main.h            Project-level structs and enums
Core/Src/ekf.c             6-state EKF implementation (NEW)
Core/Inc/ekf.h             EKF API (NEW)
Core/Src/sh2_hal_impl.c    BNO085 HAL: open/close/read/write/getTimeUs
Core/Src/sh2/*             CEVA SH2 protocol library — do not modify
Core/Src/sh2/euler.c       Quaternion → Euler (q_to_ypr)
Core/Src/myprintf.c        printf retarget to USART3 (polling, not thread-safe)
Core/Src/stm32h7xx_hal_timebase_tim.c   TIM6 as HAL 1 ms timebase
odrive_task_config.c       DESIGN REFERENCE, not compiled

Legacy / unused but still compiled:
  Core/Src/bno055.c        Old IMU driver, never called
  Core/Src/mcp2515.c       Old SPI CAN driver, never called
```

---

## 11. Known limitations and future work

1. **`printf()` is not mutex-protected.** `MutexUART_DataHandle` exists
   but is never acquired. Wrap any new task's `printf` with it.
2. **IMU yaw is not fed into IK.** `phi=0.0` is hardcoded in the IK
   call. Passing `g_bno085_yaw` would enable field-oriented control.
   The EKF *does* consume IMU yaw, so the *odometry* is corrected; only
   the IK is still body-frame-only.
3. **`ControlTask` is empty.** Scaffolded at `osPriorityAboveNormal`
   but body is just `osDelay(10)`. A heading/path controller goes here.
4. **No I²C bus-lockup recovery.** Brownouts mid-transaction freeze
   IMU data. Add SCL toggling + `HAL_I2C_Init` recovery.
5. **No stack-overflow detection.** Set
   `configCHECK_FOR_STACK_OVERFLOW` to 2 in `FreeRTOSConfig.h` during
   development.
6. **Old IMU/CAN drivers still compile.** `bno055.c` + `mcp2515.c`
   waste ~2–4 KB of flash. Drop them from the build when convenient.
7. **`StartDefaultTask` declared but never created.** Body is an old
   CAN smoke test with hard-coded node IDs that collide with prod IDs.
8. **BT watchdog commented out.** The 1 s diagnostic in `BT_RX_Task` is
   disabled. Re-enable for production.
9. **EKF `use_imu_acceleration` defaults off.** Sign convention of the
   BNO085 linear-accel output vs. the chassis body frame must be
   verified before enabling, or the predict step will diverge.

PDF To-Do items mapped to firmware actions:

- _"Poner comentarios con qué ID es cada motor FL/BL/FR/BR"_ → see
  Section 3.5. Bench-confirm wheel positions and add comments in
  `ODriveTask`.
- _"Hacer que BT_active sea 1 si conectado y 2 si activo"_ → currently
  `BT_active` is set to 1 on any valid type-3 and cleared by the toggle
  button. Add a separate "connected" (1) vs. "active/override" (2)
  state and surface it.
- _"Hacer que la ESP mande un heartbeat"_ → ESP32-side change; MCU
  would need a new message type and a connection-state flag.

---

## 12. Minor PDF updates

### 12.1 Tutorial

- Add baud rate (115 200), port (USART3 via USB), and the command
  protocol so the first test command works without reading the rest of
  the doc.
- Add: ODrives must be powered at 12 V before the STM32 boots, otherwise
  the startup sequence fails with `CAN TX timeout`.
- Cross-reference `IMPORTANTE: POLARIDAD NO INTERCAMBIABLE` from the
  ODrive section.

### 12.2 ODrive configuration

- Four ODrives must be configured with node IDs `33, 34, 36, 40`
  (not all the same).
- After calibration each ODrive saves to NVM. On power-up they start
  in IDLE; the STM32 firmware sends `Set_Axis_Requested_State =
  CLOSED_LOOP_CONTROL (8)`.

### 12.3 FDCAN section

- Status: FDCAN with TJA1051 transceiver works at 500 kbps in classic
  CAN mode. Remove the "PENDIENTE PROBAR" label.
- Auto-retransmission is **disabled intentionally**. Missed frames
  surface via `CAN TX timeout` instead.

### 12.4 ROS 2 section

- Replace screenshots with the serial interface contract: USART3,
  115 200 baud, text CSV. The ROS node subscribes to the telemetry
  CSV and publishes type-1 / type-2 lines.
- Reference path: `omnibase_ws/src/odrive_comm`.

---

## 13. Status snapshot (2026-05-20)

- **Done**: BNO085 migration; FDCAN at 500 kbps; ROS USART3 link;
  ESP32 BT USART2 link with override + estop; per-axis CAN watchdog;
  pose-rest / safety stop on command-timeout; **6-state EKF odometry
  fusing wheel encoders + IMU yaw / yaw-rate, with ROS-shaped
  covariance output**.
- **In progress / scaffolded**: `ControlTask` body, heading-aware IK,
  ESP32 heartbeat.
- **Open issues**: `printf` mutex, stack-overflow trap, I²C bus reset,
  drop unused legacy drivers, EKF acceleration channel verification.
