# On-MCU Extended Kalman Filter — STM32H7 OMNIBASE firmware

> Author: implementation pass on `home-custom-base @ merged_major_update`.
> Scope: the firmware EKF that replaces home2's `robot_localization` node for
> the omnibase, plus the dashboard side that turns its output into
> `nav_msgs/Odometry` + `odom → base_link` TF, plus the punch list of what
> the home2 `omnidriver` package needs to consume it cleanly.

---

## 1. What changed and why

### Before

Two stages of fusion ran in series:

1. **Firmware (STM32H7)** — pure dead-reckoning. Mecanum forward kinematics
   turned the four ODrive wheel velocities into a body-frame twist, rotated
   it into the world frame by the current heading, and Euler-integrated to
   produce `(x, y, phi)`. This raw estimate was published over UART as
   `ODOM_*` fields.

2. **ROS (`robot_localization` on home2)** — a 15-state EKF fused
   `/odrive/odom` (wheel twist) with `/odrive/imu` (BNO085 yaw + ω_z) into
   `/odometry/filtered` and published the `odom → base_link` TF.

That stack works but stacks one filter on top of another integration: the
firmware integrates yaw with no IMU input, then ROS re-fuses everything.
Drifting heading is corrected only after a 30 Hz round-trip through the
ROS stage, and there's no single source of truth for *where the robot is
right now* until both stages have spoken.

### After

The EKF moves onto the MCU. The firmware's `ODrive_UpdateTelemetryAndOdometry`
runs a 6-state EKF every 20 ms (50 Hz). It fuses:

- **Wheel-derived body twist** at every tick (Step 1 of mecanum FK only —
  no world-rotation by `phi`).
- **BNO085 yaw + ω_z** whenever a fresh sample is available (gated by a
  sequence counter so each IMU report fires at most one correction).

It publishes a fully-filtered `nav_msgs/Odometry`-shaped payload over UART:
pose with quaternion, body- and world-frame twist, and the 6×6 covariance
matrices that downstream consumers (Nav2, slam_toolbox, RTABMap) expect.

The dashboard then puts that on `/odrive/odom` directly and broadcasts the
`odom → base_link` TF. `robot_localization` becomes unnecessary.

### Why

- **One source of truth** for pose, available at boot, on the same wire
  that already carries telemetry. No need to wait for a ROS node to come up
  before the robot has a heading.
- **Tighter loop**. Wheel + IMU corrections fire at 50 Hz with no UART or
  CycloneDDS hop between them. Yaw drift is bounded by the BNO085's own
  noise, not by how fast `robot_localization` happens to be running.
- **Live covariance**. Instead of the static placeholders the dashboard
  was emitting, downstream consumers see the EKF's actual posterior
  variance — useful for the slam_toolbox / Nav2 cost-weighting machinery.
- **Robustness**. The firmware EKF survives the ROS stack going down
  (kill a container, switch a launch file) — the robot keeps a coherent
  pose estimate in case someone else picks it up.

---

## 2. State, model, and equations

State vector (6 elements):

```
x = [ px,  py,  theta,  vx_body,  vy_body,  omega ]^T
    (m)  (m)   (rad)    (m/s)     (m/s)     (rad/s)
```

`theta` is wrapped to `(-π, π]`. The body-frame velocities are what
mecanum kinematics give you natively; the EKF carries `theta` and rotates
the prediction into world frame internally, which means the EKF
itself owns the world↔body conversion.

### Prediction (constant-twist motion model)

For `dt` seconds elapsed:

```
px_{k+1}     = px + dt * ( cos(theta) * vx_body - sin(theta) * vy_body )
py_{k+1}     = py + dt * ( sin(theta) * vx_body + cos(theta) * vy_body )
theta_{k+1}  = wrap( theta + dt * omega )
vx_body_{k+1} = vx_body   ← random-walk: model is "stays the same"
vy_body_{k+1} = vy_body
omega_{k+1}  = omega
```

The Jacobian `F = ∂f/∂x` evaluated at the prior mean is computed
analytically — see `ekf.c::ekf_predict`. The covariance update is
`P ← F P F^T + Q · dt`, with `Q` from `EKFParams`.

### Correction — wheel-derived body twist

Observation: `z = (vx_body_meas, vy_body_meas, omega_meas)` from
`bodySpeedsFromUMecanum()` (just Step 1 of the existing mecanum FK).

`H` picks states 3, 4, 5: identity on the last three rows of the state
vector. `R` is a 3×3 diagonal: `diag(σ_vx², σ_vy², σ_omega²)`.

### Correction — IMU yaw + ω_z

Observation: `z = (yaw_BNO085, omega_z_BNO085)`. Two separate corrections
back-to-back, one with `H = [0,0,1,0,0,0]` (yaw → state 2) and one with
`H = [0,0,0,0,0,1]` (omega → state 5).

The yaw correction's innovation is angle-wrapped. Both corrections share
the same `seq` argument so they advance the sequence counter together
(otherwise yaw correction would lock the counter and ω_z would always
return "already consumed").

### Joseph form

All corrections use:

```
S = H P H^T + R
K = P H^T S^-1
x ← x + K · (z - h(x))
P ← (I - K H) P (I - K H)^T + K R K^T   ← Joseph form
```

The Joseph form is symmetric and numerically stable — the naive form
(`P ← (I - K H) P`) loses positive-definiteness after enough corrections
because of float rounding, and for an MCU running 24/7 without restart
that matters. See `ekf.c::correct`.

The 6×6 inverse needed for `S^-1` is a Gauss-Jordan in-place solve
(`gj_inverse`) — small enough that LU-with-pivoting isn't needed. The
solver bails on `|pivot| < 1e-12` and the caller silently drops the
correction (rather than poisoning `P` with NaN).

---

## 3. Tuning — matched to `robot_localization`'s omnibase config

The home2 omnibase launch (`omni_basics.launch.py`) uses these inline:

```python
'odom0_config': [F,F,F, F,F,F,  T,T,F,  F,F,F,  F,F,F]   # vx, vy
'imu0_config':  [F,F,F, F,F,T,  F,F,F,  F,F,T,  F,F,F]   # yaw, vyaw
'imu0_relative': True
'two_d_mode':    True
'publish_tf':    True
'frequency':     30.0
```

It does NOT override `process_noise_covariance` or
`initial_estimate_covariance`, so `robot_localization` falls back to its
built-in defaults. The EKF's `default_params()` in `ekf.c` mirrors this
behaviour exactly:

| Knob | r_l per-cycle variance @30 Hz | EKF std-dev (Q·dt convention) | Notes |
|---|---|---|---|
| `q_x` / `q_y` | 0.05 m² | 1.2247 m·√s⁻¹ | matched: same per-second injection |
| `q_theta`      | 0.06 rad² | 1.3416 rad·√s⁻¹ | matched |
| `q_vx` / `q_vy`| 0.025 m²·s⁻² | 0.8660 m·s⁻¹·√s⁻¹ | matched |
| `q_omega`     | 0.02 rad²·s⁻² | 0.7746 rad·s⁻¹·√s⁻¹ | matched |
| `r_imu_yaw`   | from message default | 0.03 rad (σ) | typical BNO085 short-term yaw |
| `r_imu_omega` | from message default | 0.03 rad/s (σ) | typical BNO085 gyro |
| `r_wheel_vx`  | from message default | 0.05 m/s (σ) | trusted |
| `r_wheel_vy`  | from message default | 0.05 m/s (σ) | trusted |
| `r_wheel_omega` | not used (config[11]=False) | **1000 rad/s (σ²=1e6)** | ignored — see below |

Why the per-second mapping — and why it is **rate-independent**:
`robot_localization` adds `Q` once per cycle at 30 Hz, so its per-second
variance injection is `Q_rl · 30`. The on-MCU EKF uses a `Q · dt`
convention: each predict step adds `q² · dt` to the covariance diagonal.
Integrated over one second the total is

```
sum_per_second = q² · dt · rate = q² · (1/rate) · rate = q²
```

so the per-second injection equals `q²` regardless of how fast the EKF
runs (50 Hz, 100 Hz, anything). Matching r_l therefore means
`q² = 30 · Q_rl`, i.e. `q = sqrt(30 · Q_rl)`. Hence the `1.2247 = sqrt(30
· 0.05)` numbers above. The constants are correct at 50 Hz exactly as
they were at 100 Hz — no rescaling needed when the firmware cycle changes.

### Why `r_wheel_omega = 1000` (almost-infinite noise)

Mecanum wheels slip — they're rollers, not wheels in the diff-drive sense.
Body-frame ω derived from wheel encoders is *systematically* corrupted by
roller slip plus mechanical alignment error. `robot_localization` handles
this on home2 by setting `odom0_config[11] = False` (don't fuse vyaw from
odom at all). The EKF achieves the same effect with a huge noise: with
`σ² = 10^6`, the Kalman gain on that observation row collapses to ≈ 0
and the wheel-derived ω has no influence on the filter's `omega` state —
that state is owned by the BNO085 alone.

### `imu_relative: true` — yaw zeroing at boot

`robot_localization`'s `imu0_relative: true` latches the first IMU yaw
sample as an offset and reports `(yaw - offset)` from then on, so the
robot boots at heading 0 regardless of its absolute heading at power-on.

The EKF mirrors this with `EKF.imu_relative = 1` (default). On the first
`ekf_correct_imu` call, `imu_yaw_offset` is set to the incoming yaw and
`imu_yaw_initialized` flips to 1. From then on the correction uses
`(yaw_sensor - imu_yaw_offset)`. The very first sample produces a
zero-innovation update — covariance still tightens but the mean doesn't
move.

### IMU yaw outlier rejection

The home2 omnidriver dashboard used to drop entire telemetry lines if
`|Δyaw|` between consecutive 50 Hz samples exceeded 40° — that's the
"yaw flash" symptom from a corrupted SH2 packet or a transient I2C
glitch. Ported into the EKF: after the `imu_relative` offset is applied,
`ekf_correct_imu` compares `effective_yaw` against the EKF's posterior
`x[2]` and rejects the correction if `|Δ| > 0.70 rad` (about 40°). The
`last_imu_seq` counter is still advanced so the same bad sample isn't
re-evaluated on the next tick.

---

## 4. Firmware integration

### Files

- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/ekf.h` — public API.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Src/ekf.c` — implementation.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/main.h` —
  `OdomData` extended with `z_pos`, `qx..qw`, body twist, 6×6
  `pose_covariance[36]` and `twist_covariance[36]`.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/FreeRTOSConfig.h` —
  `configTOTAL_HEAP_SIZE` bumped 32 KB → 48 KB to absorb the larger
  `ODriveTelemetryMsg` queue elements + bigger task stacks.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Src/main.c` — wiring,
  IMU seq counter, `bodySpeedsFromUMecanum`, EKF instance in
  `StartODriveTask`, rewritten telemetry printfs.

### Where it lives at runtime

The EKF instance is a stack-local in `StartODriveTask` (one filter per
robot, deterministic lifetime). It is invoked exactly once per
`telemetry_period = 20 ms` tick from inside
`ODrive_UpdateTelemetryAndOdometry`:

1. Pull fresh ODrive encoder velocities, derive `u[0..3]` in rad/s.
2. `bodySpeedsFromUMecanum(u)` → body twist `(vx_body, vy_body, ω_wheel)`.
3. `ekf_predict(&ekf, dt_s)`.
4. `ekf_correct_wheel_twist(&ekf, vx_body, vy_body, ω_wheel)`.
5. Snapshot `g_bno085_seq / g_bno085_yaw / g_bno085_wz`.
6. `ekf_correct_imu(&ekf, -yaw_imu, -omega_imu, seq)` — negation explained
   below.
7. Fill `OdomData` from the new EKF state and the 6×6 `P` mapped to ROS
   convention (`pose_cov[14/21/28] = 1e9` etc. for the un-estimated
   z/roll/pitch slots).
8. Push the message to `CAN_2_UTX_QueueHandle`. The UART_TX_Task drains
   the queue and prints the slim line + (every 5th tick) the fat line.

The bulk scratch matrices used inside Joseph-form correction live in BSS
(`static double` inside `correct()`) so per-call stack frames stay bounded
— the filter is invoked from one task only, so reentrancy isn't a concern.

### IMU sequence gating

The BNO085 reports at ~50 Hz via the SH2 `GAME_ROTATION_VECTOR`. Inside
`imu_sensor_data_cb` (the SH2 callback), after writing the new
`g_bno085_yaw/qx/qy/qz/qw`, the file-scope counter `g_bno085_seq` is
incremented **last** — so a reader who sees a fresh `seq` is guaranteed
to see the corresponding fresh yaw values, not a half-written previous
sample.

`ekf_correct_imu(ekf, yaw, omega_z, seq)` then guards on
`seq == e->last_imu_seq`: if the EKF already consumed that sample, the
function returns immediately. This means:

- At 100 Hz telemetry vs 50 Hz IMU, the EKF applies an IMU correction
  on every other tick. (Cadence aligned now since we run telemetry at
  50 Hz too — but the guard stays, since BNO085 isn't perfectly
  isochronous.)
- A delayed sample (transient I2C glitch) doesn't get applied twice.

### IMU yaw + ω_z sign inversion

The BNO085 mounted on this robot reports yaw and gyro-z **inverted**
relative to ROS `base_link` convention: rotating the robot +90° CCW makes
the raw yaw read approximately –90°. This is rooted in
`firmware/.../sh2/euler.c::q_to_ypr` using a non-ROS quaternion-to-yaw
formula (numerator `2*i*j - 2*r*k` instead of ROS-convention `+ 2*r*k`,
denominator using `j²` slot in place of `qy²` etc.). Both yaw *and* gyro-z
flip together because they share the same physical z-axis.

Previously the home2 `omnidriver` dashboard papered over this by negating
`imu_yaw` and `imu_wz` before publishing `/odrive/imu`. With the EKF
on-MCU, that downstream fix doesn't help — the EKF integrates the raw
yaw into its own state, computes position with mirrored heading, and
emits inverted heading in `ODOM_phi` and `ODOM_q*`. So the fix moves
into the firmware:

```c
const double yaw_imu   = -(double)g_bno085_yaw;
const double omega_imu = -(double)g_bno085_wz;
ekf_correct_imu(ekf, yaw_imu, omega_imu, seq);
```

The raw `g_bno085_*` globals are left untouched so the `IMU_yaw` /
`IMU_wz` UART fields stay backward-compatible with any consumer that has
its own sign fix (notably the home2 omnidriver dashboard, which will
need its negation removed when it adopts the new firmware — see § 8).

A cleaner fix would be to rewrite `q_to_ypr` itself to use the ROS yaw
formula, but that has wider blast radius (multiple consumers, mounting
assumptions). One-line negation at the EKF input is surgical and easy
to undo later.

### Runtime flag: `EKF_TRUST_IMU`

A compile-time `#define EKF_TRUST_IMU 1` at the top of `main.c` sets
`ekf.use_imu` at boot. Setting it to 0 makes `ekf_correct_imu` a no-op
— the EKF runs in wheel-only mode (kinematic prediction + wheel
correction, no IMU), which is the "trust nothing about the IMU"
fallback. It is functionally similar to the old dead-reckoning but with
proper covariance accounting.

### Stack & heap budget

- `ODriveTask` stack: 1024×4 → 1536×4 (6 KB). Peak frame: EKF (~424 B) +
  `ODriveTelemetryMsg` local (~940 B) + cmd + correction call frames ≈
  2.1 KB. Comfortable margin.
- `UART_TX_Task` stack: 1024×4 → 1536×4. Peak frame: telem local + cmd +
  printf varargs ≈ 2.6 KB.
- `configTOTAL_HEAP_SIZE`: 32 768 → 49 152. The telemetry queue (3 deep)
  grew by ~1.9 KB; stack bumps add another ~4 KB; +16 KB heap leaves
  comfortable margin.

---

## 5. Telemetry protocol

### Compact numeric format

Every UART telemetry field is now `i=value` where `i` is a positional
index into a single shared table. The full index map lives as a comment
block above the printf in `main.c::Start_UART_TX_Task` and is mirrored
in `omnibase_ws/src/odrive_comm/odrive_comm/odrive_dashboard.py` as
`TELEM_FIELDS[]` — edits must stay in lock-step.

Highlights of the layout:

| Index range | Fields | In SLIM line | In FAT line |
|---|---|---|---|
| 0..2 | CMD twist | no | yes |
| 3..5 | IMU Euler (deg) | only `3=IMU_yaw` | yes |
| 6..9 | IMU quaternion | no | yes |
| 10..12 | IMU ω | only `12=IMU_wz` | yes |
| 13..15 | IMU linear accel | no | yes |
| 16..19 | IK wheel speeds | no | yes |
| 20..23 | EKF pose (incl `z=0`) | yes | yes |
| 24..27 | EKF quaternion | yes | yes |
| 28..30 | World twist | yes | yes |
| 31..32 | Body twist | yes | yes |
| 33..38 | Pose + twist covariance diag | yes | yes |
| 39..90 | Four axis blocks (13 fields each) | no | yes |
| 91..94 | BT state | no | yes |
| 95 | ESP32 age in ms | no | yes |

### Rates and bandwidth

- SLIM line emitted **every** 20 ms cycle ⇒ **50 Hz**, ~220 chars.
- FAT line emitted every 5th cycle ⇒ **10 Hz**, ~920 chars.

Aggregate: 50 · 220 + 10 · 920 ≈ **20 200 chars/s** ≈ **20 kB/s** at the
UART symbol level (10-bit framing).

Multiples-of-115 200 bandwidth budget for these rates (10-bit framing):

| Baud rate | Capacity | Utilization | Verdict |
|---|---|---|---|
| 115 200 | 11 520 B/s | 175 % | impossible |
| 230 400 | 23 040 B/s | 88 % | tight — no margin for jitter |
| 460 800 | 46 080 B/s | 44 % | sustainable, comfortable |
| 921 600 | 92 160 B/s | 22 % | luxurious |

Firmware ships at **921 600** for headroom; **460 800** is the floor for
50 Hz slim with the kind of margin you want on a USB-CDC link that's also
carrying control inputs in the opposite direction.

### Slim/fat detection on the host

The dashboard parser distinguishes SLIM from FAT by presence of fat-only
fields (specifically `N0` at index 39 — the axis-0 node ID). SLIM lines
don't emit it, so `if 'N0' not in data: <slim path>` works the same as
before, just operates on translated named keys.

---

## 6. Dashboard side (home-custom-base `odrive_comm`)

The home-custom-base dashboard was updated to consume the new EKF outputs
and stand in for the role `robot_localization` previously played:

- **Parameter `publish_tf`** (default `True`) — mirrors `r_l`'s
  `publish_tf: True`. A `tf2_ros.TransformBroadcaster` emits the
  `odom → base_link` transform after every `nav_msgs/Odometry` publish,
  using the same quaternion + position that just went into the topic so
  TF and topic agree exactly.
- **EKF quaternion** consumed when `ODOM_qx..qw` is present, falls back
  to a yaw-derived quaternion for legacy firmware compatibility.
- **EKF body-frame twist** (`ODOM_vxb`, `ODOM_vyb`) plugged into
  `nav_msgs/Odometry.twist.twist.linear` — ROS convention requires
  twist in `child_frame_id` (= body frame). The legacy code unrotated
  `ODOM_vx/vy` (world-frame) into body frame using the firmware's `phi`;
  with the new firmware emitting body frame directly, no rotation is
  needed.
- **Live EKF covariance** (`ODOM_var_x/y/yaw/vx/vy/wz`) populates the
  pose/twist covariance diagonals. Un-estimated slots (`z`, `roll`,
  `pitch`, `vz`, `wx`, `wy`) get the conventional `1e9` "unknown"
  marker so downstream filters don't treat them as tightly zero.
- **`tf2_ros`** added to `package.xml` `exec_depend`.

Backward compatibility: every change is gated by "is this EKF field
present in the parsed dict?". Old firmware that doesn't emit the new
fields falls through to the previous behaviour — yaw-derived quaternion
and the `0.05` placeholder covariance.

---

## 7. Replaceability vs `robot_localization`

For the **omnibase** configuration in `omni_basics.launch.py`, the
on-MCU EKF reproduces robot_localization's fusion semantics exactly:

| `robot_localization` knob | STM32 EKF equivalent | Match |
|---|---|---|
| `frequency: 30.0` | telemetry tick at 50 Hz | ✓ (faster is fine) |
| `sensor_timeout: 0.2` | predict-only without correction grows P | ✓ |
| `two_d_mode: True` | inherent — only 2D states tracked | ✓ |
| `publish_tf: True` | dashboard's `_tf_broadcaster` + `publish_tf` param | ✓ |
| `world_frame: odom` | `odom_frame_id` parameter (default `odom`) | ✓ |
| `odom0_config: [vx, vy]` | `ekf_correct_wheel_twist` with `r_wheel_omega=1000` | ✓ (ω effectively ignored) |
| `imu0_config: [yaw, vyaw]` | `ekf_correct_imu(yaw, omega_z)` | ✓ |
| `imu0_relative: True` | `EKF.imu_relative = 1` (default) | ✓ |
| `imu0_remove_gravitational_acceleration: True` | n/a (accel not fused) | ✓ |
| `process_noise_covariance` default | `default_params()` rescaled by 30× for /√s | ✓ |
| `initial_estimate_covariance` default (1e-9) | `init_cov_pos = 0.10` etc. | slightly looser at boot, irrelevant after a few corrections |

### What this means operationally

After the firmware EKF lands, the home2 launch can drop
`robot_localization` entirely and the downstream consumers (Nav2,
slam_toolbox, RTABMap) keep working unchanged. The dashboard now
provides both `/odrive/odom` (filtered) and the `odom → base_link` TF,
which is exactly the two outputs `robot_localization` was producing.

**Dashgo configuration is not affected** — it uses a separate EKF that
fuses `dashgo_odom` (vx + vyaw) with its own IMU. Only the omnibase
launch path changes.

### What this does NOT replace

- **Map → odom TF.** That comes from SLAM (`slam_toolbox` on omnibase,
  RTABMap on dashgo), not from the local EKF. Nothing changes there.
- **15-state semantics.** `robot_localization` carries z, roll, pitch,
  accelerations etc. The EKF only tracks the 6 states that actually
  matter for an omnibase. Anything you were doing with `two_d_mode:
  False` elsewhere wouldn't be served by this EKF — but `two_d_mode:
  True` on the omnibase config means you weren't using those states
  anyway.

---

## 8. What the home2 `omnidriver` package needs

`omnidriver` is a downstream fork of the same dashboard, hardened for
production but unaware of the new firmware. To consume the EKF firmware
cleanly:

### 8.1 Required changes

1. **Bump `baud_rate` default** from 230 400 to 921 600 (or 460 800 if
   the cable is dodgy at higher rates). Firmware-side baud is
   non-negotiable; host must match.

2. **Drop the manual world→body un-rotation block** in both
   `_publish_imu_odom()` (slim path) and `_parse_and_publish()` (fat
   path). The firmware now emits `ODOM_vxb`/`ODOM_vyb` directly. Use
   them. Keep the `linear_scale_x / linear_scale_y` calibration knobs
   — they're empirical wheel-radius corrections and still apply.

3. **Drop the `-math.radians(imu_y)` / `-imu_wz` yaw inversion** in the
   `/odrive/imu` build path. The firmware now negates internally before
   feeding the EKF. If you leave the dashboard inversion in *while* the
   new firmware is flashed, you double-invert.

4. **Prefer `ODOM_q*` for orientation.** When the new EKF fields are
   present in the parsed dict, use them. Fall back to
   `_yaw_to_quaternion(o_phi)` only for old firmware.

5. **Prefer `ODOM_var_*` for covariance**, with the `1e9` "unknown"
   convention on un-estimated slots. Fall back to the existing tuned
   `(0.02, 0.08, 0.05)` twist values + `(0.05, 0.05, 0.10)` pose values
   for legacy firmware only.

6. **Add a `TransformBroadcaster`** and a `publish_tf` parameter
   (default `True`). Emit `odom → base_link` after the
   `nav_msgs/Odometry` publish in BOTH the fast path and the fat path.

7. **Verify `tf2_ros`** is in `package.xml` (it already is, for
   robot_localization).

The required edits are roughly 60 lines across `_parse_and_publish`,
`_publish_imu_odom`, `__init__`, and the parameter callback. Mirror
the home-custom-base diff for what each change looks like.

### 8.2 Things to keep unchanged

- The **FAST PATH** (`_publish_imu_odom`) for the slim line is still a
  real win — it skips the per-axis parsing when nothing per-axis is
  going to be parsed. Keep it; just update its build to use the new
  fields like the fat path does.
- **Web-socket disconnect safety** (zero `tx_vx/vy/wz` on disconnect).
  Important safety knob; nothing changes.
- **IMU yaw outlier rejection** in the parser. Now that the EKF has its
  own outlier rejection (see § 3), the dashboard's gate is redundant
  but not harmful — leave it in as defence-in-depth, or remove it after
  bench-verifying the EKF's filter catches everything.
- **`linear_scale_x` / `linear_scale_y`**. These are mecanum wheel-radius
  calibration knobs. They still apply, just to the body-frame value the
  firmware now provides directly.

### 8.3 Home2 launch-file change

In `omni_basics.launch.py`, remove the `robot_localization` `Node(...)`
entry. The downstream subscribers (slam_toolbox / Nav2 / map_context)
that currently read `/odometry/filtered` should be remapped to
`/odrive/odom`. Alternatively, rename the dashboard's publisher topic to
`/odometry/filtered` — same outcome, different lever.

`omnidriver`'s `imu_frame_id: 'base_link'` setting can stay (it skips a
TF lookup, which is fine). Frame IDs already match
robot_localization's outputs (`odom`, `base_link`), so no other launch
plumbing changes.

### 8.4 Order of operations on the home2 robot

To stage the changeover safely:

1. **Firmware first.** Flash the new firmware. The home2 robot is now
   running the on-MCU EKF, but the omnidriver dashboard's inversion +
   un-rotation is double-correcting → `/odrive/odom` will look wrong.
   Don't bring up Nav2 in this state.
2. **omnidriver second.** Apply the dashboard edits above and
   `colcon build --packages-select omnidriver`. Now `/odrive/odom` is
   correct, `odom → base_link` TF flows from the dashboard, and
   `robot_localization` is still running and double-filtering on top —
   but the two are in agreement, so this is harmless and a useful sanity
   check.
3. **Drop robot_localization.** Remove the `Node` from the launch file.
   The omnidriver dashboard alone now serves `/odrive/odom` and TF.
   Verify slam_toolbox + Nav2 still see what they need.
4. **(Optional)** Verify the in-firmware outlier rejection catches the
   "yaw flash" condition before removing the dashboard's redundant
   pre-filter.

---

## 9. Known gaps and future work

- **Pitch / roll not estimated.** The EKF is planar. A mounting tilt on
  the BNO085 would manifest as systematic position drift; address by
  either (a) levelling the IMU mount, or (b) extending the EKF to fuse
  the gravity-aligned roll/pitch from the BNO085. The OdomData struct
  already carries the slots; the math change is mechanical.
- **No IMU outlier rejection on ω_z.** Yaw is filtered, gyro-z isn't.
  If the BNO085 spat out a corrupted ω sample, it would update the
  filter cleanly because `r_imu_omega = 0.03` is small. Symmetry would
  argue for a similar guard on ω_z, e.g. `|Δω| > ω_max_phys`.
- **`q_to_ypr` global fix.** The narrow yaw negation in §4 is surgical
  but the underlying bug is in `sh2/euler.c::q_to_ypr`. A proper fix
  there (ROS-convention quaternion-to-yaw, and a single negation of
  `g_bno085_wz` at the IMU callback) globalises the correction so
  nobody downstream has to compensate. Best done in coordination with
  the omnidriver dashboard's removal of its own inversion, since
  fixing both at once avoids a transient double-invert.
- **Initial covariance is loose.** `init_cov_pos = 0.10` lets the first
  IMU correction settle without fighting an over-confident prior, but
  it means downstream consumers see "unknown pose" for the first few
  EKF ticks. If that ever matters, tighten to `1e-9` to match r_l's
  default — corrections will pull state in quickly anyway.
- **No fuse-of-CMD in EKF.** The commanded velocity is observable
  (`last_cmd.robot_twist`) and could be used as a third correction
  source (encoder-disagreement check). Probably not worth the
  complexity, but worth recording as a possibility.
- **No on-MCU bias estimation.** Gyro bias is what makes the BNO085
  yaw drift slowly when stationary. With `imu_relative: True` we
  zero the initial offset but not the drift rate. If long-duration
  navigation without SLAM corrections becomes important, adding a
  bias state to the EKF is the standard remedy (state grows to 7 or
  8 depending on whether you also track ax/ay bias).

---

## 10. File inventory

**New:**

- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/ekf.h`
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Src/ekf.c`

**Modified (firmware):**

- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/main.h` — `OdomData`
  extended with EKF outputs + 6×6 covariance matrices.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Inc/FreeRTOSConfig.h` —
  `configTOTAL_HEAP_SIZE` 32 768 → 49 152.
- `firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/Core/Src/main.c` —
  `ekf.h` include, `g_bno085_seq` global + increment in IMU callback,
  `EKF_TRUST_IMU` flag, `bodySpeedsFromUMecanum` helper,
  `ODrive_UpdateTelemetryAndOdometry` rewritten around the EKF,
  yaw + ω negation before `ekf_correct_imu`, slim/fat printfs
  converted to numeric-index format with shared index-map comment,
  `telemetry_period` 10 ms → 20 ms (50 Hz slim, 10 Hz fat),
  `ODriveTask` and `UART_TX_Task` stacks 1024×4 → 1536×4.

**Modified (ROS, home-custom-base):**

- `omnibase_ws/src/odrive_comm/odrive_comm/odrive_dashboard.py` —
  `tf2_ros` import + `TransformBroadcaster`, `publish_tf` parameter,
  parser uses real `ODOM_q*` quaternion and `ODOM_var_*` covariance
  when present (falls back for legacy firmware), TF emission after
  every `nav_msgs/Odometry` publish, TELEM_FIELDS comment updated.
- `omnibase_ws/src/odrive_comm/package.xml` — `tf2_ros` exec_depend.

**Pending (ROS, home2 — not done in this branch):**

- `navigation/packages/omnidriver/omnidriver/odrive_dashboard.py` —
  see § 8.1.
- `navigation/packages/nav_main/launch/omni_setup/omni_basics.launch.py`
  — remove `robot_localization` Node (see § 8.3).
