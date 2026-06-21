# odrive_comm

ROS2 package that bridges the STM32H7 omnibase firmware (USB-CDC serial) to ROS2 topics, and serves a Flask + Flask-SocketIO web dashboard for live monitoring and manual control.

For the firmware side of this link, see `../../../firmware/STM32H7_OMNIBASE_CAN_BNO085/omnibase_documentation.md` (§11 covers the on-MCU EKF design in detail, §12 the pending home2 `omnidriver` migration). For a dated changelog of this package specifically, see `status.txt` in this directory.

---

## What's in this package

| File | Role |
|---|---|
| `odrive_comm/odrive_dashboard.py` | **Main node.** Serial bridge + ROS2 publishers/subscribers + embedded web dashboard. This is what you actually run. |
| `odrive_comm/odrive_serial_twist.py` | Minimal alternative node: forwards `cmd_vel` to the STM32 over serial as `"vx vy wz\r\n"`, with no telemetry parsing or web UI. |
| `odrive_comm/simple_rx.py` | Debug utility: dumps raw serial lines from the STM32 onto `/uart_rx_raw`, nothing else. |
| `assets/dashboard.html` | The dashboard's entire frontend (HTML/CSS/JS in one file — no build step). |
| `assets/js/*.min.js` | Vendored Chart.js + socket.io client, served locally so the dashboard works without internet access. |

Three console scripts are installed: `odrive_dashboard`, `odrive_serial_twist`, `simple_rx`. In normal use you only need `odrive_dashboard`.

---

## Running it

```bash
cd omnibase_ws
colcon build --packages-select odrive_comm
source install/setup.bash
ros2 run odrive_comm odrive_dashboard
```

Dashboard at `http://<host_ip>:5000`.

### Key parameters (`odrive_dashboard.py`)

| Parameter | Default | Notes |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Auto-detected by USB VID/PID (`_find_stm_port`) if not found at this path; this is just the fallback. |
| `baud_rate` | `230400` | **Must match** the STM32 firmware's `USART3.Init.BaudRate` (`main.c`). |
| `tx_period` | `0.1` (10 Hz) | How often the `cmd_vel`-derived velocity command is sent to the STM32. This is also the relevant timescale for `vel_ramp_rate` tuning — see the firmware docs. |
| `node_ids` | `[33, 34, 35, 40]` | Expected ODrive CAN node IDs; used only for the "discovered vs expected" check, not for control. |
| `enable_web_gui` | `True` | Set `False` to run as a headless serial↔ROS bridge only. |
| `web_gui_port` | `5000` | |
| `publish_tf` | `True` | Broadcasts `odom -> base_link` after every `/odrive/odom` publish, using the live EKF quaternion. Set `False` if running an external EKF (e.g. `robot_localization`) instead — see "EKF and TF" below. |
| `odom_frame_id` / `base_frame_id` / `imu_frame_id` | `odom` / `base_link` / `imu_link` | Frame IDs used in published `Odometry`/`Imu` messages and the TF. |
| `use_stamped_cmd_vel` | `True` | Set `False` if your `cmd_vel` source publishes unstamped `geometry_msgs/Twist` (e.g. `teleop_twist_keyboard`, nav2 on Humble by default). |
| `linear_scale_x` / `linear_scale_y` | `1.0` | Empirical wheel-radius calibration multipliers applied to body-frame velocity before publishing. Tune live with `ros2 param set` — no rebuild needed. |
| `demo_mode` | `False` | Runs without a serial connection (falls back automatically if the configured port can't be opened). |

---

## Serial protocol (PC ↔ STM32)

**PC → STM32:**
```
Type 1 — velocity:  "1 <vx> <vy> <wz>\r\n"
Type 2 — config:    "2 <sub_type> <mask_hex> [params...]\r\n"
```
The full `sub_type` table (clear errors, set state, gains, limits, startup, arbitrary CAN parameter write, etc.) is documented in the module docstring at the top of `odrive_dashboard.py`, and mirrored in the firmware docs.

**STM32 → PC**, three line shapes, all parsed by `_parse_and_publish()`:
- **Slim telemetry** (sent every firmware cycle, no per-axis fields) — the two EKF outputs (`/odrive/imu`, `/odrive/odom`) at high rate. Recognized by the *absence* of an `N0` field.
- **Fat telemetry** (every 5th cycle) — full per-axis diagnostic snapshot (errors, states, position/velocity, bus voltage/current, etc.) plus everything in the slim line.
- **Firmware error lines** — `E=<code>,<axis>,<detail>` and `ELOST=<n>`, parsed first (`line.startswith('E=')` / `'ELOST='`) before the generic key=value regex runs.

### Why the slim/fat split matters here

`_parse_and_publish()` has an explicit fast-path: if `'N0' not in data`, it's a slim line, and the parser **must not** fall through to the full per-axis rebuild — doing so would default every axis field (`node_ids`, `axis_states`, `pos_est`, ...) to zero and overwrite the good values just published from the last fat line. This actually broke once (slim lines flickering the dashboard between real data and `UNDEFINED`/`0.000`) and was fixed by patching only the odom/IMU/`sm_state` fields onto a copy of the cached telemetry dict on the fast path, instead of rebuilding the whole dict. If you ever touch this parser, preserve that guard.

### Firmware error parsing

`E=`/`ELOST=` lines are checked **before** anything else in `_parse_and_publish()` and `return` immediately — they must not reach the generic `\w+=value` regex, since e.g. `E=1,255,0` would partially match (`E=1`) and then incorrectly take the slim-line fast path, publishing a phantom all-zero `/odrive/imu`+`/odrive/odom` message. `FERR_DESCRIPTIONS` maps every firmware error code to a readable string; matching codes are defined in the firmware's `main.h` (`FERR_*`) — keep both in sync if codes change.

---

## EKF and TF

The STM32 runs an on-MCU 6-state EKF (wheel odometry + BNO085 IMU). This node consumes its quaternion (`ODOM_q*`) and full pose/twist covariance (`ODOM_var_*`) when present in the parsed telemetry, and falls back to a yaw-derived quaternion + placeholder covariance for older firmware that doesn't emit those fields — so this node works against either firmware generation without a parameter flip.

With `publish_tf: True` (default), this node's `TransformBroadcaster` fully replaces `robot_localization` for the omnibase configuration — see `omnibase_documentation.md` §11 for the replaceability analysis and known gaps (no pitch/roll, no IMU staleness watchdog yet, etc.).

**Resetting the EKF**: restarting this node does **not** reset the EKF — the filter state lives entirely on the STM32 (a local variable inside its `ODriveTask`, initialized once at firmware boot). Restarting this node only clears the node's own cached `_latest_telem`. To re-zero the EKF, reset the STM32 itself (power cycle, NRST, ST-Link reset, or reflash).

---

## Firmware error log vs. activity log (dashboard UI)

The dashboard has two distinct log panels — don't confuse them:
- **Activity log**: things *you* did (commands sent from the UI).
- **Firmware errors**: things the *firmware* reported (CAN faults, ODrive faults, IMU faults, watchdogs, stack health). Sourced from `E=`/`ELOST=` lines, shown via the `firmware_error` socket event.

These used to cross-post into each other; that mirroring was removed so each panel only shows its own kind of event.

---

## Dashboard gain controls — read this before tuning

The Pos/Vel/Int gain sliders in the dashboard are **write-only, not a readback**:
- The numbers shown on page load are static HTML defaults, not the ODrive's actual configured gains.
- Gains are **never sent automatically** — only when you click "Apply gains".
- There is no way to read gains back from the ODrive over CAN (ODrive's CANSimple protocol has no `GET_VEL_GAINS`/`GET_POSITION_GAIN`). If you need to confirm a gain actually took effect, check via `odrivetool`/USB.

The same write-only caveat applies to the "Advanced param (RxSdo)" box, which writes an arbitrary float32 ODrive parameter by its `flat_endpoints.json` endpoint ID (see the firmware docs, Section 10) — no read-back is implemented there either. **`odrive_config/flat_endpoints.json` at the repo root is the fw v0.6.12 map for these 4 ODrives** — look up any `axis0.controller.config.*` (or other) parameter name there to get the numeric endpoint ID for this box; it's not limited to `vel_ramp_rate`.

---

## Axis Lab tab — untested

The dashboard's **Axis Lab tab** (`#tab-axislab` in `dashboard.html`) — per-motor inspection, manual control, and calibration — has **not been validated against real hardware**. Treat it as unverified until someone runs through it on the physical robot.

---

## Topics published

`odrive/raw`, `odrive/debug`, `odrive/cmd_twist`, `odrive/imu_euler`, `odrive/imu` (`sensor_msgs/Imu`), `odrive/ik_wheel_speeds`, `odrive/odom` (`nav_msgs/Odometry`), `odrive/body_twist`, `odrive/node_ids`, `odrive/axis_errors`, `odrive/axis_states`, `odrive/axis_states_str`, `odrive/controller_status`, `odrive/updated`, `odrive/pos_est`, `odrive/vel_est`, `odrive/encoder_shadow`, `odrive/encoder_cpr`, `odrive/bus_voltage`, `odrive/bus_current`, `odrive/iq_setpoint`, `odrive/iq_measured`, `odrive/discovered_nodes`.

Plus the `odom -> base_link` TF (when `publish_tf: True`).

## Topics subscribed

`cmd_vel` (`Twist` or `TwistStamped`, per `use_stamped_cmd_vel`), `odrive/config_cmd` (raw `String`, forwarded verbatim to the STM32 if it starts with `"2 "`).

## Web socket events (dashboard ↔ browser)

`telemetry` (full state snapshot, browser-bound), `command` (browser → node, dispatched in `_handle_web_command`), `firmware_error` (browser-bound, see above), `log` (browser-bound activity-log entries).
