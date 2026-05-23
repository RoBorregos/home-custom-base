# odrive_comm

ROS 2 (rclpy) package that bridges the STM32 firmware to ROS and serves
the web dashboard.

It reads the 100 Hz CSV telemetry stream from the STM32 over USART3,
splits it into typed ROS topics (`/odrive/odom`, `/odrive/imu`, etc.),
re-emits the same data to a Socket.IO/Flask web UI at
`http://<host>:5000`, and accepts `/cmd_vel` (`Twist` or `TwistStamped`)
to write back type-1 velocity commands over the same serial port.

For the firmware side — pinout, CSV layout, EKF derivation, watchdogs
— see
[`firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/docs_update.md`](../../../firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/docs_update.md)
and the firmware
[`README`](../../../firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/README.md).

---

## Nodes

| Executable             | What it does                                                                |
| ---------------------- | --------------------------------------------------------------------------- |
| `odrive_dashboard`     | Full serial bridge + web dashboard (the main node)                          |
| `odrive_serial_twist`  | Minimal twist → STM32 path, no dashboard                                    |
| `simple_rx`            | Diagnostic: dump raw telemetry lines to stdout                              |

All three are launched via `ros2 run odrive_comm <name>`.

---

## Build & run

```bash
cd ~/Github/home-custom-base/omnibase_ws
colcon build --packages-select odrive_comm
source install/setup.bash

ros2 run odrive_comm odrive_dashboard
```

Then open `http://<host>:5000` in a browser. The dashboard serves itself
entirely from local assets — no internet connection required at runtime.

### Dashboard assets

The HTML at `assets/dashboard.html` loads two third-party libraries:

```html
<script src="/js/chart.umd.min.js"></script>
<script src="/js/socket.io.min.js"></script>
```

Both files live under `assets/js/` and are served by the node's own
`/js/<path:filename>` Flask route. They are already vendored into the
package; if you ever delete the `assets/js/` folder or pull a fresh
checkout that's missing them, restore with:

```bash
mkdir -p ~/Github/home-custom-base/omnibase_ws/src/odrive_comm/assets/js

wget -O ~/Github/home-custom-base/omnibase_ws/src/odrive_comm/assets/js/chart.umd.min.js \
    https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js

wget -O ~/Github/home-custom-base/omnibase_ws/src/odrive_comm/assets/js/socket.io.min.js \
    https://cdn.jsdelivr.net/npm/socket.io@4.7.2/client-dist/socket.io.min.js
```

After restoring, rebuild with `colcon build --packages-select odrive_comm`
so `setup.py` re-installs the files into the package's `share/` directory.

> Why bundled and not via CDN? `python-socketio` 5.x dropped its built-in
> client-JS endpoint (`/socket.io/socket.io.js` returns HTTP 400), so the
> client library must be served by us.

---

## Topics published by `odrive_dashboard`

All on the `/odrive/...` namespace unless noted:

| Topic                       | Type                        | Notes                                    |
| --------------------------- | --------------------------- | ---------------------------------------- |
| `/odrive/raw`               | `std_msgs/String`           | Raw CSV line from the STM32              |
| `/odrive/debug`             | `std_msgs/String`           | Parser diagnostics                       |
| `/odrive/cmd_twist`         | `std_msgs/Float32MultiArray`| Last commanded `[vx, vy, wz]`            |
| `/odrive/imu`               | `sensor_msgs/Imu`           | BNO085 quaternion + gyro + linear accel  |
| `/odrive/imu_euler`         | `std_msgs/Float32MultiArray`| Yaw / pitch / roll in degrees            |
| `/odrive/odom`              | `nav_msgs/Odometry`         | **EKF output** — pose + body twist + 6×6 covariance |
| `/odrive/body_twist`        | `std_msgs/Float32MultiArray`| EKF body-frame `[vx, vy, wz]`            |
| `/odrive/ik_wheel_speeds`   | `std_msgs/Float32MultiArray`| IK output `u[0..3]` (rad/s)              |
| `/odrive/node_ids`          | `std_msgs/Int32MultiArray`  | Live ODrive node IDs                     |
| `/odrive/axis_errors`       | `std_msgs/Int32MultiArray`  | Per-axis error code                      |
| `/odrive/axis_states`       | `std_msgs/Int32MultiArray`  | Per-axis state enum                      |
| `/odrive/axis_states_str`   | `std_msgs/String`           | Human-readable axis state                |
| `/odrive/controller_status` | `std_msgs/Int32MultiArray`  | Per-axis controller status flag          |
| `/odrive/pos_est`           | `std_msgs/Float32MultiArray`| Encoder position estimate (turns)        |
| `/odrive/vel_est`           | `std_msgs/Float32MultiArray`| Encoder velocity estimate (turns/s)      |
| `/odrive/encoder_shadow`    | `std_msgs/Int32MultiArray`  | Encoder shadow counts                    |
| `/odrive/encoder_cpr`       | `std_msgs/Int32MultiArray`  | Encoder CPR                              |
| `/odrive/bus_voltage`       | `std_msgs/Float32MultiArray`| Per-axis bus voltage                     |
| `/odrive/bus_current`       | `std_msgs/Float32MultiArray`| Per-axis bus current                     |
| `/odrive/iq_setpoint`       | `std_msgs/Float32MultiArray`| Iq setpoint                              |
| `/odrive/iq_measured`       | `std_msgs/Float32MultiArray`| Iq measured                              |
| `/odrive/updated`           | `std_msgs/Int32MultiArray`  | CAN-RX freshness flag                    |
| `/odrive/discovered_nodes`  | `std_msgs/Int32MultiArray`  | Node IDs seen since boot                 |

Subscribed: `/cmd_vel` as either `geometry_msgs/Twist` or
`geometry_msgs/TwistStamped` — used to feed type-1 velocity commands
back to the STM32.

---

## Web dashboard

`http://<host>:5000` shows ODrive status, the EKF odometry trace, IK
wheel speeds, the IMU quaternion, and a "Links" health card. Anything
on the dashboard is also available as a ROS topic above.

### Bluetooth status indicator

Top-row, third column: a colored badge driven by the `BT_active` field
parsed from the telemetry line.

| `BT_active` | Label    | Colour |
| ----------- | -------- | ------ |
| 0           | Inactive | red    |
| 1           | Paired   | amber  |
| 2           | Active   | green  |

Updates are pure read-only — the dashboard has no toggle for it. The
STM32 sets the value based on what the ESP32 reports in the Type-3
packet (`buttons` bit `0x02` toggles the BT path off; see firmware
docs § 5).

### Link-status widget

Two rows on the Odometry card:

```
ESP32 → STM32 :   <age> ms     [OK | WARN | LOST | UNKNOWN]
STM32 → PC    :   <age> ms     [OK | WARN | LOST | UNKNOWN]
```

| Bucket  | Age range                | Pill colour       |
| ------- | ------------------------ | ----------------- |
| OK      | `< 500 ms`               | green             |
| WARN    | `500 ms ≤ age < 2 s`     | amber             |
| LOST    | `≥ 2 s`                  | red               |
| UNKNOWN | firmware sentinel (`-1`) | muted (default)   |

- **ESP32 → STM32** is computed on the firmware side and arrives as the
  `ESP32_age_ms` field of the telemetry CSV. See the firmware-side
  reference: [`docs_update.md` § 6.1](../../../firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7/docs_update.md#61-link-status-monitoring-esp32_age_ms).
- **STM32 → PC** is computed in the ROS node from `time.monotonic()`,
  stamped on every raw line read from the serial port. A 4 Hz timer
  re-emits the latest telemetry to the browser so the age value keeps
  ticking up even when the STM32 has gone silent.

### Disconnection cases

| Failure                        | ESP32 → STM32 widget                       | STM32 → PC widget               |
| ------------------------------ | ------------------------------------------ | ------------------------------- |
| ESP32 off / paired but silent  | climbs to WARN → LOST                      | stays OK                        |
| STM32 unplugged / firmware crash | freezes at last value (do **not** extrapolate) | climbs to WARN → LOST       |
| ROS node restarted on host     | both rows reset to UNKNOWN, then re-converge | (the node itself was down)     |

The STM32 → PC LOST pill is the authoritative "you can no longer trust
the ESP32 → STM32 number" signal — the firmware can't update it while
its own UART is offline.

### Verifying from the CLI

```bash
ros2 topic echo /odrive/raw --once    # confirms a telemetry line is flowing
ros2 topic hz /odrive/raw             # should be ~100 Hz
```

The raw line must end with `...,ESP32_age_ms=<n>` for the firmware-side
link monitoring to work. If the field is missing, the dashboard shows
`UNKNOWN` for the ESP32 row regardless of actual BT state.
