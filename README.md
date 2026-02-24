# xgo2_ros — ROS 2 Humble Controller for XGO Lite v2

ROS 2 Humble controller for the [XGO Lite v2](https://www.luwudynamics.com) quadruped robot.
Communicates with the robot exclusively over **Bluetooth LE** (via [bleak](https://github.com/hbldh/bleak)) — no serial port or host-side ROS 2 install required.
Ships a **Docker-first** development workflow.

---

## Architecture

```
joystick (joy_node)
        │  /joy
        ▼
teleop_twist_joy  ──►  /cmd_vel  ──►  xgo2_ros_node  ──BLE write──►  XGO Lite v2
                        /body_pose ──►        │
                        /arm_pose  ──►        │◄── BLE notify ──────  XGO Lite v2
                                              │
                              /joint_states, /imu/data,
                              /battery, /odom, TF odom→base_link
```

`xgo2_ros_node` is a Python (rclpy) node that communicates with the robot using the XGO binary protocol over BLE.

### BLE communication

The node uses a **request-response** polling model (the XGO does not stream feedback over BLE):

- A 10 Hz ROS timer calls `_poll_state()`, which sends three read requests per tick via the write characteristic (`0xFFF2`):
  - `0x01` — battery (1 byte)
  - `0x50` — 15 joint angles (uint8 each)
  - `0x62` — roll, pitch, yaw as three 32-bit little-endian floats (12 bytes)
- The robot responds via the notify characteristic (`0xFFF1`) with framed packets: `0x55 0x00 LEN TYPE ADDR [data...] CHECKSUM 0x00 0xAA`
- Responses are routed by ADDR to update the appropriate state and publish the corresponding ROS messages.

### BLE connection management

On startup the node first **scans** for the robot's MAC address. If it is not found it retries every 5 seconds rather than crashing. Once connected, if the link drops the node automatically clears its connected state and returns to scanning.

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| Docker + Docker Compose | Host machine only |
| Bluetooth adapter on host | Shared into the container via `/run/dbus` |
| XGO Lite v2 powered on | BLE address configured in `config/config.json` |
| Joystick (optional) | Connected before container starts |

---

## Quick start

### 1. Clone

```bash
git clone <this-repo> xgolite_ws
cd xgolite_ws
```

### 2. Configure the BLE address

Edit `src/xgo_ros/config/config.json` and set `bt_address` to match your robot:

```json
{
    "bt_address": "88:57:21:92:73:02",
    ...
}
```

Find your robot's address from the host with:

```bash
bluetoothctl scan on
```

### 3. Build and start the container

```bash
docker compose build
docker compose up -d
```

### 4. Build the ROS 2 workspace (first time only)

```bash
docker compose exec ros2-xgolite bash
colcon build
source install/setup.bash
```

### 5. Launch

```bash
# Full stack: xgo node + joy driver + teleop
ros2 launch xgo2_ros xgo_control_launch.py

# Without joystick
ros2 launch xgo2_ros xgo_control_launch.py use_joy:=false

# Different joystick device
ros2 launch xgo2_ros xgo_control_launch.py joy_dev:=/dev/input/js1
```

---

## Joystick control

Joystick input is handled directly by `xgo_bt_node.py` (`/joy` subscriber). The mapping assumes an Xbox-style controller; run `jstest /dev/input/js0` inside the container to verify your controller's indices.

### Walking (hold BtnTL1 to activate)

| Input | Index | Action |
|-------|-------|--------|
| **BtnTL1** (hold) | button 4 | Enable walking mode |
| Left stick ↕ | axis 1 | Forward / backward (vx) |
| Left stick ↔ | axis 0 | Strafe left / right (vy) |
| Right stick ↔ | axis 3 | Yaw / turn (vyaw) |

### Body pose (hold BtnTR1 to activate)

| Input | Index | Action |
|-------|-------|--------|
| **BtnTR1** (hold) | button 5 | Enable body-pose mode |
| Left stick ↕ | axis 1 | Body translation X |
| Left stick ↔ | axis 0 | Body translation Y |
| Right stick ↔ | axis 3 | Roll |
| Right stick ↕ | axis 4 | Pitch |

Walk and pose modes are mutually exclusive — holding both buttons activates walk only.

### Height

| Input | Index | Action |
|-------|-------|--------|
| **BtnThumbL** | button 9 | Raise body height +5 mm (range 75–115 mm) |
| **BtnThumbR** | button 10 | Lower body height −5 mm |

Height is preserved across pose-mode releases and is only reset to default (108 mm) by BtnStart.

### Arm

| Input | Index | Action |
|-------|-------|--------|
| **BtnSelect** | button 6 | Toggle arm mode (register 0x72: 0↔1) |
| Left Pad ↔ | axis 6 | Arm X position ±10 units per press |
| Left Pad ↕ | axis 7 | Arm Z position ±10 units per press |
| **BtnTL2** (trigger) | axis 2 | Toggle claw open / closed |

### Behaviour

| Input | Index | Action |
|-------|-------|--------|
| **BtnX** | button 2 | Cycle pace mode: normal → slow → high |
| **BtnY** | button 3 | Cycle gait type: trot → walk → high walk |
| **BtnStart** | button 7 | Reset robot to default pose |

---

## Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All 15 servo angles + claw |
| `/imu/data` | `sensor_msgs/Imu` | Roll, pitch, yaw orientation (quaternion) |
| `/battery` | `sensor_msgs/BatteryState` | Battery percentage |
| `/odom` | `nav_msgs/Odometry` | Dead-reckoning odometry |
| TF `odom→base_link` | — | Transform integrated from velocity + IMU orientation |

## Subscribed topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command (linear x/y, angular z) |
| `/body_pose` | `geometry_msgs/Pose` | Incremental body translation and orientation |
| `/arm_pose` | `geometry_msgs/Pose` | Incremental arm position (x/y/z) |

---

## Configuration reference (`config/config.json`)

| Key | Description |
|-----|-------------|
| `bt_address` | BLE MAC address of the robot |
| `bt_char_write` | GATT characteristic UUID for commands (default `0xFFF2`) |
| `bt_char_notify` | GATT characteristic UUID for feedback (default `0xFFF1`) |
| `vx_max` / `vy_max` / `vyaw_max` | Velocity limits (m/s and rad/s) |
| `joint_limit` | Min/max degree range for each joint group |
| `body_limit` | Min/max range for body translation/rotation |
| `arm_limit` | Min/max range for arm axes |

---

## Project structure

```
xgolite_ws/
├── docker-compose.yml          # Container definition
├── Dockerfile                  # ros:humble-desktop + bleak + joystick tools
├── entrypoint.sh               # Sources ROS 2 and workspace overlay
├── cyclonedds.xml              # CycloneDDS RMW config
├── xgolib.py                   # Upstream XGO Python library (reference)
├── xgolite_bt.py               # BLE comms prototype (reference)
└── src/
    └── xgo_ros/                # xgo2_ros ROS 2 package
        ├── CMakeLists.txt      # Installs xgo_bt_node.py as xgo2_ros_node
        ├── src/
        │   └── xgo_bt_node.py  # Main BLE controller node
        ├── launch/
        │   └── xgo_control_launch.py
        └── config/
            └── config.json
```

---

## Known limitations

- IMU provides roll/pitch/yaw orientation but no angular velocity (gyro) readings.
- Odometry is dead-reckoning only (velocity integration + IMU yaw); accuracy degrades without a correction source.
- Body pose and arm pose subscribers apply **incremental** deltas — absolute positioning is not yet supported.
- BLE poll rate is capped at ~10 Hz; command throughput is lower than the original serial interface.
- No linear acceleration data currently polled from the robot (register not yet identified).

---

## License

Apache License 2.0 — see [LICENSE](../../LICENSE).

