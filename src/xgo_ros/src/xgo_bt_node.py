#!/usr/bin/env python3
"""
XGO2 ROS2 Bluetooth Node
Replaces the serial-based C++ node with a BLE implementation using bleak.
Publishes: /joint_states, /imu/data, /battery, /odom, TF odom->base_link
Subscribes: cmd_vel, body_pose, arm_pose
"""

import asyncio
import json
import math
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState, Joy
from tf2_ros import TransformBroadcaster

from bleak import BleakClient, BleakScanner

# Default BLE characteristic UUIDs for XGO robots
_DEFAULT_CHAR_WRITE = "0000fff2-0000-1000-8000-00805f9b34fb"
_DEFAULT_CHAR_NOTIFY = "0000fff1-0000-1000-8000-00805f9b34fb"


# ---------------------------------------------------------------------------
# Helpers (mirror of conversion_utils.hpp)
# ---------------------------------------------------------------------------

def _limit(data: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, data))


def _float_to_uint8(data: float, min_limit: float, max_limit: float) -> int:
    return int((data - min_limit) / (max_limit - min_limit) * 255) & 0xFF


def _uint8_to_float(data: int, min_limit: float, max_limit: float) -> float:
    return data / 255.0 * (max_limit - min_limit) + min_limit


def _to_rad(degrees: float) -> float:
    return degrees / 57.3


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Return [x, y, z, w] quaternion from roll/pitch/yaw (radians)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _build_packet(addr: int, data_bytes) -> bytes:
    """Construct an XGO protocol packet."""
    length = len(data_bytes) + 8
    mode = 0x01
    value_sum = sum(data_bytes)
    checksum = (255 - (length + mode + addr + value_sum) % 256) & 0xFF
    packet = [0x55, 0x00, length, mode, addr]
    packet.extend(data_bytes)
    packet.extend([checksum, 0x00, 0xAA])
    return bytes(packet)


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class XGOBTNode(Node):
    """XGO2 quadruped controller node communicating over Bluetooth LE."""

    def __init__(self):
        super().__init__('xgo_control_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self._config = self._load_config(config_path)
        # Right-stick mapping is configurable to support different joypads.
        self.declare_parameter('right_stick_h_axis', 3)
        self.declare_parameter('right_stick_v_axis', 4)
        self.declare_parameter('walk_yaw_axis', 3)
        self.declare_parameter('walk_yaw_sign', 1.0)
        self.declare_parameter('joypad_profile', 'classic')
        self.declare_parameter('pose_swap_roll_pitch', False)
        self.declare_parameter('pose_roll_sign', 1.0)
        self.declare_parameter('walk_button_index', 4)
        self.declare_parameter('pose_button_index', 5)
        self.declare_parameter('tl2_axis_index', 2)
        self.declare_parameter('tl2_button_index', -1)
        self.declare_parameter('tr2_button_index', 7)
        self._right_stick_h_axis = int(self.get_parameter('right_stick_h_axis').value)
        self._right_stick_v_axis = int(self.get_parameter('right_stick_v_axis').value)
        self._walk_yaw_axis = int(self.get_parameter('walk_yaw_axis').value)
        self._walk_yaw_sign = float(self.get_parameter('walk_yaw_sign').value)
        self._joypad_profile = str(self.get_parameter('joypad_profile').value).lower()
        pose_swap_roll_pitch = self.get_parameter('pose_swap_roll_pitch').value
        if isinstance(pose_swap_roll_pitch, str):
            pose_swap_roll_pitch = pose_swap_roll_pitch.lower() in ('1', 'true', 'yes', 'y', 'on')
        self._pose_swap_roll_pitch = bool(pose_swap_roll_pitch)
        self._pose_roll_sign = float(self.get_parameter('pose_roll_sign').value)
        self._walk_button_index = int(self.get_parameter('walk_button_index').value)
        self._pose_button_index = int(self.get_parameter('pose_button_index').value)
        self._tl2_axis_index = int(self.get_parameter('tl2_axis_index').value)
        self._tl2_button_index = int(self.get_parameter('tl2_button_index').value)
        self._tr2_button_index = int(self.get_parameter('tr2_button_index').value)

        # ----- joystick state -----
        self._prev_buttons = []
        self._current_buttons = []
        self._prev_axes = []
        self._current_axes = []
        self._gait_types = ['trot', 'walk', 'high_walk']
        self._gait_index = 0
        self._pace_modes = ['normal', 'slow', 'high']
        self._pace_index = 0
        self._claw_closed = False
        self._arm_mode = 0  # 0x00 or 0x01
        self._attitude_adj_mode = False  # BtnMode toggle: adjust walking attitude while stationary

        # ----- robot state -----
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._battery = 0.0
        self._body_pose = [0.0, 0.0, 108.0, 0.0, 0.0, 0.0]
        self._arm_pose = [85.0, 55.0]   # [arm_x, arm_z]
        self._joint_angle = [0.0] * 15
        self._imu_angle = [0.0, 0.0, 0.0]        # roll, pitch, yaw (rad)
        self._imu_accel = [0.0, 0.0, 0.0]
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._last_time = self.get_clock().now()
        self._last_cmd_vel_time: rclpy.time.Time | None = None

        # ----- inbound packet state machine -----
        self._rx_flag = 0
        self._rx_ptr = 0
        self._rx_len = 0
        self._rx_data_len = 0
        self._rx_addr = 0
        self._rx_data = bytearray(256)

        # ----- BLE async infrastructure -----
        self._send_queue: asyncio.Queue = None  # created inside the BLE thread
        self._ble_loop = asyncio.new_event_loop()
        self._connected = threading.Event()
        self._ble_thread = threading.Thread(
            target=self._run_ble_loop, name="xgo_ble", daemon=True
        )
        self._ble_thread.start()

        # Wait up to 60 s for BLE connection (scan 10 s + connect 15 s + margin)
        if not self._connected.wait(timeout=60.0):
            self.get_logger().error(
                f"BLE connection to {self._config['bt_address']} timed out!"
            )
        else:
            self.get_logger().info("BLE connected — starting poll timer.")

        # ----- publishers -----
        self._joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self._battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ----- subscribers -----
        self.create_subscription(Twist, 'cmd_vel', self._velocity_cb, 10)
        self.create_subscription(Pose, 'body_pose', self._body_pose_cb, 10)
        self.create_subscription(Pose, 'arm_pose', self._arm_pose_cb, 10)
        self.create_subscription(Joy, 'joy', self._joy_cb, 10)

        # ----- 10 Hz state poll (BLE is request-response, not autofeedback) -----
        self._poll_timer = self.create_timer(0.1, self._poll_state)

        self.get_logger().info("XGOBTNode initialised.")

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def _load_config(self, path: str) -> dict:
        with open(path, 'r') as f:
            root = json.load(f)
        return {
            'bt_address':    root.get('bt_address', ''),
            'bt_char_write': root.get('bt_char_write', _DEFAULT_CHAR_WRITE),
            'bt_char_notify': root.get('bt_char_notify', _DEFAULT_CHAR_NOTIFY),
            'vx_max':        float(root.get('vx_max', 0.3)),
            'vy_max':        float(root.get('vy_max', 0.2)),
            'vyaw_max':      float(root.get('vyaw_max', 1.2)),
            'joint_limit':   root.get('joint_limit', []),
            'body_limit':    root.get('body_limit', []),
            'arm_limit':     root.get('arm_limit', []),
            'period_limit':  root.get('period_limit', []),
        }

    # ------------------------------------------------------------------
    # BLE async loop (runs in its own thread)
    # ------------------------------------------------------------------

    def _run_ble_loop(self):
        asyncio.set_event_loop(self._ble_loop)
        self._ble_loop.run_until_complete(self._ble_main())

    async def _ble_main(self):
        self._send_queue = asyncio.Queue()
        address = self._config['bt_address']
        char_write = self._config['bt_char_write']
        char_notify = self._config['bt_char_notify']

        while rclpy.ok():
            # --- pre-flight: check the device is visible before connecting ---
            self.get_logger().info(f"Scanning for XGO at {address} ...")
            device = await BleakScanner.find_device_by_address(address, timeout=10.0)
            if device is None:
                self.get_logger().warn(
                    f"XGO ({address}) not found in scan — retrying in 5 s ..."
                )
                await asyncio.sleep(5.0)
                continue

            self.get_logger().info(f"XGO found — connecting ...")
            try:
                async with BleakClient(address, timeout=15.0) as client:
                    self.get_logger().info("BLE connected.")
                    await client.start_notify(char_notify, self._notification_handler)
                    self._connected.set()

                    # Send loop: drain the queue while connected
                    while rclpy.ok() and client.is_connected:
                        try:
                            data = await asyncio.wait_for(
                                self._send_queue.get(), timeout=0.1
                            )
                            await client.write_gatt_char(char_write, data, response=False)
                        except asyncio.TimeoutError:
                            pass

                    self._connected.clear()
                    if rclpy.ok():
                        self.get_logger().warn("BLE connection lost — reconnecting ...")

            except Exception as exc:
                self._connected.clear()
                self.get_logger().error(f"BLE error: {exc} — retrying in 5 s ...")
                await asyncio.sleep(5.0)

    # ------------------------------------------------------------------
    # BLE notification handler → state machine (mirrors readState())
    # ------------------------------------------------------------------

    def _notification_handler(self, _sender, data: bytearray):
        """Called by bleak in the BLE thread for each incoming notification."""
        for byte in data:
            self._parse_byte(byte)

    def _parse_byte(self, byte: int):
        # Response packet format (from xgolib __unpack):
        #   0x55 0x00 LEN TYPE ADDR data[LEN-9] CHECKSUM 0x00 0xAA
        # where LEN = data_count + 9
        flag = self._rx_flag
        if flag == 0:
            if byte == 0x55:
                self._rx_flag = 1
        elif flag == 1:
            self._rx_flag = 2 if byte == 0x00 else 0
        elif flag == 2:  # LEN byte
            self._rx_len = byte
            self._rx_data_len = byte - 8  # total packet = data + 8 overhead bytes
            self._rx_flag = 3
        elif flag == 3:  # TYPE byte (ignore)
            self._rx_flag = 4
        elif flag == 4:  # ADDR byte — capture it for routing
            self._rx_addr = byte
            self._rx_flag = 5
            self._rx_ptr = 0
        elif flag == 5:  # data bytes
            self._rx_data[self._rx_ptr] = byte
            self._rx_ptr += 1
            if self._rx_ptr >= self._rx_data_len:
                self._rx_flag = 6
        elif flag == 6:  # CHECKSUM byte (skip verification)
            self._rx_flag = 7
        elif flag == 7:
            self._rx_flag = 8 if byte == 0x00 else 0
        elif flag == 8:
            if byte == 0xAA:
                self._update_state()
            self._rx_flag = 0
            self._rx_ptr = 0

    def _update_state(self):
        """Route the response payload to the correct state variable based on ADDR."""
        d = self._rx_data
        addr = self._rx_addr

        if addr == 0x01:  # battery (1 byte)
            self._battery = d[0]
            self._publish_battery()

        elif addr == 0x50:  # motor angles (15 bytes, uint8)
            joint_limit = self._config['joint_limit']
            for i in range(15):
                j_idx = i % 3 if i < 12 else i - 9
                if j_idx < len(joint_limit):
                    self._joint_angle[i] = _to_rad(
                        _uint8_to_float(d[i], joint_limit[j_idx][0], joint_limit[j_idx][1])
                    )
            self._publish_joint_state()

        elif addr == 0x62:  # IMU roll+pitch+yaw (3 × 4-byte little-endian floats)
            self._imu_angle[0] = _to_rad(struct.unpack_from('<f', d, 0)[0])
            self._imu_angle[1] = _to_rad(struct.unpack_from('<f', d, 4)[0])
            self._imu_angle[2] = _to_rad(struct.unpack_from('<f', d, 8)[0])
            self._publish_imu()
            self._publish_base_transforms()
            self._publish_odometry()

        else:
            self.get_logger().warn(f"Unhandled ADDR 0x{addr:02x} in response")

    # ------------------------------------------------------------------
    # Send helpers
    # ------------------------------------------------------------------

    def _send_order(self, addr: int, data_bytes):
        """Thread-safe: queue a WRITE packet (mode=0x01) for the BLE write char."""
        packet = _build_packet(addr, data_bytes)
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(packet), self._ble_loop
        )

    def _send_read(self, addr: int, read_len: int):
        """Thread-safe: queue a READ request (mode=0x02) for the BLE write char."""
        mode = 0x02
        checksum = (255 - (0x09 + mode + addr + read_len) % 256) & 0xFF
        packet = bytes([0x55, 0x00, 0x09, mode, addr, read_len, checksum, 0x00, 0xAA])
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(packet), self._ble_loop
        )

    def _poll_state(self):
        """Called at 10 Hz to request state from the robot via separate register reads."""
        if not self._connected.is_set():
            return
        self._send_read(0x01, 1)   # battery
        self._send_read(0x50, 15)  # joint angles (motor registers)
        self._send_read(0x62, 12)  # IMU: roll+pitch+yaw as 3 consecutive floats
        hold_walk = (
            len(self._current_buttons) > self._walk_button_index
            and self._current_buttons[self._walk_button_index] == 1
        )
        hold_pose = (
            len(self._current_buttons) > self._pose_button_index
            and self._current_buttons[self._pose_button_index] == 1
        )
        # Attitude adjustment mode (BtnMode): robot stationary, sticks set body pose for walking
        if self._attitude_adj_mode:
            self._send_body_pose()
        # While BtnTL1 (button 4) is held, send speed and body pose at 10 Hz (maintain attitude)
        elif hold_walk:
            self._send_body_pose()
            self._send_speed()
        # While BtnTR1 (button 5) is held, send body pose at 10 Hz
        elif hold_pose:
            self._send_body_pose()
        # Autonomous: if cmd_vel received recently (e.g. from pure_pursuit), send speed at 10 Hz
        elif self._last_cmd_vel_time is not None:
            age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
            if age < 0.5:
                self._send_speed()

    def _send_speed(self):
        cfg = self._config
        self._send_order(0x30, [_float_to_uint8(self._vx, -cfg['vx_max'], cfg['vx_max'])])
        self._send_order(0x31, [_float_to_uint8(self._vy, -cfg['vy_max'], cfg['vy_max'])])
        self._send_order(0x32, [_float_to_uint8(self._vyaw, -cfg['vyaw_max'], cfg['vyaw_max'])])

    def _send_body_pose(self):
        body_limit = self._config['body_limit']
        for i in range(6):
            val = _float_to_uint8(self._body_pose[i], body_limit[i][0], body_limit[i][1])
            self._send_order(0x33 + i, [val])

    def _send_arm_pose(self):
        arm_limit = self._config['arm_limit']
        self._send_order(0x73, [_float_to_uint8(self._arm_pose[0], arm_limit[0][0], arm_limit[0][1])])  # arm_x
        self._send_order(0x74, [_float_to_uint8(self._arm_pose[1], arm_limit[1][0], arm_limit[1][1])])  # arm_z

    def _action(self, action_id: int):
        self._send_order(0x3E, [action_id & 0xFF])

    def _send_gait_type(self, mode: str):
        """Send gait type: 'trot'=0x00, 'walk'=0x01, 'high_walk'=0x02."""
        value = {'trot': 0x00, 'walk': 0x01, 'high_walk': 0x02}.get(mode, 0x00)
        self._send_order(0x09, [value])

    def _send_pace(self, mode: str):
        """Send pace/move mode: 'normal'=0x00, 'slow'=0x01, 'high'=0x02."""
        value = {'normal': 0x00, 'slow': 0x01, 'high': 0x02}.get(mode, 0x00)
        self._send_order(0x3D, [value])

    def _reset(self):
        """Send the reset action (0xFF) to return the robot to its default pose."""
        self._action(0xFF)
        self._body_pose = [0.0, 0.0, 108.0, 0.0, 0.0, 0.0]

    def _send_claw(self):
        """Send claw state: 0=open, 255=closed (register 0x71)."""
        self._send_order(0x71, [255 if self._claw_closed else 0])

    def _send_arm_mode(self):
        """Send arm mode: 0x00 or 0x01 (register 0x72)."""
        self._send_order(0x72, [self._arm_mode & 0x01])

    # ------------------------------------------------------------------
    # ROS2 subscribers
    # ------------------------------------------------------------------

    def _velocity_cb(self, msg: Twist):
        cfg = self._config
        self._vx   = _limit(msg.linear.x,  -cfg['vx_max'],   cfg['vx_max'])
        self._vy   = _limit(msg.linear.y,  -cfg['vy_max'],   cfg['vy_max'])
        self._vyaw = _limit(msg.angular.z, -cfg['vyaw_max'], cfg['vyaw_max'])
        self._last_cmd_vel_time = self.get_clock().now()

    def _body_pose_cb(self, msg: Pose):
        self._body_pose[0] += msg.position.x
        self._body_pose[1] += msg.position.y
        self._body_pose[2] += msg.position.z
        self._body_pose[3] += msg.orientation.x
        self._body_pose[4] += msg.orientation.y
        self._body_pose[5] += msg.orientation.z
        self._send_body_pose()

    def _arm_pose_cb(self, msg: Pose):
        arm_limit = self._config['arm_limit']
        self._arm_pose[0] = _limit(self._arm_pose[0] + msg.position.x, arm_limit[0][0], arm_limit[0][1])  # arm_x
        self._arm_pose[1] = _limit(self._arm_pose[1] + msg.position.y, arm_limit[1][0], arm_limit[1][1])  # arm_z
        self._send_arm_pose()

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes
        prev = self._prev_buttons
        prev_axes = self._prev_axes
        self._current_buttons = list(buttons)
        self._current_axes = list(axes)

        _TRIGGER = 0.5  # threshold: trigger axis > this = pressed (axis goes -1.0→+1.0)
        _PAD_STEP = 10.0  # arm position step per d-pad press (physical units)

        hold_walk = (
            len(buttons) > self._walk_button_index
            and buttons[self._walk_button_index] == 1
        )  # BtnTL1
        hold_pose = (
            len(buttons) > self._pose_button_index
            and buttons[self._pose_button_index] == 1
        )  # BtnTR1
        prev_walk = (
            len(prev) > self._walk_button_index
            and prev[self._walk_button_index] == 1
        )
        prev_pose = (
            len(prev) > self._pose_button_index
            and prev[self._pose_button_index] == 1
        )
        suppress_button_actions = (
            self._joypad_profile == 'newpad' and (hold_walk or hold_pose)
        )

        # BtnSelect (button 6) rising edge — toggle arm mode (register 0x72)
        if (not suppress_button_actions) and len(buttons) > 6 and buttons[6] == 1 and (len(prev) <= 6 or prev[6] == 0):
            self._arm_mode = 1 - self._arm_mode
            self.get_logger().info(f'BtnSelect pressed — arm mode {self._arm_mode}.')
            self._send_arm_mode()

        # Left touchpad: axis 6 (L-R) → arm_x step, axis 7 (U-D) → arm_z step
        # D-pad gives discrete -1/0/+1; act on rising edge only
        if len(axes) > 7 and self._config.get('arm_limit'):
            arm_limit = self._config['arm_limit']
            pad6_now = round(axes[6])   # -1, 0, or +1
            pad7_now = round(axes[7])
            pad6_was = round(prev_axes[6]) if len(prev_axes) > 6 else 0
            pad7_was = round(prev_axes[7]) if len(prev_axes) > 7 else 0
            if pad6_now != 0 and pad6_now != pad6_was:
                self._arm_pose[0] = _limit(self._arm_pose[0] + pad6_now * _PAD_STEP,
                                           arm_limit[0][0], arm_limit[0][1])
                self._send_arm_pose()
            if pad7_now != 0 and pad7_now != pad7_was:
                self._arm_pose[1] = _limit(self._arm_pose[1] - pad7_now * _PAD_STEP,
                                           arm_limit[1][0], arm_limit[1][1])
                self._send_arm_pose()

        # Right-stick values for pose/attitude handling.
        right_h_val = axes[self._right_stick_h_axis] if len(axes) > self._right_stick_h_axis else 0.0
        right_v_val = axes[self._right_stick_v_axis] if len(axes) > self._right_stick_v_axis else 0.0
        if self._pose_swap_roll_pitch:
            right_h_val, right_v_val = right_v_val, right_h_val
        right_h_val *= self._pose_roll_sign

        # BtnTL2 rising edge — toggle claw.
        # Supports axis-based triggers (classic pads) and button-based TL2 (newpad).
        if self._tl2_button_index >= 0:
            tl2_now = (
                len(buttons) > self._tl2_button_index
                and buttons[self._tl2_button_index] == 1
            )
            tl2_was = (
                len(prev) > self._tl2_button_index
                and prev[self._tl2_button_index] == 1
            )
        else:
            tl2_now = (
                self._tl2_axis_index >= 0
                and len(axes) > self._tl2_axis_index
                and axes[self._tl2_axis_index] > _TRIGGER
            )
            tl2_was = (
                self._tl2_axis_index >= 0
                and len(prev_axes) > self._tl2_axis_index
                and prev_axes[self._tl2_axis_index] > _TRIGGER
            )
        if tl2_now and not tl2_was:
            self._claw_closed = not self._claw_closed
            state = 'closed' if self._claw_closed else 'open'
            self.get_logger().info(f'BtnTL2 pressed — claw {state}.')
            self._send_claw()

        # BtnStart = button index 7 — trigger on rising edge (press, not hold)
        if (not suppress_button_actions) and len(buttons) > 7 and buttons[7] == 1 and (len(prev) <= 7 or prev[7] == 0):
            self.get_logger().info('BtnStart pressed — resetting robot.')
            self._reset()

        # BtnY = button index 3 — cycle through gait types
        if (not suppress_button_actions) and len(buttons) > 3 and buttons[3] == 1 and (len(prev) <= 3 or prev[3] == 0):
            self._gait_index = (self._gait_index + 1) % len(self._gait_types)
            gait = self._gait_types[self._gait_index]
            self.get_logger().info(f'BtnY pressed — switching gait to: {gait}')
            self._send_gait_type(gait)

        # BtnX = button index 2 — cycle through pace modes
        if (not suppress_button_actions) and len(buttons) > 2 and buttons[2] == 1 and (len(prev) <= 2 or prev[2] == 0):
            self._pace_index = (self._pace_index + 1) % len(self._pace_modes)
            pace = self._pace_modes[self._pace_index]
            self.get_logger().info(f'BtnX pressed — switching pace to: {pace}')
            self._send_pace(pace)

        # BtnA = button 0 — increase stride length (pace: slow → normal → high)
        if (not suppress_button_actions) and len(buttons) > 0 and buttons[0] == 1 and (len(prev) <= 0 or prev[0] == 0):
            if self._pace_index < len(self._pace_modes) - 1:
                self._pace_index += 1
                pace = self._pace_modes[self._pace_index]
                self.get_logger().info(f'BtnA pressed — stride up: {pace}')
                self._send_pace(pace)
        # BtnB = button 1 — decrease stride length
        if (not suppress_button_actions) and len(buttons) > 1 and buttons[1] == 1 and (len(prev) <= 1 or prev[1] == 0):
            if self._pace_index > 0:
                self._pace_index -= 1
                pace = self._pace_modes[self._pace_index]
                self.get_logger().info(f'BtnB pressed — stride down: {pace}')
                self._send_pace(pace)

        # BtnMode = button index 8 — toggle attitude adjustment mode (robot stationary, sticks set pose)
        if (not suppress_button_actions) and len(buttons) > 8 and buttons[8] == 1 and (len(prev) <= 8 or prev[8] == 0):
            self._attitude_adj_mode = not self._attitude_adj_mode
            if self._attitude_adj_mode:
                self._vx = self._vy = self._vyaw = 0.0
                self._send_speed()  # stop any motion
                self.get_logger().info('BtnMode: attitude adjustment ON — use sticks to set pose.')
            else:
                self.get_logger().info('BtnMode: attitude adjustment OFF — new pose retained for walking.')

        # BtnThumbL (9) / BtnThumbR (10) — raise / lower body height (translation z)
        _HEIGHT_STEP = 5.0
        if (not suppress_button_actions) and len(self._config['body_limit']) > 2:
            z_min, z_max = self._config['body_limit'][2][0], self._config['body_limit'][2][1]
            if len(buttons) > 9 and buttons[9] == 1 and (len(prev) <= 9 or prev[9] == 0):
                self._body_pose[2] = _limit(self._body_pose[2] + _HEIGHT_STEP, z_min, z_max)
                self.get_logger().info(f'BtnThumbL — height {self._body_pose[2]:.0f} mm.')
                self._send_body_pose()
            if len(buttons) > 10 and buttons[10] == 1 and (len(prev) <= 10 or prev[10] == 0):
                self._body_pose[2] = _limit(self._body_pose[2] - _HEIGHT_STEP, z_min, z_max)
                self.get_logger().info(f'BtnThumbR — height {self._body_pose[2]:.0f} mm.')
                self._send_body_pose()

        # BtnTL1 (button 4) held — left stick drives vx/vy, right stick drives vyaw
        # Ignored when in attitude adjustment mode (robot stays stationary)
        # Values updated here; _poll_state() transmits at 10 Hz to avoid flooding BLE
        if self._attitude_adj_mode:
            body_limit = self._config['body_limit']

            def _scale(stick_val: float, limits) -> float:
                mid = (limits[0] + limits[1]) / 2.0
                half = (limits[1] - limits[0]) / 2.0
                return mid + stick_val * half

            # Left stick: axis 0 (L-R) → body y, axis 1 (U-D) → body x
            self._body_pose[0] = _scale(axes[1], body_limit[0])
            self._body_pose[1] = _scale(axes[0], body_limit[1])
            # Right stick: axis 3 (L-R) → roll, axis 4 (U-D) → pitch
            self._body_pose[3] = _scale(right_h_val, body_limit[3])
            self._body_pose[4] = _scale(right_v_val, body_limit[4])
        elif hold_walk and not self._attitude_adj_mode:
            cfg = self._config
            self._vx   = _limit(-axes[1], -cfg['vx_max'],   cfg['vx_max'])
            self._vy   = _limit( axes[0], -cfg['vy_max'],   cfg['vy_max'])
            yaw_val = axes[self._walk_yaw_axis] if len(axes) > self._walk_yaw_axis else 0.0
            self._vyaw = _limit(self._walk_yaw_sign * yaw_val, -cfg['vyaw_max'], cfg['vyaw_max'])
        elif prev_walk and not hold_walk and not self._attitude_adj_mode:
            # Button released — stop movement
            self._vx = self._vy = self._vyaw = 0.0
            self._send_speed()

        # BtnTR1 (button 5) held — left stick controls body translation, right stick controls attitude
        # Mutually exclusive with walk mode; _poll_state() transmits at 10 Hz to avoid flooding BLE
        elif hold_pose:
            body_limit = self._config['body_limit']

            def _scale(stick_val: float, limits) -> float:
                mid = (limits[0] + limits[1]) / 2.0
                half = (limits[1] - limits[0]) / 2.0
                return mid + stick_val * half

            # Left stick: axis 0 (L-R) → body y, axis 1 (U-D) → body x
            self._body_pose[0] = _scale( axes[1], body_limit[0])
            self._body_pose[1] = _scale( axes[0], body_limit[1])
            # Right stick: axis 3 (L-R) → roll, axis 4 (U-D) → pitch
            self._body_pose[3] = _scale(right_h_val, body_limit[3])
            self._body_pose[4] = _scale(right_v_val, body_limit[4])
        elif prev_pose and not hold_pose:
            # Button released — reset body pose to neutral, preserving current height
            self._body_pose = [0.0, 0.0, self._body_pose[2], 0.0, 0.0, 0.0]
            self._send_body_pose()

        self._prev_buttons = list(buttons)
        self._prev_axes = list(axes)

    # ------------------------------------------------------------------
    # ROS2 publishers
    # ------------------------------------------------------------------

    def _publish_joint_state(self):
        ja = self._joint_angle
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "left_front_hip_joint",  "left_front_thigh_joint",  "left_front_calf_joint",
            "right_front_hip_joint", "right_front_thigh_joint", "right_front_calf_joint",
            "right_back_hip_joint",  "right_back_thigh_joint",  "right_back_calf_joint",
            "left_back_hip_joint",   "left_back_thigh_joint",   "left_back_calf_joint",
            "down_arm_joint", "up_arm_joint", "gear_arm_joint",
            "right_claw_joint", "left_claw_joint",
        ]
        msg.position = [
            ja[2],  ja[1],  ja[0],
            -ja[5], ja[4],  ja[3],
            -ja[8], ja[7],  ja[6],
            ja[11], ja[10], ja[9],
            -ja[14], ja[13], ja[12],
            ja[12] / 110.0, -ja[12] / 110.0,
        ]
        self._joint_pub.publish(msg)

    def _publish_imu(self):
        roll, pitch, yaw = self._imu_angle
        q = _quaternion_from_euler(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.linear_acceleration.x = self._imu_accel[0]
        msg.linear_acceleration.y = self._imu_accel[1]
        msg.linear_acceleration.z = self._imu_accel[2]
        self._imu_pub.publish(msg)

    def _publish_battery(self):
        msg = BatteryState()
        msg.percentage = float(self._battery)
        self._battery_pub.publish(msg)

    def _publish_base_transforms(self):
        roll, pitch, yaw = self._imu_angle
        q = _quaternion_from_euler(roll, pitch, yaw)
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "odom"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = self._odom_x
        ts.transform.translation.y = self._odom_y
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(ts)

    def _publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        roll, pitch, yaw = self._imu_angle
        self._odom_x += (self._vx * math.cos(yaw) - self._vy * math.sin(yaw)) * dt
        self._odom_y += (self._vx * math.sin(yaw) + self._vy * math.cos(yaw)) * dt

        q = _quaternion_from_euler(roll, pitch, yaw)
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.position.z = 0.12
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self._vx
        msg.twist.twist.linear.y = self._vy
        msg.twist.twist.angular.z = self._vyaw
        self._odom_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = XGOBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
