# XGOlib.py Documentation

## Overview

**Version:** 1.2.0  
**Last Modified:** 2023/2/16

This library provides a Python interface for controlling XGO series robot dogs (XGO Mini and XGO Lite) via serial communication. It enables control of movement, posture, servos, LED, arm mechanics, and various other robot functions.

---

## Table of Contents

1. [Global Data Structures](#global-data-structures)
2. [Utility Functions](#utility-functions)
3. [XGO Class](#xgo-class)
4. [Movement Methods](#movement-methods)
5. [Position & Attitude Control](#position--attitude-control)
6. [Motor Control](#motor-control)
7. [Periodic Motion](#periodic-motion)
8. [Robot Configuration](#robot-configuration)
9. [Data Reading Methods](#data-reading-methods)
10. [Advanced/Experimental Methods](#advancedexperimental-methods)

---

## Global Data Structures

### XGOorder Dictionary

Stores command addresses and corresponding data for robot control. Each command has an address (first element) followed by data bytes.

**Key Commands:**
- `BATTERY`: Battery status query
- `MOVE_TEST`, `VX`, `VY`, `VYAW`: Movement controls
- `TRANSLATION`, `ATTITUDE`: Body position/orientation
- `MOTOR_ANGLE`, `MOTOR_SPEED`: Servo control
- `LEG_POS`: Individual leg positioning
- `IMU`, `ROLL`, `PITCH`, `YAW`: IMU sensor data
- `CLAW`, `ARM_MODE`, `ARM_X`, `ARM_Z`: Arm/gripper control
- And more...

### XGOparam Dictionary

Defines the operational parameter limits for the robot dog.

**Parameters:**
- `TRANSLATION_LIMIT`: [X, Y, Z] translation ranges in mm
- `ATTITUDE_LIMIT`: [Roll, Pitch, Yaw] attitude ranges in degrees
- `LEG_LIMIT`: Leg position ranges
- `MOTOR_LIMIT`: Servo angle limits for 6 joint types
- `PERIOD_LIMIT`: Periodic motion period range
- `MARK_TIME_LIMIT`: Mark time height range
- `VX_LIMIT`, `VY_LIMIT`, `VYAW_LIMIT`: Movement velocity limits
- `ARM_LIMIT`: Arm positioning limits

**Note:** Parameters differ between XGO Mini and XGO Lite versions.

---

## Utility Functions

### `search(data, list)`

Searches for an element in a list and returns its 1-based index.

**Parameters:**
- `data`: Element to search for
- `list`: List to search in

**Returns:**
- Index + 1 if found (1-based indexing)
- -1 if not found

---

### `conver2u8(data, limit, mode=0)`

Converts actual physical parameters to single-byte (0-255) values for serial communication.

**Parameters:**
- `data`: The actual value to convert
- `limit`: Either a single number (symmetric limit) or `[min, max]` list
- `mode`: 
  - `0` (default): Maps range to 0-255
  - `1`: Maps range to 1-255

**Returns:**
- Integer value between 0-255 (or 1-255 if mode=1)

**Logic:**
- For symmetric limits: Center is 128, scales proportionally
- For asymmetric limits: Linear mapping from `[min, max]` to `[0, 255]`
- Clamps values that exceed limits

---

### `conver2float(data, limit)`

Converts a byte value (0-255) back to the original float value.

**Parameters:**
- `data`: Byte value (0-255)
- `limit`: Either a single number or `[min, max]` list

**Returns:**
- Float value representing the actual parameter

---

### `Byte2Float(rawdata)`

Converts 4 bytes to a float using IEEE 754 format.

**Parameters:**
- `rawdata`: 4-byte array

**Returns:**
- Float value

**Note:** Reverses byte order for proper float reconstruction.

---

### `changePara(version)`

Updates the global `XGOparam` dictionary based on robot version.

**Parameters:**
- `version`: String, either `'xgomini'` or `'xgolite'`

**Effect:**
- Modifies global `XGOparam` with version-specific limits
- XGO Lite has more restricted ranges than XGO Mini

---

## XGO Class

### `__init__(self, port, baud=115200, version='xgomini')`

Initializes the XGO robot connection.

**Parameters:**
- `port`: Serial port name (e.g., '/dev/ttyUSB0', 'COM3')
- `baud`: Baud rate (default: 115200)
- `version`: Robot version - `'xgomini'` or `'xgolite'` (default: 'xgomini')

**Actions:**
- Opens serial connection
- Clears input/output buffers
- Sets parameter limits based on version
- Initializes internal state variables

---

### Private Methods

#### `__send(self, key, index=1, len=1)`

Sends a command to the robot over serial.

**Parameters:**
- `key`: Command key from XGOorder dictionary
- `index`: Starting index in the command data array
- `len`: Length of data to send

**Protocol:**
- Constructs packet: `[0x55, 0x00, length, mode, order, data..., checksum, 0x00, 0xAA]`
- Calculates checksum
- Prints transmitted data for debugging

---

#### `__read(self, addr, read_len=1)`

Sends a read request to the robot.

**Parameters:**
- `addr`: Address to read from
- `read_len`: Number of bytes to read

---

#### `__change_baud(self, baud)`

Changes the serial communication baud rate.

**Parameters:**
- `baud`: New baud rate

---

#### `__unpack(self, timeout=1)`

Unpacks received serial data using a state machine parser.

**Parameters:**
- `timeout`: Maximum time to wait for response (seconds)

**Returns:**
- `True` if valid packet received
- `False` if timeout or invalid packet

**Protocol:**
- Validates packet structure
- Verifies checksum
- Stores data in `self.rx_data`

---

## Movement Methods

### `stop(self)`

Stops all robot movement.

**Actions:**
- Sets X velocity to 0
- Sets Y velocity to 0
- Stops mark time
- Stops turning

---

### `move(self, direction, step)`

Generic movement command.

**Parameters:**
- `direction`: 'x'/'X' for forward/back, 'y'/'Y' for left/right
- `step`: Movement speed (-limit to +limit)

---

### `move_x(self, step)`

Moves the robot forward (positive) or backward (negative).

**Parameters:**
- `step`: Speed value (-VX_LIMIT to +VX_LIMIT)
  - Positive: Forward
  - Negative: Backward

---

### `move_y(self, step)`

Moves the robot left (positive) or right (negative).

**Parameters:**
- `step`: Speed value (-VY_LIMIT to +VY_LIMIT)
  - Positive: Left
  - Negative: Right

---

### `turn(self, step)`

Rotates the robot.

**Parameters:**
- `step`: Rotation speed (-VYAW_LIMIT to +VYAW_LIMIT)
  - Positive: Counter-clockwise
  - Negative: Clockwise

---

### `forward(self, step)`

Moves the robot forward.

**Parameters:**
- `step`: Forward speed (always uses absolute value)

---

### `back(self, step)`

Moves the robot backward.

**Parameters:**
- `step`: Backward speed (always uses absolute value)

---

### `left(self, step)`

Moves the robot left.

**Parameters:**
- `step`: Left speed (always uses absolute value)

---

### `right(self, step)`

Moves the robot right.

**Parameters:**
- `step`: Right speed (always uses absolute value)

---

### `turnleft(self, step)`

Turns the robot counter-clockwise.

**Parameters:**
- `step`: Turn speed (always uses absolute value)

---

### `turnright(self, step)`

Turns the robot clockwise.

**Parameters:**
- `step`: Turn speed (always uses absolute value)

---

## Position & Attitude Control

### `translation(self, direction, data)`

Performs body translation while keeping feet stationary.

**Parameters:**
- `direction`: Single character ('x', 'y', 'z') or list of characters
- `data`: Translation value(s) in mm, or list of values

**Examples:**
```python
dog.translation('x', 10)  # Move body 10mm forward
dog.translation(['x', 'y', 'z'], [10, -5, 90])  # Multiple axes
```

---

### `attitude(self, direction, data)`

Performs body rotation while keeping feet stationary.

**Parameters:**
- `direction`: Single character ('r'=Roll, 'p'=Pitch, 'y'=Yaw) or list
- `data`: Rotation angle(s) in degrees, or list of values

**Examples:**
```python
dog.attitude('r', 15)  # Roll 15 degrees
dog.attitude(['r', 'p'], [10, -5])  # Multiple axes
```

---

### `action(self, action_id)`

Executes a preset action.

**Parameters:**
- `action_id`: Integer (1-255)
  - 255: Reset action (returns to initial state)
  - Other IDs: Robot-specific preset motions

---

### `reset(self)`

Stops all movement and returns robot to initial state.

**Actions:**
- Calls `action(255)`
- Waits 200ms

---

### `leg(self, leg_id, data)`

Controls a single leg's position in 3D space.

**Parameters:**
- `leg_id`: Leg number (1-4)
  - 1: Front-right
  - 2: Front-left
  - 3: Rear-left
  - 4: Rear-right
- `data`: `[x, y, z]` position in mm

---

### `mark_time(self, data)`

Makes the robot march in place.

**Parameters:**
- `data`: Step height
  - 0: Stop marking time
  - Positive value: Height of step (within MARK_TIME_LIMIT)

---

## Motor Control

### `motor(self, motor_id, data)`

Controls individual servo motors.

**Parameters:**
- `motor_id`: Motor ID or list of IDs
  - Format: XY where X=leg (1-4), Y=joint (1=lower, 2=middle, 3=upper)
  - 51, 52, 53: Arm servos
- `data`: Angle in degrees, or list of angles

**Valid Motor IDs:**
- 11, 12, 13: Leg 1 (front-right)
- 21, 22, 23: Leg 2 (front-left)
- 31, 32, 33: Leg 3 (rear-left)
- 41, 42, 43: Leg 4 (rear-right)
- 51: Claw/gripper
- 52, 53: Arm servos

**Examples:**
```python
dog.motor(11, 45)  # Single motor
dog.motor([11, 12, 13], [30, -20, 15])  # Multiple motors
```

---

### `motor_speed(self, speed)`

Sets the servo rotation speed.

**Parameters:**
- `speed`: Speed value (1-255)
  - Higher values = faster rotation
  - Only affects individual motor control, not gait control

---

### `unload_motor(self, leg_id)`

Disables (unloads) motors for a specific leg.

**Parameters:**
- `leg_id`: Leg number (1-4) or 5 for arm
  - Allows manual positioning

---

### `unload_allmotor(self)`

Disables all motors.

**Use Case:**
- Manual manipulation
- Power saving
- Emergency stop

---

### `load_motor(self, leg_id)`

Enables (loads) motors for a specific leg.

**Parameters:**
- `leg_id`: Leg number (1-4) or 5 for arm

---

### `load_allmotor(self)`

Enables all motors.

---

## Periodic Motion

### `periodic_rot(self, direction, period)`

Makes the robot body rotate periodically.

**Parameters:**
- `direction`: 'r' (Roll), 'p' (Pitch), or 'y' (Yaw), or list
- `period`: Oscillation period in seconds (1.5-8), or list
  - 0: Stop periodic rotation

**Example:**
```python
dog.periodic_rot('r', 3)  # Roll with 3-second period
dog.periodic_rot(['r', 'p'], [3, 4])  # Multiple axes
```

---

### `periodic_tran(self, direction, period)`

Makes the robot body translate periodically.

**Parameters:**
- `direction`: 'x', 'y', or 'z', or list
- `period`: Oscillation period in seconds (1.5-8), or list
  - 0: Stop periodic translation

---

## Robot Configuration

### `pace(self, mode)`

Changes the step frequency during movement.

**Parameters:**
- `mode`: String
  - `"normal"`: Standard pace
  - `"slow"`: Slower pace
  - `"high"`: Faster pace

---

### `gait_type(self, mode)`

Changes the gait pattern.

**Parameters:**
- `mode`: String
  - `"trot"`: Trot gait (default)
  - `"walk"`: Walk gait
  - `"high_walk"`: High walk gait

---

### `imu(self, mode)`

Enables/disables the IMU-based self-stabilization.

**Parameters:**
- `mode`: Integer
  - `0`: Disable IMU stabilization
  - `1`: Enable IMU stabilization

---

### `perform(self, mode)`

Enables/disables automatic action performance loop.

**Parameters:**
- `mode`: Integer
  - `0`: Disable performance mode
  - `1`: Enable performance mode (cycles through actions)

---

### `bt_rename(self, name)`

Renames the robot's Bluetooth identifier.

**Parameters:**
- `name`: String (max 10 ASCII characters)

**Constraints:**
- Must be ASCII characters only
- Maximum 10 characters

---

## Data Reading Methods

### `read_motor(self)`

Reads all 15 servo motor angles.

**Returns:**
- List of 15 float values (motor angles in degrees)
- Empty list if read fails

---

### `read_battery(self)`

Reads the battery level.

**Returns:**
- Integer (0-100) representing battery percentage
- 0 if read fails

---

### `read_firmware(self)`

Reads the firmware version string.

**Returns:**
- String with firmware version
- 'Null' if read fails

---

### `read_roll(self)`

Reads the Roll angle from IMU.

**Returns:**
- Float value in degrees
- 0 if read fails

---

### `read_pitch(self)`

Reads the Pitch angle from IMU.

**Returns:**
- Float value in degrees
- 0 if read fails

---

### `read_yaw(self)`

Reads the Yaw angle from IMU.

**Returns:**
- Float value in degrees
- 0 if read fails

---

### `read_lib_version(self)`

Returns the library version.

**Returns:**
- String with library version number

---

## Advanced/Experimental Methods

### `arm(self, arm_x, arm_z)`

Controls the robot's mechanical arm position.

**Parameters:**
- `arm_x`: Forward/backward position (within ARM_LIMIT[0])
- `arm_z`: Up/down position (within ARM_LIMIT[1])

**Note:** Only available on robots with arm attachment.

---

### `arm_mode(self, mode)`

Sets the arm control mode.

**Parameters:**
- `mode`: Integer
  - `0x00`: One mode
  - `0x01`: Another mode

---

### `claw(self, pos)`

Controls the gripper/claw position.

**Parameters:**
- `pos`: Position value (0-255)
  - 0: Fully open
  - 255: Fully closed

---

### `calibration(self, state)`

**⚠️ WARNING: Use with extreme caution!**

Initiates software calibration mode.

**Parameters:**
- `state`: String
  - `'start'`: Begin calibration
  - `'end'`: End calibration

**Note:** Improper use can damage the robot. Consult manual before using.

---

### `upgrade(self, filename)`

**⚠️ EXPERIMENTAL: Do not use in production.**

Attempts firmware upgrade via serial.

**Parameters:**
- `filename`: Path to binary firmware file

**Process:**
- Switches to 350000 baud
- Uploads firmware file
- Returns to 115200 baud

---

## Usage Examples

### Basic Movement
```python
from xgolib import XGO

# Initialize robot
dog = XGO(port='/dev/ttyUSB0', version='xgolite')

# Move forward
dog.forward(15)
time.sleep(2)

# Stop
dog.stop()
```

### Body Control
```python
# Tilt body
dog.attitude('r', 10)  # Roll 10 degrees
dog.translation('z', 95)  # Lift body to 95mm height

# Reset to neutral
dog.reset()
```

### Motor Control
```python
# Set motor speed
dog.motor_speed(50)

# Control individual leg motors
dog.motor([11, 12, 13], [30, -20, 10])
```

### Reading Sensors
```python
# Read battery
battery = dog.read_battery()
print(f"Battery: {battery}%")

# Read IMU data
roll = dog.read_roll()
pitch = dog.read_pitch()
yaw = dog.read_yaw()
print(f"Orientation: R={roll}, P={pitch}, Y={yaw}")
```

---

## Communication Protocol

The library uses a custom serial protocol:

**TX Packet Structure:**
```
[0x55, 0x00, length, mode, address, data..., checksum, 0x00, 0xAA]
```

**RX Packet Structure:**
```
[0x55, 0x00, length, type, address, data..., checksum, 0x00, 0xAA]
```

- **Checksum:** `255 - (sum of length, mode/type, address, data) % 256`
- **Mode:** `0x01` for write, `0x02` for read

---

## Version Differences

### XGO Mini vs XGO Lite

| Parameter | XGO Mini | XGO Lite |
|-----------|----------|----------|
| X Translation | ±35mm | ±25mm |
| Z Height Range | 75-115mm | 60-110mm |
| Roll Limit | ±20° | ±20° |
| Pitch Limit | ±15° | ±10° |
| Yaw Limit | ±11° | ±12° |
| Mark Time Height | 10-35mm | 10-25mm |

Motor limits also differ between versions.

---

## Troubleshooting

### Common Issues

1. **Robot not responding:**
   - Check serial port connection
   - Verify correct port name
   - Ensure robot is powered on
   - Check baud rate (should be 115200)

2. **Erratic movement:**
   - Call `reset()` to return to initial state
   - Check battery level
   - Verify parameter values are within limits

3. **Read functions return 0 or 'Null':**
   - Increase timeout in `__unpack()` method
   - Check serial buffer isn't full
   - Ensure robot firmware is compatible

---

## Notes

- Always call `stop()` or `reset()` before ending your program
- Parameter values exceeding limits are automatically clamped
- The library prints TX/RX data for debugging (can be commented out)
- Some functions may not be available on all robot versions
- IMU data reading requires firmware support

---

## Credits

This library is designed for XGO series quadruped robots manufactured by Luwu Intelligence.

For more information, visit: [https://www.luwudynamics.com/](https://www.luwudynamics.com/)
