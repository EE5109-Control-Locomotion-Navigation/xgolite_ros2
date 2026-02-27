# Joypad Mapping (xgo_ros)

Raw button/axis indices and their functions in the XGO control node.

## Button indices

| Index | Name      | Function                                                                 |
|-------|-----------|--------------------------------------------------------------------------|
| 0     | BtnA      | Increase stride length (pace: slow → normal → high)                      |
| 1     | BtnB      | Decrease stride length (pace: high → normal → slow)                      |
| 2     | BtnX      | Cycle pace mode: normal → slow → high                                    |
| 3     | BtnY      | Cycle gait type: trot → walk → high_walk                                 |
| 4     | BtnTL1    | Hold: walk mode — left stick = vx/vy, right stick = vyaw                 |
| 5     | BtnTR1    | Hold: pose mode — left stick = body x/y, right stick = roll/pitch        |
| 6     | BtnSelect | Toggle arm mode                                                          |
| 7     | BtnStart  | Reset robot to default pose                                              |
| 8     | BtnMode   | Toggle attitude adjustment: adjust walking pose while stationary (sticks)  |
| 9     | BtnThumbL | Raise body height (+5 mm)                                                |
| 10    | BtnThumbR | Lower body height (−5 mm)                                                |

## Axis indices

| Index | Name           | Function                                                                 |
|-------|----------------|--------------------------------------------------------------------------|
| 0     | Left stick L-R | Walk: vy — Pose: body y                                                  |
| 1     | Left stick U-D | Walk: vx — Pose: body x                                                  |
| 2     | BtnTL2         | Press: toggle claw open/closed (threshold > 0.5)                         |
| 3     | Right stick L-R| Walk: vyaw — Pose: roll                                                  |
| 4     | Right stick U-D| Pose: pitch (only when BtnTR1 held)                                      |
| 5     | BtnTR2         | (unused)                                                                 |
| 6     | Left Pad L-R   | Arm x position step (discrete -1/0/+1 per press)                         |
| 7     | Left Pad U-D   | Arm z position step (discrete -1/0/+1 per press)                         |

## Modes

- **Walk** (BtnTL1 held): Left stick controls linear velocity, right stick controls yaw. Speed sent at 10 Hz while held. Uses the current body attitude (set via BtnMode or BtnTR1).
- **Pose** (BtnTR1 held): Left stick controls body x/y translation, right stick controls roll/pitch. On release, pose resets to neutral (height preserved).
- **Attitude adjustment** (BtnMode toggle): Robot stays stationary. Left stick = body x/y, right stick = roll/pitch. Toggle off to retain the new attitude; it is then used when walking.
