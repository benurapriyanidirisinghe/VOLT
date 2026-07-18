# VOLT Servo Calibration

This workflow maps canonical ROS joint radians to physical PCA9685 servo
channels. Suspend the robot before any hardware test.

## Safe Workflow

1. Suspend the robot and remove ground load from every foot.
2. Confirm Jetson, Arduino, PCA9685, and servo supply share ground.
3. Confirm the servo power supply is stable and not powered from the Arduino.
4. Upload `firmware/volt_arduino_pca9685/volt_arduino_pca9685.ino`.
5. Open serial and confirm startup:
   `OK VOLT_PCA9685_READY DISARMED OUTPUT_DISABLED`.
6. Send `PING`; expect `OK PONG`.
7. Start calibration dry-run first:
   `ros2 launch volt_description servo_calibration.launch.py dry_run:=true use_hardware:=false`.
8. Start hardware only after dry-run conversion looks correct:
   `ros2 launch volt_description servo_calibration.launch.py serial_port:=/dev/ttyUSB0 use_hardware:=true dry_run:=false`.
9. In physical-channel mode, press `ARM` only when ready.
10. Use `SERVO channel degrees` controls to identify each PCA channel.
11. Assign each physical channel to one logical joint and save mapping.
12. In ROS joint calibration mode, set each `neutral_deg` to the servo command
    that places that real joint at URDF zero radians.
13. Test `+0.05 rad`; compare real link motion with Gazebo, not servo angle.
14. If real link motion is opposite Gazebo, set `direction` to `-1`.
15. Test `-0.05 rad` and return to zero.
16. Set conservative `min_deg` and `max_deg`.
17. Restart dry-run and verify the logged conversion table.
18. Enable hardware and test one joint at a time.
19. Test a complete static crouched pose.
20. Only after stable standing should gait control be enabled.

## Arduino Manual Serial Test

Compile and upload:

```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/volt_arduino_pca9685
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano firmware/volt_arduino_pca9685
```

For old Nano bootloaders, use:

```text
arduino:avr:nano:cpu=atmega328old
```

Manual test:

```text
PING
ARM
SERVO 0 90
STATUS
HOLD
DISARM
DISABLE
```

Only PCA channel `0` should move for `SERVO 0 90`.

Front foot channel check:

```text
ARM
SERVO 2 0
SERVO 2 20
SERVO 2 0
SERVO 5 180
SERVO 5 160
SERVO 5 180
```

Expected with the current initial mapping:

- `SERVO 2 ...` moves only `front_left_foot`.
- `SERVO 5 ...` moves only `front_right_foot`.

If either front foot does not move in this direct channel test, the issue is
not Gazebo or joint direction; identify the real PCA channel with `SERVO
channel degrees` and update `pca_channel` in `config/servo_calibration.yaml`.

## Architecture

Canonical command representation:

```text
named ROS joint positions in radians using URDF convention
```

Flow:

```text
motion/manual/calibration source
-> /volt/joint_commands/<source>
-> volt_joint_command_router.py
-> /joint_group_position_controller/commands for Gazebo
-> /joint_command_router/output for serial bridge
-> volt_serial_bridge.py converts once
-> FRAME in PCA channel order
-> Arduino physical output
```

Neutral, trim, direction, and PCA channel mapping live in:

```text
src/volt_description/config/servo_calibration.yaml
```

The Arduino no longer applies ROS radians, neutral offsets, trims, or logical
joint-to-channel mapping.

## Troubleshooting

- Wrong servo moves: use physical-channel mode and correct `pca_channel`.
- Correct servo moves wrong way: compare real link with Gazebo for `+0.05 rad`;
  invert `direction`.
- Servo hits a limit: reduce `min_deg`/`max_deg`, check neutral and direction.
- All servos move while testing one joint: make sure only the router publishes
  to `/joint_group_position_controller/commands`.
- Gazebo moves but hardware does not: check `dry_run`, `hardware_enabled`,
  serial port, `ARM`, and `/volt/serial_status`.
- Hardware moves but Gazebo does not: check the controller subscriber and router
  status.
- Commands return to center after a pause: old firmware is still uploaded; new
  firmware holds on timeout and `DISARM`.
- Duplicate publishers: run
  `ros2 topic info /joint_group_position_controller/commands -v`.
- Serial buffer backlog: keep `ACK_FRAME_COMMANDS` false in firmware.
- Malformed packets: firmware rejects bad counts, bad channels, `nan`, `inf`,
  and partial numbers.
- Incorrect `/joint_states` order: all calibration code reconstructs order by
  joint names.
