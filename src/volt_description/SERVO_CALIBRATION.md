# VOLT Servo Calibration

This guide maps canonical ROS joint radians to physical PCA9685 servo channels.
The gait, kinematics, router, Gazebo position controller, and serial bridge all
share the same 12-value ROS-radian command. Only the serial calibration layer
converts that command to physical degrees.

In normal motion mode, conversion alone does not authorize physical output.
The bridge requires fresh router status with owner `MOTION` as well as the
firmware readiness/ARM handshake. Ownership departure or staleness inhibits
frames and requests firmware `HOLD`; `/volt/serial_status` reports
`owner_fresh`, `owner_age`, and `owner_allowed`.

Suspend the robot before every physical calibration or first-motion test. Keep
the servo-power disconnect within reach, and never power the servos from the
Arduino.

The current firmware source uses `MAX_DEG_PER_SECOND = 240.0` as a fault
ceiling, not a shaping filter; it has not been physically validated. The gait
engine keeps commanded joint speeds well below it by validating every gait
configuration at load against its servo budgets: 80 deg/s on loaded stance
joints and 190 deg/s on the unloaded swing leg. If the Nano still contains an
older low-slew firmware image, reflash the current source before live
walking. If it already contains the current image, Python or host-side gait
changes do not by themselves require another upload. Do not raise the
firmware ceiling.

The hardware-only open-loop seed and firmware safe-start frame both represent
the calibrated standing pose. Before first PWM/ARM, support the robot and
verify the real servo shafts/joints correspond to that origin; software cannot
measure the unpowered mechanism, and a mismatched mechanical pose can still
jump when pulses are first enabled.

## First physical test checklist

Use this exact process:

1. Place the robot on a raised test stand.
2. Keep all feet clear of the floor.
3. Keep the servo-power disconnect accessible.
4. Start with `hardware_enabled:=false`.
5. Start with `dry_run:=true`.
6. Confirm `use_sim_time:=false`.
7. Confirm all 12 joint names.
8. Confirm all 12 channel mappings.
9. Confirm every joint direction individually.
10. Confirm right-side inversion occurs only in calibration.
11. Select `AMBLE`.
12. Set speed to 10–20%.
13. Enable command owner `MOTION`.
14. Test step-in-place in dry-run.
15. Enable hardware explicitly.
16. ARM Arduino explicitly.
17. Test step-in-place while raised.
18. Test one very slow complete walking cycle.
19. Send HOLD.
20. Lower the robot only after all movements are verified.

This general calibration checklist does not authorize floor testing. Keep
the robot raised and follow the `REAL_DIAGNOSTIC` then `REAL_SAFE` gait
progression in [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md). No gait or profile
change requires a neutral, trim, direction, channel, or physical-limit change
in `config/servo_calibration.yaml`.

## Canonical pipeline and order

```text
PyQt GUI / gamepad
    -> /cmd_vel, /volt/action, /volt/gait, /volt/body_pose
    -> volt_motion_controller
    -> /volt/joint_commands/motion
    -> volt_joint_command_router
    -> /joint_command_router/output (12 canonical ROS radians)
       |-> /joint_group_position_controller/commands -> Gazebo Sim
       `-> volt_serial_bridge
           -> neutral + trim + direction conversion and clamping
           -> PCA-channel ordering
           -> FRAME d0 d1 ... d11
           -> Arduino Nano -> PCA9685 -> servos
```

The router validates the array once and sends the same canonical radians to
simulation and the bridge. The serial bridge reconstructs named positions using
this exact order:

```text
front_left_shoulder
front_left_leg
front_left_foot
front_right_shoulder
front_right_leg
front_right_foot
rear_left_shoulder
rear_left_leg
rear_left_foot
rear_right_shoulder
rear_right_leg
rear_right_foot
```

Do not reorder this list. It is `volt_kinematics.JOINT_NAMES`, the position
controller order, router order, calibration `joint_order`, and serial input
order. PCA channel order is different and is created only after named
calibration.

## Exact conversion and inversion ownership

For each named joint, the serial bridge performs exactly:

```text
servo_deg = neutral_deg + trim_deg + direction * degrees(ros_joint_radians)
servo_deg = clamp(servo_deg, min_deg, max_deg)
```

It then places `servo_deg` at that joint's configured `pca_channel`.

`direction` is applied exactly once, in
`config/servo_calibration.yaml`. That file owns all mechanical polarity,
including any right-side inversion. The gait controller and IK produce
canonical URDF radians; the router copies them unchanged; the Arduino does not
invert them. Do not add another sign change to gait code, IK, the router,
`FRAME`, or firmware.

Physical leveling corrections belong in per-joint `trim_deg`. Do not put them
in `WALK_POSE`, `NOMINAL_FEET`, or emote poses: those are canonical URDF values,
and changing them also changes Gazebo. Runtime output remains clamped even when
a trim makes URDF zero physically unreachable.

The authoritative file is:

```text
src/volt_description/config/servo_calibration.yaml
```

It owns:

- Exact canonical `joint_order`
- PCA channel
- Servo direction
- Neutral angle
- Mechanical trim
- Minimum and maximum physical angle
- Minimum and maximum pulse-width metadata

An invalid joint set, duplicate/missing channel, non-finite value, invalid
direction, or unsafe range prevents valid hardware output.

## Current PCA mapping

The current local mapping is shown below. All pulse metadata is currently
`600..2400 us`; the YAML remains authoritative if values are deliberately
recalibrated.

| PCA channel | Canonical joint | `direction` | `neutral_deg` | `trim_deg` | Safe degrees |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0 | `front_right_shoulder` | -1 | 120.00 | +0.02 | 70..160 |
| 1 | `front_left_leg` | +1 | 60.00 | -0.01 | 0..180 |
| 2 | `front_left_foot` | -1 | 0.00 | +0.22 | 0..150 |
| 3 | `front_left_shoulder` | -1 | 120.00 | -0.02 | 70..160 |
| 4 | `front_right_leg` | -1 | 120.00 | +0.01 | 0..180 |
| 5 | `front_right_foot` | +1 | 180.00 | -7.15 | 30..180 |
| 6 | `rear_right_shoulder` | -1 | 104.15 | +0.02 | 50..140 |
| 7 | `rear_left_leg` | +1 | 60.00 | +0.02 | 0..180 |
| 8 | `rear_left_foot` | +1 | 180.00 | +9.18 | 30..180 |
| 9 | `rear_left_shoulder` | -1 | 93.10 | -0.02 | 50..140 |
| 10 | `rear_right_leg` | -1 | 120.00 | -0.02 | 0..180 |
| 11 | `rear_right_foot` | -1 | 0.00 | -0.01 | 0..150 |

Confirm this mapping against the physical wiring; documentation is not a
substitute for identifying each channel on a raised robot.

## `FRAME` protocol

The face-enabled bridge and firmware use 57600 baud and newline-terminated
ASCII. Reflash the firmware and rebuild/restart the ROS stack together whenever
updating between revisions so both ends retain the same baud rate:

```text
FRAME d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
SERVO channel degrees
PING
ARM
HOLD
DISARM
DISABLE
STATUS
```

`FRAME` always contains:

- Exactly 12 finite physical servo angles in degrees
- One value for each PCA9685 channel `0..11`
- Values already calibrated and safely clamped by the ROS bridge
- Whole-degree wire tokens, rounded by the bridge

It never contains radians and is not in canonical joint order. Do not send a
canonical joint array directly to the Arduino. The Arduino remains a low-level
output device and contains no gait, IK, neutral/trim, logical joint mapping, or
right-side inversion.

`SERVO channel degrees` is a suspended calibration command for one physical PCA
channel. It also uses physical degrees and requires the firmware to be armed.

## 30 Hz serial dry-run

The bridge defaults to `max_send_rate:=30.0`. Dry-run performs the complete
name, formula, clamp, channel-order, and `FRAME` formatting path, but never
opens the serial port when either `dry_run:=true` or hardware is disabled.

The easiest walking dry-run uses headless Gazebo for joint feedback, the VOLT
GUI, and the serial bridge in safe mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch volt_description volt_start.launch.py \
  gui:=true \
  gazebo_gui:=false \
  start_serial_bridge:=true \
  use_hardware:=false \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=true
```

Verify the conversion:

1. Confirm the GUI says `DRY RUN — NO SERVO OUTPUT`.
2. Select `VOLT STABLE WALK` and a 10–20% speed limit.
3. Press `ENABLE MOTION`, then `STAND`, and wait for `STANDING`.
4. Observe one canonical router output:

   ```bash
   ros2 topic echo /joint_command_router/output --once
   ```

5. Confirm it contains exactly 12 finite radians in canonical order.
6. Press `STEP IN PLACE`; inspect the serial bridge log for `Dry-run
   conversion` tables and `FRAME` lines.
7. Inspect bridge state and clamping:

   ```bash
   ros2 topic echo /volt/serial_status --once
   ```

8. Confirm `dry_run=1`, `hardware_enabled=0`, `connected=0`, a valid
   12-value `frame=`, and an empty `clamped=` field for the nominal standing
   pose.
9. Confirm no more than 30 frames per second are formatted/sent by the bridge.
10. Press `STOP`, `HOLD`, and stop the launch.

For hardware-time validation without Gazebo, use:

```bash
ros2 launch volt_description hardware_control.launch.py \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false
```

No `/clock` is required, the serial port remains closed, and controller/bridge
status and timeouts use system time. Since hobby servos provide no measured
`/joint_states`, this explicit hardware-only profile seeds the canonical
`WALK_POSE` that matches firmware `CHANNEL_SAFE_START_DEG`. Treat the pose as
an open-loop assumption and verify its 12 values in dry-run before enabling
either physical-output gate. Simulation does not use this assumption.

## Example standing-pose conversion

The following reproducible example uses the current canonical `WALK_POSE` and
current calibration. No value is clamped.

| PCA channel | Joint | ROS radians | Physical degrees | Wire token |
| ---: | --- | ---: | ---: | ---: |
| 0 | `front_right_shoulder` | -0.049620338 | 122.863 | 123 |
| 1 | `front_left_leg` | +0.499194867 | 88.592 | 89 |
| 2 | `front_left_foot` | -1.081211653 | 62.169 | 62 |
| 3 | `front_left_shoulder` | +0.049620338 | 117.137 | 117 |
| 4 | `front_right_leg` | +0.499194867 | 91.408 | 91 |
| 5 | `front_right_foot` | -1.081211653 | 110.901 | 111 |
| 6 | `rear_right_shoulder` | -0.049620338 | 107.013 | 107 |
| 7 | `rear_left_leg` | +0.695626658 | 99.876 | 100 |
| 8 | `rear_left_foot` | -1.081211653 | 127.231 | 127 |
| 9 | `rear_left_shoulder` | +0.049620338 | 90.237 | 90 |
| 10 | `rear_right_leg` | +0.695626658 | 80.124 | 80 |
| 11 | `rear_right_foot` | -1.081211653 | 61.939 | 62 |

The corresponding wire packet is:

```text
FRAME 123 89 62 117 91 111 107 100 127 90 80 62
```

Use this as a software-path verification value, not as permission to energize
an unverified robot. If the YAML changes intentionally, regenerate the table
from the new calibration rather than forcing the servos to match this old line.

## Detailed safe calibration workflow

1. Suspend the robot and remove ground load from every foot.
2. Confirm Jetson, Arduino, PCA9685, and servo supply share ground.
3. Confirm the servo power supply is stable and not powered from the Arduino.
4. Upload `firmware/volt_arduino_pca9685/volt_arduino_pca9685.ino`.
5. Open serial and confirm startup:
   `OK VOLT_PCA9685_READY DISARMED OUTPUT_DISABLED`.
6. Send `PING`; expect `OK PONG`.
7. Start calibration dry-run first:

   ```bash
   ros2 launch volt_description servo_calibration.launch.py \
     use_gazebo:=true \
     use_hardware:=false \
     dry_run:=true \
     max_send_rate:=30.0
   ```

8. Start physical calibration only after dry-run conversion looks correct:

   ```bash
   ros2 launch volt_description servo_calibration.launch.py \
     serial_port:=/dev/ttyUSB0 \
     use_gazebo:=false \
     use_hardware:=true \
     dry_run:=false \
     max_send_rate:=30.0
   ```

9. In physical-channel mode, press `ARM` only while the robot is suspended and
   the power disconnect is accessible.
10. Use `SERVO channel degrees` controls to identify each PCA channel.
11. Assign each physical channel to one logical joint and save the mapping.
12. In ROS-joint calibration mode, set each `neutral_deg` to the servo command
    that places that real joint at URDF zero radians.
13. Test `+0.05 rad`; compare real link motion with Gazebo, not servo angle.
14. If the real link motion is opposite Gazebo, change that joint's
    `direction` to `-1` or `+1` as appropriate.
15. Test `-0.05 rad` and return to zero.
16. Level the physical feet with per-joint `trim_deg`. Do not put these
    corrections in `WALK_POSE`, `NOMINAL_FEET`, or emote joint positions.
17. Set conservative `min_deg` and `max_deg`.
18. Restart the serial bridge and verify the dry-run conversion table.
19. Enable hardware and test one joint at a time.
20. Test a complete static crouched/standing pose while raised.
21. Only after stable suspended operation should gait control be enabled.

`servo_calibration.launch.py` is a special, manual, suspended single-servo
workflow and intentionally sets `require_motion_safe_to_arm:=false`; it does
not launch the walking motion controller when Gazebo is disabled. This
exception must never be used to bypass the normal stable-pose ARM gate during a
gait test.

## Arduino manual serial test

Compile and upload:

```bash
arduino-cli compile --fqbn arduino:avr:nano firmware/volt_arduino_pca9685
arduino-cli upload -p /dev/ttyUSB0 \
  --fqbn arduino:avr:nano \
  firmware/volt_arduino_pca9685
```

For an old Nano bootloader, use the board option:

```text
arduino:avr:nano:cpu=atmega328old
```

With the robot suspended and outputs physically safe, the basic protocol test
is:

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

Front-foot channel check:

```text
ARM
SERVO 2 0
SERVO 2 20
SERVO 2 0
SERVO 5 180
SERVO 5 160
SERVO 5 180
HOLD
```

Expected with the current mapping:

- `SERVO 2 ...` moves only `front_left_foot`.
- `SERVO 5 ...` moves only `front_right_foot`.

If either front foot does not move in this direct channel test, the fault is not
Gazebo or ROS joint direction. Identify the real PCA channel using small,
conservative `SERVO channel degrees` changes, then update only `pca_channel` in
the calibration YAML.

The direct serial console bypasses the ROS bridge's stable-pose gate. It is
therefore for deliberate suspended calibration only.

## Command and output safety

The following states are separate:

| Layer | Safe/start state | Motion-enable action | Stop actions |
| --- | --- | --- | --- |
| ROS router | `HOLD` | Publish owner `MOTION` | `HOLD` or `DISABLED` |
| Serial bridge | hardware off, dry-run on, auto-arm off | Explicitly enable hardware and disable dry-run | Stop forwarding or stop the launch |
| Arduino firmware | disarmed, PCA pulses disabled | Acknowledged `ARM`, followed by a valid frame | `HOLD`, `DISARM`, `DISABLE`, or command timeout |

For normal physical gait motion, all of these must be true:

- Router owner is `MOTION`.
- The router has a finite 12-joint pose and a fresh motion source.
- The calibration is valid.
- `hardware_enabled:=true` and `dry_run:=false`.
- The bridge identified the VOLT firmware through its startup banner or
  `OK PONG`.
- The motion controller recently reported a connected, stopped `standing` or
  `sitting` pose.
- Arduino `ARM` was explicitly requested and acknowledged.

`ARM` does not automatically select ROS `MOTION`, and selecting ROS `MOTION`
does not arm the Arduino.

The GUI's ROS `DISABLE OUTPUT COMMANDS` stops router publications but does not
remove existing physical pulses. Use Arduino `DISABLE SERVO OUTPUTS` when
physical pulses must be switched off.

Firmware behavior:

- Startup is disarmed with PCA9685 outputs disabled.
- `HOLD` and `DISARM` reject new `FRAME`/`SERVO` commands while maintaining the
  last enabled output and holding torque.
- `DISABLE` disarms and switches off PCA9685 pulses.
- A 750 ms command timeout holds the last position and disarms.
- Servo targets are constrained by channel limits and physical slew limiting.
- Bad counts, bad channels, non-finite values, partial numbers, long lines, and
  unknown commands are rejected.
- `ACK_FRAME_COMMANDS` remains false to avoid Nano receive-buffer backlog.

Host behavior:

- The router rejects wrong-length, non-numeric, NaN, and infinity commands.
- Stale active-source ownership returns the router to `HOLD`.
- The bridge requires exactly 12 canonical values and a valid calibration.
- The bridge caps formatting/transmission at 30 Hz by default.
- Dry-run or hardware-disabled mode never opens serial.
- Live frames remain blocked until Arduino readiness and ARM acknowledgement.
- A lost/unsafe pending ARM is replaced by `HOLD`, never assumed successful.

## Status inspection

```bash
ros2 topic echo /volt/command_router_status --once
ros2 topic echo /volt/serial_status --once
ros2 topic info /joint_command_router/output -v
ros2 topic info /joint_group_position_controller/commands -v
```

Important serial fields include:

- `connected`, `ready`, `armed`, `streaming`, and `output_enabled`
- `dry_run`, `hardware_enabled`, and `calibration_valid`
- `motion_safe`, command `age`, and frame counters
- `pending`, `error`, and last `response`
- `clamped` joint names and the latest PCA-order `frame`

`connected=1 ready=0` means the port is open but the bridge has not identified
the expected VOLT firmware. `pending=ARM` means output is still blocked while
the bridge waits for an acknowledgement. An increasing `blocked` count means
canonical frames reached the bridge but firmware was unready, disarmed,
inhibited, or awaiting an acknowledgement.

## Troubleshooting

### Wrong servo or channel moves

Use suspended physical-channel mode and correct only `pca_channel`. Confirm the
12 configured channels are unique and exactly `0..11`. Do not compensate by
reordering `JOINT_NAMES` or the controller array.

### Correct servo moves in the wrong direction

Issue a small `+0.05 rad` canonical test and compare the real link direction
with Gazebo, not the shaft's visual rotation. Change only that joint's
calibration `direction`, then repeat `+0.05`, `-0.05`, and zero. Search for and
remove any extra sign change elsewhere. Right-side mechanical inversion belongs
in calibration exactly once.

### Servo output is clamped

Read `clamped=` in `/volt/serial_status` and the dry-run conversion row. Verify
the ROS radians are plausible, then check `neutral_deg`, `trim_deg`,
`direction`, `min_deg`, and `max_deg`. A trim or incorrect direction can push a
valid canonical pose outside the physical range. Do not widen a mechanical
limit until suspended channel tests prove the extra range is safe.

### Gazebo moves but hardware does not

Check `start_serial_bridge`, `hardware_enabled`/`use_hardware`, `dry_run`,
serial port permissions, calibration validity, Arduino readiness, explicit
ARM, and `/volt/serial_status`. `connected=0` is expected in dry-run.

### Hardware moves but Gazebo does not

Confirm the active Gazebo Sim position controller and router:

```bash
ros2 control list_controllers
ros2 topic info /joint_group_position_controller/commands -v
ros2 topic echo /volt/command_router_status --once
```

There must be one router publisher and an active
`joint_group_position_controller`.

### Router stays in `HOLD`

Startup `HOLD` is intentional. Confirm valid joint feedback, a connected motion
controller, and exactly one router, then explicitly publish/press `MOTION`.
Active ownership goes stale when its source stops sending valid commands.
Router `HOLD` is distinct from Arduino `HOLD`; inspect both status topics.

### ARM remains unavailable

For a normal gait launch, finish `STAND` or `SIT`, press `STOP`, and request
fresh status. The bridge requires recent motion status, controller connection,
no active step, valid calibration, VOLT firmware readiness, hardware enabled,
and dry-run off. Do not use the calibration launch's suspended-test exception
for ordinary walking.

### Commands return to center or output changes after a pause

Upload the current firmware. It holds the last target and disarms on timeout; it
does not command a center pose. If behavior differs, verify the actual sketch,
firmware startup banner, power integrity, and PCA wiring.

### Serial buffer backlog or malformed packets

Keep `ACK_FRAME_COMMANDS` false and the bridge at or below 30 Hz. The compact
whole-degree frame is designed to fit the Nano's receive ring. Firmware rejects
bad counts, bad channels, `nan`, `inf`, partial values, and overlong lines.

### `/joint_states` appears reordered

ROS publishers may emit any message order. VOLT calibration and control rebuild
values by joint name, then apply the canonical order. The problem is a missing,
duplicate, or incorrectly named joint, not merely a different display order.
