# VOLT Physical Fast Trot

This guide describes the current open-loop physical fast-trot implementation.
It does not change servo centres, trims, directions, PCA9685 channels, pulse
limits, or the firmware `FRAME` mapping.

> **Validation status:** the current profile has software tests, simulation
> checks, and hardware-disabled command-path diagnostics. It has **not** been
> validated as a load-bearing gait on the physical robot. A correct unloaded
> trajectory is not proof that the servos, power system, mechanics, traction,
> or support geometry can reproduce it under load.

Before any live test, read [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md). The first
physical test must be performed with VOLT secured on a rigid support stand,
every foot clear of the floor, a second person or equivalent restraint
available where practical, and the servo-power disconnect within immediate
reach.

## Why a robot can trot in the air and shake on the floor

The reported symptom is consistent with several coupled effects:

- A purely timed phase clock can ask a loaded foot to lift or reverse before
  the servo has reached the preceding target.
- Moving a foot forward before it is unloaded drags against the floor instead
  of creating a useful swing.
- Too little stance overlap, an unsuitable body height or footprint, toe
  dragging, slip, backlash, or a loose horn can turn commanded stride into body
  oscillation.
- Servo speed falls under load. Hobby-servo position commands do not provide
  motor torque or internal PID control from ROS.
- Voltage sag, converter current limiting, wiring loss, or a reset can slow
  several servos at once even when the software trajectory is valid.

The current implementation addresses the command-side failure modes with a
dedicated physical profile, conservative presets, a planted ready phase,
segmented lift-transfer-lower swing motion, joint rate guards, phase retiming,
and a stop that retains the loaded footprint. Those controls cannot prove foot
contact, delivered torque, current capacity, or actual joint tracking.

## Command path and profile selection

```text
/cmd_vel
    -> physical fast-trot Cartesian feet
    -> VOLT inverse kinematics
    -> 12 canonical ROS joint radians
    -> /volt/joint_commands/motion
    -> joint-command router
       |-> /joint_group_position_controller/commands
       `-> /joint_command_router/output
           -> calibrated physical degrees
           -> FRAME d0 ... d11 at no more than 30 Hz
           -> Arduino Nano -> PCA9685 -> hobby servos
```

The diagonal pairs are fixed from the project's canonical leg order:

```text
Pair A: front_left  + rear_right
Pair B: front_right + rear_left
```

Pair B is half a cycle behind Pair A. Direction correction is applied once,
when canonical radians are converted through
`config/servo_calibration.yaml`; the gait does not add another left/right
inversion.

The profiles are deliberately separate:

- Simulation loads the `fast_trot` section in
  `config/gait_controller.yaml`.
- `hardware_control.launch.py` sets `hardware_mode:=true` and passes
  `config/physical_fast_trot.yaml` as
  `physical_fast_trot_config_file`.
- Hardware mode requires `use_sim_time:=false`; the controller refuses to
  start if a paused simulator clock could defeat physical deadlines.
- The loader replaces only the hardware stack's `fast_trot` configuration.
  Normal trot, walking modes, servo calibration, and the simulation profile
  remain in their existing files.

The controller status fields `fast_trot_profile`,
`fast_trot_config_file`, and `hardware_mode` should show that selection. Do
not continue a hardware test if the physical file is not reported.

## Current physical profile

The authoritative physical values are under
`volt_motion_controller.ros__parameters.fast_trot` in
`config/physical_fast_trot.yaml`.

| Setting | Current value | Meaning |
|---|---:|---|
| Base hardware cycle | 0.68 s / 1.470588 Hz | YAML reference used by the WIDE preset; BENCH is the runtime default |
| Cartesian stride | 0.055 m | Raw touchdown-to-liftoff sweep before runtime and physical scaling |
| Raw step height | 0.036 m | WIDE value; BENCH starts at 0.028 m |
| Duty / swing fraction | 0.62 / 0.38 | Overlapping diagonal support timing |
| Body height | 0.188 m | 0.200 m neutral plus a -0.012 m physical offset |
| Stance half-width | 0.104 m | Body centreline to each nominal foot |
| Front / rear offsets | 0.000 / 0.000 m | Deltas from the nominal front/rear foot positions |
| Liftoff / touchdown blend | 0.20 / 0.20 | Fractions reserved for vertical unload and reload |
| Trot-ready hold | 1.0 s | Four planted feet while entering the ready posture |
| Startup / shutdown ramp | 1.50 / 1.00 s | Gradual motion entry and controlled stop |
| Physical motion scale | 0.75 | Hardware-only Cartesian stride scale |
| Contact settle dwell | 0.035 s | Commanded endpoint dwell; not measured contact feedback |
| Maximum Cartesian stride | 0.075 m | Loader-enforced absolute ceiling |
| Stance ground tolerance | 0.002 m | Commanded-FK diagnostic threshold |

### Safe runtime presets

Hardware starts with BENCH. Preset buttons only load the four GUI controls;
they do not change the controller until `APPLY FAST TROT TUNING` is pressed
while all gait motion is stopped and settled.

| Preset | Stride scale | Speed scale | Nominal period | Step height | Full-command scaled stride |
|---|---:|---:|---:|---:|---:|
| BENCH | 0.50 | 0.25 | 0.80 s | 0.028 m | 20.625 mm |
| FLOOR TEST | 0.65 | 0.35 | 0.72 s | 0.032 m | 26.8125 mm |
| WIDE | 0.80 | 0.45 | 0.68 s | 0.036 m | 33.000 mm |

The last column is:

```text
55 mm * stride_scale * physical_motion_scale
```

It is a requested Cartesian sweep, not a measured floor stride. Startup,
velocity command fraction, workspace projection, joint limiting, phase
retiming, compliance, and slip can all reduce actual motion. The configured
period is also not a promise of wall-clock cadence: the phase governor may
lengthen the cycle when downstream commands cannot reach a contact boundary.

WIDE is never selected automatically. BENCH must be checked on the support
stand before FLOOR TEST, and FLOOR TEST must be checked before WIDE.

## Load-aware trajectory and stop behavior

The physical trajectory differs from the simulation trajectory only when the
controller is in hardware mode:

1. **Trot ready:** all four world-frame feet remain planted while the body
   eases toward the physical ready posture for 1.0 second.
2. **Liftoff:** the active diagonal pair rises vertically with no horizontal
   transfer.
3. **Transfer:** after unloading, the feet move toward their frozen touchdown
   targets while remaining at swing height.
4. **Touchdown:** horizontal transfer finishes before the feet lower with a
   zero-slope blend.
5. **Stance:** planted feet remain fixed in the gait world frame; body advance
   makes them move rearward in body coordinates and creates the commanded
   propulsion.

The 0.62 duty factor gives overlap between the diagonal support intervals. The
controller gates a contact transition until the filtered command is within
approximately 1 degree of the endpoint, includes the configured 0.035-second
endpoint dwell, and slows the common phase clock when command error grows. It
does not infer contact or servo position.

On STOP or a zero/expired velocity command, the controller does not start the
next diagonal swing. It completes a pair that is already airborne, lowers it,
ramps commanded motion down, and leaves the final four-foot footprint in
place. It does not drag all four loaded feet back to nominal HOME. Status
reports `LOADED_HOLD` after that physical fast-trot stop. A later deliberate
pose or gait transition remains a separate operator action.

## Host, serial, and firmware limits

The current physical fast-trot guards are:

| Guard | Value |
|---|---:|
| Motion-controller loop | 100 Hz |
| Serial `FRAME` ceiling | 30 Hz |
| Overall physical fast-trot joint velocity | 90 deg/s |
| Shoulder velocity | 55 deg/s |
| Upper-leg velocity | 75 deg/s |
| Knee/foot velocity | 90 deg/s |
| Joint acceleration | 18 rad/s² |
| Maximum canonical command delta | 0.90 deg per controller update |
| Joint command smoothing | 0.22 |
| Velocity-command smoothing | 0.20 |
| Non-fast physical gait host ceiling | 30 deg/s |

The 100 Hz controller output is intentionally not sent to the Nano at 100 Hz.
The bridge emits compact whole-frame commands on an ideal 30 Hz schedule and
clamps `max_send_rate` to at most 30 Hz. Monitor the measured frame rate in
`/volt/serial_status` or the diagnostic CSV; a configured ceiling is not proof
that the serial link maintained it.

Normal hardware motion requires firmware protocol version 2. The bridge must
receive capability fields equivalent to:

```text
FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0
```

The repository firmware's 120 deg/s slew ceiling is a final guard and has not
been physically validated. Do not raise it. The physical profile remains below
it, and the per-joint host limits above are authoritative. Reflash the current
firmware before live fast trot if status reports `firmware_compatible=0`, an
older protocol, or the old 30 deg/s firmware image. A generic serial `PONG`
without the protocol-2 capability report cannot unlock normal ARM or live
frames.

## Parameter ownership and tuning

The physical YAML exposes descriptive names as well as compatibility fields.
Aliased values are checked for equality at startup; edit both sides of an alias
as one conceptual change or the profile will be rejected.

| Requested parameter | Current physical value | How it is changed |
|---|---:|---|
| `gait_frequency` | 1.470588 Hz | YAML-only; mirrors the 0.68 s base hardware period |
| `stride_length` | 0.055 m | YAML-only; must match `step_length_x` |
| `step_height` | 0.036 m base; 0.028 m BENCH | One of four stopped-state live controls; preset values are in YAML |
| `duty_factor` | 0.62 | YAML-only; must match `stance_ratio`, with `swing_ratio=0.38` |
| `body_height` | 0.188 m | YAML-only; must equal `0.200 + body_height_offset` |
| `stance_width` | 0.104 m | YAML-only |
| `front_foot_offset` | 0.000 m | YAML-only |
| `rear_foot_offset` | 0.000 m | YAML-only |
| `touchdown_blend` | 0.20 | YAML-only |
| `liftoff_blend` | 0.20 | YAML-only |
| `joint_velocity_limit` | 90 deg/s | YAML-only; must match `hardware_joint_velocity_limit_deg_s` |
| `joint_acceleration_limit` | 18 rad/s² | YAML-only |
| `command_smoothing` | 0.22 | YAML-only; must match `joint_smoothing_alpha` |
| `physical_motion_scale` | 0.75 | YAML-only; must match `hardware_stride_scale` |

The live tuning request contains exactly four fields:

| Live field | Accepted range | BENCH |
|---|---:|---:|
| `stride_scale` | 0.50 to 1.25 | 0.50 |
| `step_height` | 0.020 to 0.050 m | 0.028 m |
| `hardware_cycle_period` | 0.50 to 0.90 s | 0.80 s |
| `hardware_speed_scale` | 0.20 to 0.75 | 0.25 |

All four values must be present, finite, in range, below the 75 mm Cartesian
ceiling, and mutually feasible. To change one variable at a time, resend the
complete tuple while keeping the other three unchanged. A shorter period may
be rejected if the unchanged speed scale cannot produce the requested stride.
That rejection is a safety result, not a reason to bypass validation.

Safe GUI application:

1. Release forward and yaw controls.
2. Press `STOP`.
3. Wait for no active gait, no swing pair, and all feet settled.
4. Load BENCH, FLOOR TEST, or WIDE.
5. Inspect the four controls.
6. Press `APPLY FAST TROT TUNING` once.
7. Confirm the controller-reported tuple and check for a rejection warning.
8. Select FAST TROT and reapply only a small command.

The equivalent BENCH topic request, still accepted only while stopped, is:

```bash
ros2 topic pub --once /volt/fast_trot_tuning std_msgs/msg/String \
  '{data: "{\"stride_scale\":0.50,\"step_height\":0.028,\"hardware_cycle_period\":0.80,\"hardware_speed_scale\":0.25}"}'
```

YAML-only changes require a controller restart. Make one conceptual change,
keep every required alias consistent, load BENCH again, and repeat dry-run and
support-stand gates. Do not increase joint limits, acceleration, calibration
ranges, or firmware slew simply to suppress a tracking or stride warning.

## Build and launch

Build and source:

```bash
source /opt/ros/humble/setup.bash
cd /home/ros2/Documents/volt_ws
colcon build --packages-select volt_description --symlink-install
source install/setup.bash
```

### Hardware-disabled dry-run

This stack uses the physical YAML but does not open the serial port:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false
```

Confirm status says hardware disabled, dry-run, disconnected, disarmed, BENCH,
and the physical fast-trot config path. Exercise STOP, tuning rejection, pair
order, and the full command path before enabling serial hardware.

### Simulation regression

Use the combined Gazebo stack to verify that the separate simulation profile
still launches without a serial bridge:

```bash
ros2 launch volt_description volt_start.launch.py \
  gui:=true \
  gazebo_gui:=true \
  start_serial_bridge:=false \
  use_hardware:=false \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  enable_physical_tests:=false \
  use_sim_time:=true
```

This run loads `config/gait_controller.yaml`, not
`config/physical_fast_trot.yaml`. Simulation success does not validate servo
speed, power, traction, contact, or load-bearing behavior.

### Live hardware stack

> Place and secure VOLT on the support stand **before** running this command.

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  serial_port:=/dev/ttyUSB0 \
  hardware_enabled:=true \
  dry_run:=false \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false
```

Use `/dev/ttyACM0` instead if that is the verified device. The launch does not
ARM automatically. Confirm protocol 2, firmware compatibility, calibration,
router ownership, stopped standing status, and BENCH before deliberately using
the GUI's safe ARM workflow. Exact finite test commands and the required typed
support-stand acknowledgement are in [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md).

## Passive diagnostic recording

The recorder publishes no motion, owner, ARM, or serial command. It writes rows
only while `fast_trot` is active.

Start it in another sourced terminal:

```bash
ros2 run volt_description volt_fast_trot_diagnostic.py --ros-args \
  -p output_path:=/tmp/volt_fast_trot_runs \
  -p hardware_enabled:=false
```

Arm and close a recording:

```bash
ros2 topic pub --once /volt/fast_trot_diagnostic \
  std_msgs/msg/String '{data: start}'

ros2 topic pub --once /volt/fast_trot_diagnostic \
  std_msgs/msg/String '{data: stop}'
```

The recorder prints the unique CSV path. Plot it with:

```bash
ros2 run volt_description volt_plot_fast_trot.py \
  /tmp/volt_fast_trot_runs/volt_fast_trot_TIMESTAMP.csv \
  --output /tmp/volt_fast_trot.pdf
```

The CSV includes global and per-leg phase, swing/stance sets, desired feet,
canonical joints, mapped servo degrees and deltas, loop and publish timing,
configured and observed cycle period, requested and commanded-FK stride and
clearance, projection and clamp counters, serial frame rate, duplicate
publisher evidence, and warnings. Open-loop joint tracking and commanded
ground height are proxies, not encoder or contact measurements.

Before trusting a recording, also check:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic hz /cmd_vel
ros2 topic echo /volt/serial_status
ros2 topic echo /volt/status
```

An extra velocity source can interleave commands even though the downstream
router still has one actuator owner. Stop unintended GUI, teleoperation, emote,
or CLI publishers.

The existing files under `log/fast_trot_*.csv` and `log/fast_trot_*.pdf` are
software/simulation or hardware-disabled artifacts from earlier profiles.
They are useful for tooling examples but are not current load-bearing evidence
and must not be presented as physical validation.

## Acceptance and stop criteria

During every run, stop and settle on any:

- wrong diagonal pair or front/rear propulsion direction;
- toe motion forward or sideways before adequate lift;
- body/link collision, mechanical-stop approach, horn slip, or unusual noise;
- IK projection, branch discontinuity, joint/calibration clamp, or low
  workspace margin;
- persistent command delta, velocity, braking, or acceleration clamps;
- commanded stance-ground error above 2 mm or grounded signed-stride warning;
- observed cycle much longer or more erratic than expected;
- serial rate loss, rejected/blocked frames, reset, disconnect, or duplicate
  command publisher;
- voltage sag, converter limiting, synchronized servo slowdown, buzzing,
  overheating, smoke, smell, or unstable support.

Use STOP/HOLD and the physical-test emergency command when communication is
healthy. Use the accessible servo-power disconnect immediately if continued
torque or electronics behavior is unsafe. `DISABLE` removes PWM and can allow
the robot to collapse; the support stand must carry it.

## Rollback

For a runtime tuning rollback, STOP, wait for `LOADED_HOLD`/settled status,
load BENCH, apply it once, and verify the returned tuple. If behavior is still
uncertain, HOLD, DISARM, disconnect servo power, and return to dry-run.

For a YAML rollback, restore the complete known-good
`config/physical_fast_trot.yaml` from the user's backup or version-control
revision, rebuild/relaunch, and repeat validation from BENCH. Do not roll back
servo calibration or firmware as part of a gait-only experiment unless those
files were independently changed and verified. Simulation rollback is
separate because its profile remains in `config/gait_controller.yaml`.

## Remaining risks

- There is no joint encoder, torque, contact, battery-voltage, rail-voltage, or
  current sensor in this control path.
- The phase governor knows command error, not actual servo position under
  load.
- A commanded-FK planted foot may slip, deflect, or remain airborne.
- Open-loop diagonal support does not measure the centre of mass.
- Battery state, converter capacity, wiring drop, floor friction, foot
  material, mechanical backlash, horn security, and servo heating remain
  physical variables.
- Firmware acknowledgement and 30 Hz frames prove communication, not delivered
  servo torque or motion.
- HOLD/DISARM reject new motion but retain the last PWM target; DISABLE or a
  hard power cut changes the collapse risk.

Follow the one-variable test ladder and power measurements in
[PHYSICAL_TESTS.md](PHYSICAL_TESTS.md). Software must not mask a power or
mechanical fault by merely slowing the gait.
