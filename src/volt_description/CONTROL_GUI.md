# VOLT Motion Control

VOLT uses one canonical 12-joint, ROS-radian command path for simulation and
physical hardware. Walking is one engine with exactly two servo-budgeted
gaits: `amble`, a four-beat lateral-sequence walk that always keeps at least
three feet planted and is the conservative first-test gait, and `trot`, a
two-beat diagonal gait. Every historical gait name is accepted only as an
alias onto one of those two.

The principal integration launches use safe defaults: the command router
starts in `HOLD`, hardware is disabled, the serial bridge is dry-run when
started, Arduino auto-arm is disabled, and the controller does not
automatically move to its ready pose. Dedicated suspended calibration/test
launches are intentionally separate and must follow their own warnings.

## Build and launch

Run these commands from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select volt_description --symlink-install
source install/setup.bash
```

### Combined simulation and control GUI

This is the normal integration launch. It starts Gazebo Sim, the robot and
controllers, one command router, one motion controller, and the PyQt GUI. It
does not start the Arduino bridge:

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
  use_sim_time:=true
```

Use `gazebo_gui:=false` for a headless Gazebo server while retaining the VOLT
GUI. Set `gui:=false` as well when neither GUI is required.

### Active Ignition/Gazebo Sim backend only

```bash
ros2 launch volt_description ignition.launch.py \
  gui:=false \
  use_sim_time:=true
```

This launch provides the simulation, robot state publisher, VOLT entity,
controller manager, joint-state broadcaster, position controller, and `/clock`.
It does not start the VOLT command router, motion controller, or control GUI.
To run those separately in another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch volt_description control.launch.py \
  gui:=true \
  auto_ready_pose:=false \
  hardware_mode:=false \
  use_sim_time:=true
```

Do not also launch `control.launch.py` when using `volt_start.launch.py`; the
combined launch already includes it.

To run Ignition, this GUI, and the physical bridge together, use one unified
runner after the raised-robot checklist:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1
```

Add the optional Ignition servo model without changing the physical
calibration or ARM gates:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1 \
  --actuator-profile td8130mg
```

Do not start `hardware_control.launch.py` alongside that runner. The GUI and
serial bridge fail closed if duplicate critical status publishers are found.
The preset opens the bridge but never auto-arms; press `ARM SYSTEM SAFELY`
after the supported-robot confirmation.

### Hardware-only, safe dry-run

The hardware-only launch intentionally uses system time and starts no Gazebo
process:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  serial_port:=/dev/ttyUSB1 \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false \
  enable_physical_tests:=true
```

This stack has no physical joint-position feedback, so it explicitly seeds an
open-loop canonical `WALK_POSE` matching the firmware's calibrated
`CHANNEL_SAFE_START_DEG`. It is an assumption, not a measurement. Router
startup remains `HOLD`, and live motion still requires deliberate `MOTION`
ownership plus Arduino `ARM`. Pure simulation leaves this seed disabled. The
unified runner enables it only when physical hardware is explicitly enabled,
so simulator `/joint_states` cannot be mistaken for physical feedback.

`gui:=true` starts the control GUI in the same hardware-only launch. Omit it or
set it false for a headless stack.

Only after completing the physical test checklist in `SERVO_CALIBRATION.md`
should a supported, suspended robot be started with live serial output:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  serial_port:=/dev/ttyUSB1 \
  hardware_enabled:=true \
  dry_run:=false \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false \
  enable_physical_tests:=true
```

Live output still requires an explicit ROS owner of `MOTION` and a separate,
explicit Arduino `ARM`.

## Active Gazebo backend

`ignition.launch.py` is the active Gazebo Sim integration, despite its legacy
filename. Preserve this path:

```text
volt.urdf.xacro sim_backend:=gz
    -> gz_ros2_control/GazeboSimSystem
    -> controller_manager
    -> joint_state_broadcaster
    -> joint_group_position_controller
```

It uses `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_sim create`, the `default` world,
the robot name `volt`, and a bridged `/clock`.

`gazebo.launch.py` is the retained legacy Gazebo Classic launcher. It is not the
default and is not used by the walking integration. Do not mix `gazebo_ros`,
`spawn_entity.py`, or `libgazebo_ros2_control.so` into the active Gazebo Sim
path.

### Optional Ignition actuator model

`ignition.launch.py` and `volt_start.launch.py` accept
`actuator_profile:=simulation|td8130mg`; the unified runner exposes the same
choice as `--actuator-profile`. The default is `simulation`, preserving the
existing proven simulator response. `td8130mg` models the physical servo
under load:

| URDF limit/property | `simulation` | `td8130mg` |
| --- | ---: | ---: |
| Shoulder velocity | 2.0 rad/s | 4.1888 rad/s (240 deg/s firmware slew ceiling) |
| Upper-leg and foot velocity | 3.0 / 4.5 rad/s | 4.1888 rad/s (240 deg/s) |
| Joint effort | 20 | 2.8 N·m (derated stall torque) |
| Damping / friction | 0.6 / 0.1 | 0.35 / 0.10 |

The active Gazebo Sim path uses a stiff `gz_ros2_control` position gain
(18.0), so with this profile the effort clamp — load — rather than an
artificially soft tracking gain is what slows a joint, and Gazebo reproduces
load-dependent tracking lag.

For example:

```bash
ros2 launch volt_description ignition.launch.py \
  gui:=true use_sim_time:=true actuator_profile:=td8130mg

ros2 launch volt_description volt_start.launch.py \
  gui:=true gazebo_gui:=true actuator_profile:=td8130mg
```

This changes only Ignition's URDF limits and joint dynamics. It does not
select a real-robot tuning profile, calibrate a servo, change PCA9685 pulse
limits, model supply sag, or provide physical feedback.

## Simulation time and hardware time

| Process | Simulation launch | Hardware-only launch |
| --- | --- | --- |
| Gazebo and robot state publisher | `use_sim_time:=true` | not started |
| VOLT motion controller | `use_sim_time:=true` | `use_sim_time:=false` |
| Arduino serial bridge | system time | system time |
| PyQt interaction and safety-age checks | monotonic/system timing | monotonic/system timing |

The combined launch forwards `use_sim_time` to Gazebo-facing and motion-control
nodes but explicitly keeps the serial bridge on system time. The GUI's Qt
timers and safety-age checks do not wait for a simulation clock.

Never use `use_sim_time:=true` for the hardware-only stack unless a valid
external `/clock` is intentionally supplied. Without `/clock`, ROS-time motion
controller timers, transitions, status publication, and command timeouts can
appear frozen.

## Canonical joint order

The following exact order is shared by kinematics, controllers, the router,
calibration, dry-run conversion, and tests:

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

There is no separate simulation order or hardware order. PCA channel ordering
is created only inside the serial calibration layer.

## Command pipeline

```text
PyQt GUI / gamepad
    -> /cmd_vel, /volt/action, /volt/gait, /volt/body_pose
    -> volt_motion_controller
    -> /volt/joint_commands/motion (12 canonical ROS radians)
    -> volt_joint_command_router
    -> /joint_command_router/output
       |-> /joint_group_position_controller/commands -> gz_ros2_control
       `-> volt_serial_bridge -> calibrated channel-order FRAME -> Arduino
```

The router is the single publisher to the position controller and serial
output topic. Gait and IK code never compute servo degrees, PCA channels,
neutral offsets, pulse widths, or right-side servo inversion.

## ROS ownership and Arduino arming

ROS command ownership and Arduino arming are independent safety layers.

| Control | Meaning |
| --- | --- |
| `ENABLE MOTION` / owner `MOTION` | Allows valid commands from `/volt/joint_commands/motion` through the router. It does not arm the Arduino. |
| `HOLD` / owner `HOLD` | Publishes zero velocity and stop, then retains the router's last valid pose while ignoring motion commands. |
| `DISABLE OUTPUT COMMANDS` / owner `DISABLED` | Stops motion and prevents the router from publishing source commands. It does not switch off PCA9685 pulses. |
| `ARM SYSTEM SAFELY` | After confirmation, sequences zero/STOP, fresh router `MOTION` ownership, new stable MOTION-owned 12-joint frames after each STOP, and one Arduino `ARM` request. |
| `HOLD SERVOS` or `DISARM ARDUINO` | Returns ROS ownership to `HOLD`, stops new physical frames, and keeps the last enabled servo pulses/holding torque. |
| `DISABLE SERVO OUTPUTS` | Returns ROS ownership to `HOLD`, disarms the Arduino, and switches off PCA9685 output pulses. |

Physical gait motion therefore requires both intentional gates:

```text
ROS owner = MOTION
Arduino = ready and ARMED
```

The router always starts in `HOLD`, and opening the GUI never selects `MOTION`
automatically. The guided ARM action may request it only after explicit
confirmation. A stale active-source command returns the router to `HOLD`.
Closing the GUI sends zero velocity, controller `stop`, router `HOLD`, and
Arduino `HOLD`.

The normal serial bridge independently watches the router status. If
`MOTION` ownership leaves or its status becomes stale, the bridge cancels ARM,
blocks physical `FRAME` output, and sends firmware `HOLD` when connected. The
GUI requires both its fresh router view and the bridge's `owner_fresh=1`,
`owner_allowed=1`, `frame_ready=1` report before sending Arduino `ARM`.
`frame_ready=1` means the complete frame was accepted under the current
`MOTION` ownership epoch, remained within the configured stability tolerance
for the settle interval, and is still fresh. The GUI also requires
`frame_seq` to advance after both of its STOP commands.

## GUI layout

The console is divided into four focused tabs so high-use controls remain
visible without navigating one long two-axis-scrolling page:

- **CONTROL** contains robot state, ownership and STOP actions, gait and motion
  controls, the joystick, gamepad state, Arduino safety actions, and body pose.
- **EMOTES + FACE** places robot emotes beside the complete Face LEDs panel.
- **TUNING** contains the Real Robot Tuning panel and profile management.
- **DIAGNOSTICS** contains raised-hardware gait tests and commanded telemetry.

Each tab scrolls vertically only when the available display height requires
it. Safety-critical motion and physical-output controls remain on the first
tab; changing tabs does not change command ownership or robot state.

## First GUI operation

The GUI defaults to `AMBLE`, a 20% speed limit, and zero
velocity. For the first simulation or raised-stand test:

1. Wait for controller status and `ACTIVE OWNER: HOLD`.
2. In simulation, confirm the position controller is connected. In the
   hardware-only launch, `position controller: not used (open-loop hardware)`
   is expected; verify the documented open-loop seed against the raised
   mechanism instead.
3. Keep `AMBLE` selected and set speed to 10–20%.
4. Press `ENABLE MOTION`.
5. Press `STAND` and wait for the state to become `STANDING`.
6. Use `STEP IN PLACE` first, or move the joystick briefly and release it.
7. Press `STOP`, then `HOLD`, at the end of the test.

`STAND`, `SIT`, and `STEP IN PLACE` are blocked until the owner is `MOTION`;
step-in-place additionally requires the controller state `STANDING`. Non-zero
joystick velocity is published only while the controller is `STANDING` and the
owner is `MOTION`. Releasing the GUI control, disabling the gamepad, or losing a
gamepad connection publishes zero velocity and controller `stop` immediately.
The controller also zeros stale velocity after 0.6 seconds. STEP IN PLACE uses
a one-second keepalive lease renewed by the GUI, so a lost GUI cannot leave
bench stepping active indefinitely; an airborne foot still completes touchdown
before settling.

Normal steering maps left/right to yaw. Crab/omnidirectional mode maps
left/right to lateral motion, with the yaw-trim control available separately.

For live hardware, support the robot, use `STAND`/`SIT` or the exact calibrated
open-loop `WALK_POSE`, press `STOP`, and wait for a recent stable status before
pressing `ARM SYSTEM SAFELY`. A `HOLD` is accepted only when the motion
controller explicitly certifies that exact stopped `WALK_POSE`; arbitrary held
poses remain locked.
Confirm the raised-robot warning once. The workflow holds all motion controls
at zero; freezes pose, gait, tuning, and non-STOP action inputs; requests
`MOTION`; and waits for new router, controller, and stable-frame bridge
reports before sending one ARM request. Timeout, focus loss, stale status,
gamepad loss/disable, or any HOLD/DISARM/DISABLE action cancels to ROS and
firmware HOLD. The GUI never opens the serial port itself and never arms merely
because it started.

## GUI controls and status

The two gait buttons publish explicit canonical names:

- `AMBLE` -> `amble` (default)
- `TROT` -> `trot`

`amble` is a four-beat lateral-sequence walk (`rear_left`, `front_left`,
`rear_right`, `front_right`, each a quarter cycle apart) with a 0.76 duty
factor, so at least three feet are always planted, plus a small lateral body
sway away from the swinging side. `trot` is a two-beat diagonal gait with a
0.58 duty factor and the required diagonal pairing:

```text
Pair A: front_left + rear_right
Pair B: front_right + rear_left, one-half cycle later
```

Every historical gait name published on `/volt/gait` resolves through the
explicit alias table onto one of these two: the old walk and crawl names
select `amble`, and every old trot variant selects `trot`. Status always
reports the canonical name.

The GUI subscribes to `/volt/status`, `/volt/command_router_status`, and
`/volt/serial_status`. It displays:

- Requested, active, and pending gait
- Controller state, motion-active state, phase index/name, and phase progress
- Swing and stance legs
- Current and target body shift, shift completion, support validity, and
  whether lift is allowed
- ROS command owner and position-controller connection
- Simulation joint tracking error (or open-loop/N/A on hardware), IK projected
  targets, clamped commands, and current warning
- Serial bridge connection, Arduino ready/armed state, and output state
- Dry-run, hardware-enabled, calibration, pending acknowledgement, and
  clamping status
- Face LED loading and host-snapshot synchronization state

The **Arduino / Physical Robot** group labels the physical connection
explicitly as disconnected, initializing, or connected and ready. Its
always-visible **ARM readiness** box lists every current start blocker, rather
than hiding only the first reason in a disabled-button tooltip. Controller
state, calibrated-pose certification, active motion, bridge `motion_safe`,
router status/owner, serial freshness, hardware/dry-run mode, calibration,
connection, firmware readiness, and pending commands are shown as separate
reasons. The disabled button repeats the blocker count and its tooltip repeats
the full list. Green **ARM READY** means only that the pre-flight gates pass;
the guided sequence still re-checks them and verifies fresh ownership and
post-STOP frames before it sends `ARM`.

The physical-output controls are **REQUEST STATUS**, **ARM SYSTEM SAFELY**,
**HOLD SERVOS**, **DISARM ARDUINO**, and **DISABLE SERVO OUTPUTS**.

## Real Robot Tuning panel

The new panel is the authoritative stopped-state editor for the conservative
hardware amble/trot path. Hardware mode starts on `REAL_DIAGNOSTIC`; simulation
starts on `SIMULATION`. Loading or resetting a hardware profile changes only
the local widgets. **Apply** is always a separate deliberate transaction, and
never starts a gait or arms hardware. `SIMULATION` remains visible for
reference, but its value widgets plus **Apply**/**Save Profile** are read-only
so the proven simulator gait cannot be edited from this panel.

The shipped profiles in `config/real_robot_profiles.yaml` are:

| Profile | Gait | Cycle | Forward / lateral stride | Clearance | Duty | Body height / X / pitch | Velocity / acceleration | Smoothing / touchdown |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `SIMULATION` | `trot` | 1.10 s | 70 / 20 mm | 24 mm | 0.58 | 200 / 0 mm / 0 deg | 190 deg/s / 6000 deg/s² | 0.00 / 0.18 |
| `REAL_DIAGNOSTIC` | `amble` | 2.40 s | 30 / 10 mm | 22 mm | 0.80 | 200 / 0 mm / 0 deg | 100 deg/s / 2400 deg/s² | 0.15 / 0.30 |
| `REAL_SAFE` | `trot` | 1.20 s | 45 / 8 mm | 24 mm | 0.62 | 195 / -4 mm / +1 deg | 150 deg/s / 3600 deg/s² | 0.12 / 0.28 |
| `REAL_NORMAL` | `trot` | 1.05 s | 55 / 12 mm | 24 mm | 0.58 | 195 / -3 mm / +1 deg | 175 deg/s / 4800 deg/s² | 0.10 / 0.24 |

All four use a 104 mm stance half-width and otherwise neutral body Y, roll,
and yaw. `REAL_NORMAL` is an available later-stage reference, not permission
to skip raised `REAL_DIAGNOSTIC` and `REAL_SAFE` testing.

Editable values and enforced bounds are cycle 0.60–6.00 s; forward stride
5–75 mm; lateral stride 0–30 mm; clearance 0–45 mm; duty factor 0.55–0.90;
body height 175–220 mm; body X ±25 mm; body Y ±20 mm; roll/pitch ±4.5 degrees;
yaw ±10 degrees; maximum joint velocity 60–190 deg/s; maximum joint
acceleration 600–6000 deg/s²; smoothing 0–0.80; touchdown softness 0.08–0.35;
and stance half-width 80–130 mm. A profile targets either `amble` or `trot`;
the GUI's profile-gait selector offers exactly those two.

`smoothing_amount` is the user-facing amount of filtering: larger means more
smoothing. The controller derives its joint-filter alpha as `1 - amount`,
bounded to 0.20–1.00; do not interpret a larger amount as faster tracking.
Gait-specific duty checks also apply: `amble` requires a duty factor of
0.70–0.86 so at least three legs stay in stance, and `trot` requires
0.52–0.68. Those checks can reject a value that is inside the general
duty-factor widget range, and applying any profile re-runs the gait engine's
servo-budget sweep, so a profile cannot demand joint speeds the TD-8130MG
cannot deliver under load.

### Correlated atomic apply

**Apply** publishes strict JSON on `/volt/real_robot_tuning`. A request has a
1–64-character ID, selected profile name, and the complete validated value
set, for example:

```json
{"request_id":"gui-tune-1234","profile_name":"REAL_SAFE","values":{"gait":"trot","cycle_duration":1.2,"stride_length":0.045,"lateral_stride_width":0.008,"step_height":0.024,"duty_factor":0.62,"body_height":0.195,"body_x":-0.004,"body_y":0.0,"body_roll_deg":0.0,"body_pitch_deg":1.0,"body_yaw_deg":0.0,"max_joint_velocity_deg_s":150.0,"max_joint_acceleration_deg_s2":3600.0,"smoothing_amount":0.12,"touchdown_softness":0.28,"stance_width":0.104}}
```

The controller rejects partial, unknown, non-finite, out-of-range, gait-ratio,
or infeasible data. It also rejects an apply unless locomotion, velocity,
step-in-place, pose transitions, diagnostics, and emotes are completely
stopped. Before committing, it samples stance and swing extrema through IK;
any projected foot or clamped joint rejects the whole request. Only after all
checks pass are body targets, gait geometry/timing, joint velocity and
acceleration limits, smoothing, and touchdown behavior committed together.
There is no partial apply.

In non-hardware mode, the controller can acknowledge only the exact shipped
`SIMULATION` trot values; the GUI presents them read-only.
Real-hardware conditioning stays disabled, so this panel cannot silently alter
the established simulator path.

The GUI waits for the same `real_tuning_request_id` in `/volt/status` and then
shows `real_tuning_result` (`applied` or `rejected`) and
`real_tuning_message`. It never treats an unrelated/stale status as its
result. Status also returns the effective `real_profile`, complete
`real_tuning` values, bounds, profiles, and per-joint limits. Editing a built-in
value makes the exact active match `CUSTOM`; a separately named saved profile
retains its name.

**Save Profile** validates the name and values and atomically writes a user
overlay, leaving the running controller unchanged until **Apply** is pressed.
Saved names use uppercase A–Z, digits, `_`, or `-`.
The overlay is
`$XDG_CONFIG_HOME/volt_description/real_robot_profiles.yaml`, or
`~/.config/volt_description/real_robot_profiles.yaml` when
`XDG_CONFIG_HOME` is unset. It overlays the installed defaults in the GUI;
shipped YAML remains unchanged. The controller starts from its explicitly
passed shipped profile file and does not silently ingest the user overlay;
overlay values become live only when the GUI sends a complete Apply request.
Preserve that file when moving a tuned setup to another account or machine,
and record which physical robot/calibration it belongs to.

## Robot Emotes panel

The ten controller-owned Cartesian emotes are Push-ups, Body roll, Nod, Wave
left, Wave right, Heart, Bow, Stretch, Happy dance, and Shake no. `SIT` and
`STAND UP` beside them use a separate planted-foot Cartesian pose transition:
rearward shift, asymmetric rear lowering with front-leg support, settle, then
the exact captured path in reverse.
Emote repetitions/speed/amplitude/depth, correlation, and emote progress do
not apply to those two pose actions. The GUI sends correlated JSON
start/keepalive/cancel requests on `/volt/emote`, and the controller queues behind a
clean locomotion stop, requires standing plus
`MOTION` ownership, preflights the composed trajectory through IK, and remains
the sole owner of Cartesian targets during playback.

**Push-up travel** directly selects 10–25 mm of vertical body travel in 1 mm
steps; its 20 mm default is larger than the original 15 mm motion. It is
encoded through the existing validated `depth` request field and affects only
`push_ups`. The controller still preflights the complete request against the
captured stance, so 25 mm is available from the catalog's 200 mm neutral stand
but can be rejected when the current stance is already below 200 mm.

`STOP EMOTE` and ordinary STOP cancel playback; an active emote returns
smoothly to its captured commanded stand in about one second. STOP does not by
itself change router ownership or firmware ARM state. The GUI renews the
emote's 750 ms lease every 200 ms while controller status remains fresh. GUI
or client loss cancels a queued request or starts the conditioned return from
an active request; owner loss, STOP, and nonzero velocity also cancel. Owner loss is an immediate
HOLD/reset and may freeze the last command rather than complete the smooth
return. See [EMOTES.md](EMOTES.md) for the catalog, exact JSON,
balance-sensitive actions, and compatibility-client distinction.

## Face LEDs panel

The Face LEDs panel controls the two physical WS2812B strips as one mirrored
eight-pixel face. **Enabled** reapplies the selected snapshot; disabling it
sends the `off` effect without changing the saved expression. **Automatic
expression during emotes** follows controller status, while **Lock current
expression** keeps the selected manual face during emotes and walking. A lock
never suppresses an emergency, critical-fault, or reported low-voltage safety
expression.

Choose a preset from the dropdown or one of the sixteen preset buttons. A
preset loads its configured color, effect, brightness, and animation speed.
The QColor picker, R/G/B integer controls, and live preview stay synchronized;
customizing them does not publish until **APPLY** is pressed. **OFF** sends
`effect=off`, **RESTORE DEFAULT** reloads the shipped idle settings, and **TEST
LEDS** uses a Qt timer to request solid red, green, blue, and white in turn,
then restores the prior face. No LED action sleeps or blocks the GUI/ROS spin.
Alternating presets take their secondary RGB value from
`config/face_expressions.yaml`; presets without one mirror the currently shown
primary color. A separate secondary-color picker is intentionally unnecessary.

The last enabled, automatic, lock, expression, RGB, brightness, effect, and
speed values are atomically saved as JSON at
`$XDG_CONFIG_HOME/volt/face_led_settings.json`, or
`~/.config/volt/face_led_settings.json` when `XDG_CONFIG_HOME` is unset.
Invalid or corrupt saved data is ignored and the shipped defaults are used.

Automatic selection and restoration are driven by `/volt/status`:

- sit/sitting down uses `neutral`;
- stand/standing up uses `success`, then restores the saved manual expression
  (normally `idle`) once standing;
- locomotion uses `idle` unless locked;
- calibration ownership uses `thinking`;
- each active Cartesian emote uses its mapping in
  `config/face_expressions.yaml`; and
- completion/cancel restores the exact saved manual settings, not the emote
  preset values.

The Arduino/bridge status line reports support, current expression/effect/RGB,
brightness, speed, and whether the desired settings are synchronized. A safety
override can only react to telemetry that exists in `/volt/status` or
`/volt/serial_status`. This project does not currently measure battery voltage,
so low-voltage `alert` is available and tested but will not trigger until a
voltage monitor publishes `low_voltage` or `undervoltage` status.

New firmware starts the face in a cyan loading indication. The Control tab's
**Face LEDs / host sync** row reports the sequence as waiting for host PING,
applying or verifying the GUI snapshot, finalizing, or **HOST SYNCED**. The
firmware leaves loading only after it has seen a desired mutation, the bridge
has confirmed the resulting LED status, and `HOST SYNC` is acknowledged.
`face_loading`, `host_sync_state`, `host_ping`, `host_snapshot`,
`host_sync_pending`, `host_synced`, and `host_sync_error` in
`/volt/serial_status` support this display. They are visual-handshake status
only: none is an Arduino ARM predicate, and a stalled LED sync neither enables
ARM nor removes any existing motion, ownership, calibration, connection, or
firmware-readiness gate.

## Hardware Gait Diagnostic panel

This panel is visible only when the launch explicitly sets
`enable_physical_tests:=true`:

- **A — STAND** uses the existing normal stand transition; it is not leased.
- **B — SLOW SQUAT**, **C — SINGLE LEG LIFT**, and **D — STEP ONE LEG** are
  finite Cartesian requests on `/volt/physical_test`.
- **E — SELECT AMBLE** and **F — SELECT TROT** stop and select
  `amble` or `trot`; they do not start motion. Wait for
  status, then use only minimal joystick input.

B–D require hardware mode, `MOTION`, stopped standing, neutral body/feet, and
no competing gait/transition/emote. The GUI renews the exact request every
200 ms; the controller's default 750 ms lease timeout triggers a smooth
one-second commanded return. `STOP DIAGNOSTIC` cancels the matching lease and
sends STOP, but leaves ownership/firmware ARM unchanged. D requires at least
6 s even though the shared GUI selector begins at 5 s. See
[PHYSICAL_TESTS.md](PHYSICAL_TESTS.md) for exact payloads, `/dev/ttyUSB1`
launches, CLI modes, and the required support-stand sequence.

## Commanded Telemetry — No Servo Feedback

This GUI group intentionally says **Commanded**. It shows the controller's
commanded body XYZ/RPY; each leg's swing/stance state, phase, and commanded
foot XYZ; raw and filtered commanded joint targets; maximum raw-to-filtered
command difference; applied profile/emote state; ROS route; and Arduino
command-path state. `/volt/status` supplies `commanded_foot_xyz`,
`commanded_body_target`, `raw_joint_target`, `filtered_joint_target`,
`raw_to_filtered_joint_error`, `joint_names`, and effective limits.

None of those values is actual TD-8130MG shaft position, foot contact, load,
slip, torque, current, or voltage. The Arduino/PCA9685 hobby-servo route has no
servo feedback. “Grounded”/stance labels and commanded-FK quantities such as
ground height are command/kinematic proxies, not measurements. Gazebo
`/joint_states` can provide simulation tracking only and must never be
presented as physical feedback when the simulator is used as a shadow view.

## Walking behavior

Both gaits come from one engine, `scripts/volt_gait_controller.py`, sharing
world-locked stance, smoothstep swing transfer with a sin² vertical lift, and
frozen world-frame touchdown targets. During stance a planted foot holds a
fixed world point while the integrated body pose moves over it, so commanded
stance feet cannot skate by construction; command changes never jerk an
airborne foot.

Every gait configuration is validated when it is loaded — and again whenever
a real-robot profile is applied — by sweeping one full cycle at the gait's
maximum command through the real IK. A configuration is refused if commanded
joint speeds exceed 80 deg/s on loaded stance joints or 190 deg/s on the
unloaded swing leg (the firmware slew ceiling is 240 deg/s), or if commanded
accelerations exceed 6500 deg/s². The budgets differ because the swing leg
is unloaded (near the servo's ~375 deg/s free speed) while stance joints
carry the body but rotate slowly. A configuration that loads is one the
servos can execute; no downstream limiter reshapes the trajectory.

The authoritative settings are the `gaits` section of
`config/gait_controller.yaml`:

| Key | `trot` | `amble` |
| --- | ---: | ---: |
| `cycle_period` | 1.1 s | 2.0 s |
| `duty_factor` | 0.58 | 0.76 |
| `step_height` | 0.024 m | 0.020 m |
| `max_x` / `max_y` | 0.12 / 0.05 m/s | 0.05 / 0.03 m/s |
| `max_yaw` | 0.50 rad/s | 0.30 rad/s |
| `settle_time` | 0.6 s | 0.8 s |
| `body_sway_y` | 0.0 m | 0.012 m |
| `body_height_offset` | 0.0 m | 0.0 m |
| `velocity_filter_alpha` | 0.30 | 0.25 |
| `command_acceleration` | 0.25 | 0.15 |
| `hardware_speed_scale` | 0.80 | 0.85 |
| `joint_velocity_limit_deg_s` | 190 | 190 |
| `joint_acceleration_limit_deg_s2` | 6500 | 6500 |
| `stance_velocity_budget_deg_s` | 80 | 80 |
| `swing_velocity_budget_deg_s` | 190 | 190 |

The motion controller runs at 100 Hz and publishes effective gait limits in
`/volt/status`; the GUI consumes those limits instead of maintaining
independent speed tuning. The stopped-state real profiles instead apply
their complete conditioning limits atomically: 100 deg/s for
`REAL_DIAGNOSTIC`, 150 deg/s for `REAL_SAFE`, and 175 deg/s for
`REAL_NORMAL`, all below the firmware's 240 deg/s fault ceiling and subject
to the same servo-budget sweep. None of these limits is physical tracking
feedback or a validated floor speed; follow
[PHYSICAL_TESTS.md](PHYSICAL_TESTS.md).

## Stop and safe gait switching

Normal `STOP` does not reset a phase while a foot is airborne. The active
gait finishes the current swing, touches the foot down, and spends the
configured settle time returning all four feet to nominal stance. A safety
stop is latched: after a stop request or command timeout the engine stays
stopped even if the joystick is still held, and the motion controller
releases the forced-stop latch only after observing a truly neutral command.

Changing owner to `HOLD` or `DISABLED` is immediate emergency behaviour: the
router retains its last valid pose and may therefore freeze an airborne foot.
For a planned stop, press `STOP`, wait for four stance legs in status, and then
press `HOLD`.

When another gait is selected:

1. The requested name becomes `pending_gait`.
2. Commanded velocity ramps toward zero.
3. The active foot finishes its swing.
4. All four feet touch down and settle.
5. The pending gait becomes active.
6. Filtered velocity remains zero until a new `/cmd_vel` message arrives.

Watch the GUI's requested, active, and pending fields. Releasing and reapplying
the control after activation is the clearest way to provide the required fresh
command.

## ROS interfaces

- `/cmd_vel` (`geometry_msgs/Twist`): x/y translation and yaw velocity
- `/volt/action` (`std_msgs/String`): `stand`, `sit`, `stop`, `step`,
  `debug_on`, or `debug_off`
- `/volt/gait` (`std_msgs/String`): canonical gait name (`amble` or `trot`);
  historical gait names are compatibility aliases
- `/volt/body_pose` (`geometry_msgs/Twist`): body translation/height and
  roll/pitch/yaw
- `/volt/real_robot_tuning` (`std_msgs/String`): correlated, complete
  stopped-state profile transaction JSON
- `/volt/emote` (`std_msgs/String`): correlated Cartesian emote
  start/keepalive/cancel JSON
- `/volt/face/expression` (`std_msgs/String`): lower-case expression preset
- `/volt/face/color` (`std_msgs/ColorRGBA`): normalized RGB channels in
  `[0.0, 1.0]`; alpha is ignored
- `/volt/face/alternate_color` (`std_msgs/ColorRGBA`): normalized secondary
  RGB used by alternating/chasing presets; alpha is ignored
- `/volt/face/brightness` (`std_msgs/UInt8`): global brightness in `0..255`
- `/volt/face/effect` (`std_msgs/String`): lower-case firmware effect; `off`
  disables the face without erasing the saved settings
- `/volt/face/speed` (`std_msgs/UInt32`): firmware animation interval in
  milliseconds, clamped to `10..60000`
- `/volt/physical_test` (`std_msgs/String`): finite diagnostic
  start/keepalive/cancel JSON
- `/volt/command_owner` (`std_msgs/String`): `MOTION`, `HOLD`, or `DISABLED`
  from the main GUI
- `/volt/joint_commands/motion` (`std_msgs/Float64MultiArray`): canonical
  12-radian motion output
- `/joint_command_router/output` (`std_msgs/Float64MultiArray`): validated
  canonical router output
- `/joint_group_position_controller/commands`
  (`std_msgs/Float64MultiArray`): Gazebo position-controller input
- `/volt/status` (`std_msgs/String`): JSON controller/gait/IK status
- `/volt/command_router_status` (`std_msgs/String`): owner and downstream route
  status
- `/volt/serial_command` (`std_msgs/String`): safe Arduino protocol request
- `/volt/serial_status` (`std_msgs/String`): bridge, firmware, output, and
  clamping status

The standalone pose commands require the motion controller and router to be
running with owner `MOTION`:

```bash
ros2 run volt_description stand_up.py
ros2 run volt_description sit_pose.py
```

## Validation and troubleshooting

### Foot sliding or skating

- Both gaits world-lock planted feet, so commanded stance cannot skate by
  construction; visible sliding points at tracking lag or contact/friction.
- In simulation, confirm `use_sim_time:=true` and that `/clock` advances.
- Reduce the speed slider and yaw/lateral demand.
- Verify the expected support pattern in status: `amble` keeps at least three
  legs in stance, `trot` swings its diagonal pairs half a cycle apart.
- Confirm the position controller is active and `/joint_states` contains all
  12 joints. Tracking lag can look like foot slip even when the planned stance
  footholds are world-locked.
- Check ground contact/friction and avoid starting from a penetrated or
  unsupported model pose.
- Inspect `/volt/status` for joint tracking error, workspace projection, or
  joint-limit warnings.

### Incorrect right-side direction

The gait and IK output canonical URDF radians with mirrored geometry. Do not
negate right-side joints in the gait, IK, router, or Arduino. Compare a small
positive joint command against Gazebo; if only the physical link moves in the
opposite direction, change that joint's single `direction` entry in
`config/servo_calibration.yaml`. See `SERVO_CALIBRATION.md`.

### Joint or servo clamping

Check `projected_targets`, joint error, and warnings in `/volt/status`, then
check `clamped=` in `/volt/serial_status`. IK projection or URDF joint limiting
must be corrected at the foot/body command level. Physical-degree clamping must
be investigated using the joint's neutral, trim, direction, and conservative
minimum/maximum calibration; do not simply widen physical limits.

### Controller-manager startup

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /joint_states
ros2 topic info /joint_group_position_controller/commands -v
```

Expected active controllers are `joint_state_broadcaster` and
`joint_group_position_controller`, with position command interfaces for all 12
joints. If they are absent, confirm the VOLT entity spawned in world `default`,
the `gz_ros2_control/GazeboSimSystem` plugin loaded, the workspace was sourced,
and no second simulator or controller manager is running. Review the launch
logs from the entity creation and sequential controller spawners. Do not fall
back to the legacy Gazebo Classic launch for this integration.

### `/clock` is absent

```bash
ros2 topic echo /clock --once
ros2 param get /volt_motion_controller use_sim_time
```

For Gazebo Sim, confirm the `clock_bridge` and simulation server are running and
launch with `use_sim_time:=true`. For hardware-only operation, `/clock` is not
required: relaunch with `use_sim_time:=false`. A node configured for simulation
time without a publisher will not advance its ROS timers.

### Router remains in `HOLD`

```bash
ros2 topic echo /volt/command_router_status --once
ros2 node list
ros2 topic info /volt/joint_commands/motion -v
```

`HOLD` at startup is intentional. Confirm there is exactly one
`/volt_joint_command_router`, wait for valid joint feedback, and press
**ENABLE MOTION** only when movement is intended. A source that stops refreshing valid
commands returns to `HOLD`; inspect controller warnings and topic publishers
before re-enabling. `pose_valid=0` means the router has not yet received a full,
finite 12-name joint state. Do not confuse router `HOLD` with Arduino
`HOLD`/`ARM`.

### Arduino ARM remains locked

Check `/volt/serial_status`. Hardware must be explicitly enabled, dry-run must
be off, calibration must be valid, the bridge must identify the VOLT firmware,
the router must freshly report owner `MOTION`, and the motion controller must
recently certify the exact stopped calibrated open-loop `WALK_POSE`. Sitting
is never an ARM source; Stand while unarmed and wait for settlement. Inspect
`owner_fresh`, `owner_age`, `owner_allowed`, `frame_ready`, `frame_seq`,
`frame_stable`, and `frame_stable_age`; then press `STOP` and `REQUEST STATUS`.
Never bypass this gate for a normal gait test.

After the normal Ignition launch, `hardware_enabled=0 dry_run=1 connected=0
ready=0` is expected: the serial bridge is intentionally acting as a safe
simulation sink and no Arduino connection is requested. The Control tab will
therefore show hardware-disabled, dry-run, disconnected, and not-ready ARM
blockers. This is not a USB failure and ARM must remain disabled. Use the
documented hardware-only launch with deliberate `use_hardware:=true` and
`dry_run:=false` only for a supported physical test.

### GUI flashes between hardware and simulation status

Two VOLT stacks are publishing the same global topics. Stop both, then start
exactly one unified runner. New builds show
`DUPLICATE VOLT STACK — ARM LOCKED`, send STOP/HOLD, ignore the conflicting
status stream, and keep the serial bridge from arming until the extra
publishers disappear.

### Ungraceful GUI process exit

The GUI's normal close path and gamepad-disconnect path are fail-safe, but a
forced process kill cannot publish their stop/HOLD messages. The controller's
0.6-second velocity timeout removes the last velocity command and the
one-second STEP keepalive lease stops persistent bench stepping. The
still-running controller may nevertheless keep router ownership alive with
stationary poses. Before working near live hardware, keep the servo-power
disconnect accessible and supervise the GUI process. A separate GUI ownership
heartbeat remains a recommended future hardening item.
