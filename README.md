# VOLT quadruped controller

VOLT is a ROS 2 Humble 12-joint quadruped stack with one canonical joint-command
path for Gazebo Sim and the physical Arduino Nano/PCA9685 robot. It includes a
PyQt control GUI, closed-form VOLT kinematics, a command-ownership router,
calibrated serial output, diagonal trot modes, and a conservative
`spotmicro_video_walk` crawl that reproduces the slow, deliberate
body-shift-and-step style of the reference video. It is adapted from the
walking concepts in
[mike4192/spotMicro](https://github.com/mike4192/spotMicro).
The current hardware workflow adds stopped-state real-robot profiles,
controller-owned Cartesian emotes, two mirrored WS2812B face strips with
automatic emote expressions, leased A–F diagnostics, and explicitly
commanded-only telemetry.

> **Physical-hardware warning:** build and simulation success do not establish
> that a servo mapping, direction, trim, or mechanical limit is safe. Keep the
> robot raised, keep the servo-power disconnect accessible, begin in dry-run,
> and follow the complete physical-test checklist below. Nothing arms the
> Arduino automatically with the documented commands.

## Supported stack and active simulator

The active simulation backend is `launch/ignition.launch.py`. On the ROS 2
Humble environment used for this integration, it is Gazebo Sim 6 / Ignition
Gazebo and uses:

- `ros_gz_sim` for the simulator launch and `create` executable
- `ros_gz_bridge` for `/clock`
- `gz_ros2_control/GazeboSimSystem` for ROS 2 control
- `joint_state_broadcaster`
- `joint_group_position_controller`

The binary may still be named `ign gazebo` on this Humble/Gazebo 6 system while
the ROS packages use the newer `ros_gz_*` names. That naming mix is expected.
Do not migrate this path to `gazebo_ros`, `spawn_entity.py`, or
`libgazebo_ros2_control.so`.

`launch/gazebo.launch.py` is retained only as a legacy Gazebo Classic launcher.
It is not used by `spotmicro_video_walk` or `spot_walk`, is not included by the
principal launch, and is not the supported backend for this integration.

The active simulator arrangement is:

```text
volt.urdf.xacro sim_backend:=gz
              ↓
gz_ros2_control/GazeboSimSystem
              ↓
controller_manager
              ↓
joint_state_broadcaster
joint_group_position_controller
```

Gazebo resource paths are derived from the installed package rather than a
user-specific workspace path, so meshes work from a normally sourced install
and from `--symlink-install`.

### Optional TD-8130MG simulation profile

Ignition defaults to `actuator_profile:=simulation`, preserving the established
simulation behavior. To model the real servo's conservative speed/effort
envelope in Ignition, select the optional `td8130mg` xacro profile:

```bash
ros2 launch volt_description ignition.launch.py \
  gui:=true use_sim_time:=true actuator_profile:=td8130mg
```

It keeps shoulder velocity at 2.0 rad/s (about 114.6 deg/s), limits upper-leg
and foot joints to 2.0944 rad/s (120 deg/s), changes effort from 20 to 6, and
uses damping/friction 0.9/0.15 instead of 0.6/0.1. The unified runner accepts
`--actuator-profile td8130mg`, including with `--physical` for its Ignition
shadow model.

This option changes simulated URDF dynamics only. It does not select a
real-robot controller profile, calibrate the TD-8130MG, alter Arduino/PCA9685
PWM, tune the servo's internal controller, or create physical feedback.

## Build

The Nano face firmware requires `Adafruit NeoPixel` in addition to the existing
Adafruit PWM servo-driver library. In Arduino IDE 1.8, open **Sketch > Include
Library > Manage Libraries**, search for **Adafruit NeoPixel** by Adafruit, and
install it. The equivalent command available on this development machine is:

```bash
arduino --install-library 'Adafruit NeoPixel' --save-prefs
```

With `arduino-cli`, use `arduino-cli lib install "Adafruit NeoPixel"`. Verify
the current Nano old-bootloader target without uploading or moving servos:

```bash
arduino --verify --board arduino:avr:nano:cpu=atmega328old \
  firmware/volt_arduino_pca9685/volt_arduino_pca9685.ino
```

Then build the ROS package:

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select volt_description \
  --symlink-install

source install/setup.bash
```

Source `/opt/ros/humble/setup.bash` and this workspace's `install/setup.bash`
in every new terminal.

## Launch modes

The principal launch defaults and first examples below are non-actuating: no
automatic ready pose, no automatic Arduino arm, and no live physical output.
The later example explicitly labeled as live hardware deliberately overrides
those defaults and must be used only after the dry-run checklist.

### One-command safe runner

Start the Ignition server, VOLT control GUI, dry-run serial bridge, and
Ignition viewer together with:

```bash
ros2 run volt_description volt_run_all.py
```

The runner refuses to start when another VOLT or Ignition controller stack is
already present, preventing duplicate `/clock`, `/stats`, and controller
instances. Gazebo runs as a protected headless server with its viewer as a
separate process, so a display-driver failure cannot terminate simulation.
On this machine, automatic renderer selection detects the unavailable NVIDIA
PRIME kernel module and starts the Ignition viewer on Intel/Mesa with Ogre.
Hardware remains disabled, serial remains dry-run, auto-ready remains off, and
Arduino ARM remains off unless those command-line gates are deliberately
changed.

For one combined Ignition + control GUI + physical bridge stack, after the
raised-robot checks, use the same runner instead of starting
`hardware_control.launch.py` beside it:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1
```

To use the optional TD-8130MG model for the Ignition shadow in that same full
system:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1 \
  --actuator-profile td8130mg
```

The `--physical` preset enables the live bridge, uses the explicit device (or
detects one when the option is omitted), opens both GUIs, keeps automatic
ready-pose and automatic ARM disabled, and leaves the final action on the
guided `ARM SYSTEM SAFELY` button. Never run a second VOLT control or hardware
launch at the same time.

### 1. Ignition/Gazebo Sim only

This starts the active simulator, robot state publisher, VOLT model, clock
bridge, and the two ROS 2 controllers. It does **not** start the VOLT motion
controller, command router, GUI, or serial bridge.

```bash
ros2 launch volt_description ignition.launch.py \
  gui:=false \
  use_sim_time:=true
```

Use `gui:=true` when the Gazebo client is needed.

### 2. Combined simulation and VOLT control

This is the normal simulation launch. It includes the active simulator exactly
once and starts one command router, one motion controller, and the PyQt GUI
when requested. The serial bridge is absent unless explicitly enabled.

```bash
ros2 launch volt_description volt_start.launch.py \
  gui:=true \
  gazebo_gui:=true \
  use_sim_time:=true \
  start_serial_bridge:=false \
  use_hardware:=false \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false
```

The shorter safe form is:

```bash
ros2 launch volt_description volt_start.launch.py
```

On this machine, prefer the one-command runner above when a Gazebo window is
needed. It applies the required Intel/Mesa viewer settings and keeps a GUI
driver failure isolated from the simulation server.

To inspect simulated commands through the calibration conversion without
opening a serial device, add:

```text
start_serial_bridge:=true hardware_enabled:=false dry_run:=true
```

### 3. Hardware control without Gazebo

This starts one router, the motion controller, and the serial bridge. The
bridge stays in dry-run, does not open the Arduino, and uses wall/system time.
Set `gui:=true` to start the guided physical-robot control window in the same
launch.

Because the hobby-servo stack has no position feedback, this hardware-only
launch explicitly enables an open-loop seed at canonical `WALK_POSE`. It
exactly matches the firmware's calibrated `CHANNEL_SAFE_START_DEG`, but remains
an assumption rather than a measurement. Startup ownership is still `HOLD`, so
it cannot reach the serial bridge until the operator deliberately selects
`MOTION`; live output additionally requires an explicit Arduino `ARM`. When
hardware is enabled in the combined Ignition launch, the same open-loop
controller deliberately ignores simulator `/joint_states`; Gazebo is then a
shadow visualization, not physical feedback.

Before first PWM/ARM, support the robot and verify the physical joints/servo
shafts correspond to that calibrated standing origin. Software agreement
between `WALK_POSE` and `CHANNEL_SAFE_START_DEG` cannot prove the unpowered
mechanism is already there; an incorrect mechanical starting pose can still
jump when pulses are first enabled.

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  serial_port:=/dev/ttyUSB1 \
  baud_rate:=57600 \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false \
  enable_physical_tests:=true
```

The physical examples deliberately use `/dev/ttyUSB1`. Verify that device
exists and is the intended Arduino before ARM; use the actually verified
`/dev/ttyACM*` or persistent udev path when it differs. Never use a guessed
port. At the last local inspection, neither `/dev/ttyUSB*` nor `/dev/ttyACM*`
was present, so `/dev/ttyUSB1` is the requested launch target—not a currently
confirmed connection.

Only after completing the raised-robot dry-run checks, live hardware can be
explicitly unlocked by restarting the hardware-only launch with:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  serial_port:=/dev/ttyUSB1 \
  baud_rate:=57600 \
  hardware_enabled:=true \
  dry_run:=false \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false \
  enable_physical_tests:=true
```

This opens the configured port but keeps `auto_arm:=false`. In the GUI, press
`ARM SYSTEM SAFELY` once the robot is supported and status certifies the exact
stopped calibrated open-loop `WALK_POSE`. ARM from `sitting` is deliberately
forbidden; Stand while unarmed, settle, and then ARM.
After a confirmation dialog, the guided
workflow publishes zero/STOP, requests `MOTION`, waits for fresh router and
bridge ownership plus a new, stable, complete 12-joint frame after each STOP,
and sends one `ARM`. Pose, gait, tuning, and non-STOP action controls are
frozen during that short verification window. An acknowledgement timeout,
focus loss, stale status, gamepad loss, or any HOLD/DISARM/DISABLE action
returns both layers to `HOLD`. For an initial test, use the hardware-only
wall-time launch. A combined launch can also start the bridge with
`start_serial_bridge:=true`; in hardware mode the physical motion controller
still uses wall time while Gazebo uses simulation time. Servo conditioning and
watchdog leases therefore remain deterministic if Gazebo pauses; the model is
not treated as phase-synchronized physical feedback.

## Canonical command pipeline

```text
PyQt GUI / gamepad
    │
    ├── /cmd_vel
    ├── /volt/action
    ├── /volt/gait
    ├── /volt/body_pose
    ├── /volt/real_robot_tuning   (correlated stopped-state JSON)
    ├── /volt/emote               (correlated start/keepalive/cancel JSON)
    ├── /volt/physical_test       (leased diagnostic JSON)
    ├── /volt/command_owner
    └── /volt/serial_command
             ↓
volt_motion_controller
             ↓
/volt/joint_commands/motion       (12 canonical ROS radians)
             ↓
volt_joint_command_router
             ├── /joint_group_position_controller/commands
             │                 ↓
             │          gz_ros2_control
             │                 ↓
             │          Ignition/Gazebo Sim
             │
             └── /joint_command_router/output
                               ↓
                      volt_serial_bridge
                               ↓
                    calibration conversion
                               ↓
                    FRAME d0 ... d11
                               ↓
                         Arduino Nano
                               ↓
                           PCA9685
                               ↓
                           12 servos
```

The router is the only component that fans a validated pose out to simulation
and serial. The gait and IK layers know only ROS joint radians; they do not
contain servo degrees, channels, neutral angles, pulse widths, or right-side
electrical inversions.

Useful status topics are:

- `/volt/status`: motion state, requested/active/pending gait, phase, stance and
  swing legs, body shift, active real profile/tuning transaction, emote and
  diagnostic correlation, effective gait/joint limits, IK projection,
  commanded feet/body/raw-to-filtered joints, and warnings
- `/volt/command_router_status`: active owner, position-controller connection,
  output subscribers, and whether a valid hold pose exists
- `/volt/serial_status`: bridge/Arduino state, dry-run and hardware gates,
  calibration state, clamping, latest frame, and protocol errors

On physical hardware, these are command-path reports. The TD-8130MG/PCA9685
route returns no actual servo shaft position, foot contact, load, slip, torque,
voltage, or current. A commanded foot marked stance/grounded and any
commanded-FK “achieved” metric are not measurements.

## Canonical joint order

The following order is authoritative everywhere in ROS and must never be
reordered independently for simulation or hardware:

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

It is shared by `volt_kinematics.JOINT_NAMES`, the position controller, the
command router, calibration input, dry-run conversion, and tests. Channel
reordering occurs only after calibration, when the serial bridge builds the
Arduino frame.

## Command ownership and Arduino ARM are separate

VOLT has two independent safety layers:

1. **ROS command ownership** selects which software source may drive the
   router. Startup is `HOLD`. The GUI exposes `ENABLE MOTION`, `HOLD`, and
   `DISABLE OUTPUT COMMANDS`, publishing `MOTION`, `HOLD`, or `DISABLED` to
   `/volt/command_owner`.
2. **Arduino ARM state** controls whether the firmware accepts live servo
   frames. The GUI exposes guided `ARM SYSTEM SAFELY`, `HOLD SERVOS`,
   `DISARM ARDUINO`, `DISABLE SERVO OUTPUTS`, and `REQUEST STATUS` through
   `/volt/serial_command`.

For normal gait-driven physical movement, the ROS owner must be `MOTION` and
the Arduino must be ready and explicitly armed. The guided button requests
both layers in order, but each remains independently verified and revoked.
The normal serial bridge also requires a fresh
`/volt/command_router_status` report confirming `MOTION`; ownership loss or a
stale router report cancels the ARM request, inhibits new physical frames, and
sends firmware `HOLD` when connected. `owner`, `owner_fresh`, `owner_age`, and
`owner_allowed` in `/volt/serial_status` expose that interlock. `MANUAL` and
`CALIBRATION` are reserved internal router owners for their specialized tools.

Router behavior:

- `HOLD` republishes the last valid pose. Simulation seeds that pose from
  measured canonical joint feedback and never manufactures an all-zero pose.
  The explicitly open-loop hardware-only profile instead publishes its
  documented, firmware-aligned assumed `WALK_POSE` after the operator selects
  `MOTION`.
- `DISABLED` publishes no output.
- Wrong-length, non-numeric, NaN, and infinite arrays are rejected.
- A stale active source falls back to `HOLD`.
- Starting the GUI never claims `MOTION`. The operator must press
  `ENABLE MOTION` for ordinary control or confirm `ARM SYSTEM SAFELY` for the
  bounded guided-arm sequence.

The GUI publishes non-zero velocity only while the controller is standing and
the active owner is `MOTION`. Releasing the control or losing the gamepad sends
zero velocity and stops persistent step-in-place. While step-in-place is
enabled, the GUI renews a short controller lease; loss of that keepalive stops
new steps after at most one second and completes any airborne foot's touchdown.
Closing the GUI sends zero velocity, requests a controller stop, changes the
owner to `HOLD`, and asks the Arduino to hold.

## Real-hardware profiles and atomic tuning

`config/real_robot_profiles.yaml` supplies four complete profiles:

- `SIMULATION`: unchanged 5.20 s `spotmicro_video_walk` behavior
- `REAL_DIAGNOSTIC`: 2.00 s, 80%-duty one-leg-at-a-time
  `diagnostic_crawl`, 35 mm stride and 40 mm clearance
- `REAL_SAFE`: 1.20 s load-safe `real_safe_trot`, 35 mm stride and 36 mm
  clearance
- `REAL_NORMAL`: a later 0.90 s, 50 mm `real_safe_trot` reference; it is not
  the first physical profile

Hardware mode starts at `REAL_DIAGNOSTIC`; simulation starts at `SIMULATION`.
The GUI keeps `SIMULATION` visible but disables its value editors plus Apply
and Save, preserving the proven simulator gait as a read-only reference.
The GUI's Real Robot Tuning panel edits gait/cycle/stride/lateral width/step
height/duty, complete body XYZ/RPY, joint velocity and acceleration,
smoothing, touchdown softness, and stance half-width. Loading or resetting is
local only. Apply sends one complete, correlated JSON transaction on
`/volt/real_robot_tuning`.

The controller applies nothing unless the robot is fully stopped with neutral
velocity and no gait, transition, step, diagnostic, or emote active. It checks
schema/ranges and preflights stance and swing extrema through IK; projection
or joint clamping rejects the entire transaction. `/volt/status` echoes the
same request ID with `applied` or `rejected`, a message, the effective profile,
complete values, bounds, and joint limits. This makes body, gait, and
conditioning changes atomic rather than a series of partially live edits.

**Save Profile** validates and atomically writes a GUI overlay at
`$XDG_CONFIG_HOME/volt_description/real_robot_profiles.yaml`, or
`~/.config/volt_description/real_robot_profiles.yaml`. It does not modify the
installed YAML; the controller does not silently load this GUI overlay and
does not apply its values until the operator presses Apply. See
[CONTROL_GUI.md](src/volt_description/CONTROL_GUI.md) for exact
fields, bounds, built-in values, and request format.

## Integrated emotes and diagnostics

The Robot Emotes panel exposes ten controller-owned Cartesian motions:
Push-ups, Body roll, Nod, Wave left/right, Heart, Bow, Stretch, Happy dance,
and Shake no. The nearby Sit and Stand Up actions use a planted-foot Cartesian
sequence: rearward shift, asymmetric rear-leg bend/lower, front-leg support,
settle, and exact reverse through IK. `/volt/emote` start/keepalive/cancel JSON
is correlated by request ID.
Playback requires standing plus `MOTION`, waits for locomotion to settle, and
preflights the composed path through normal IK. STOP cancels; an active emote
returns smoothly to its captured commanded stand in about one second. The GUI
renews a 750 ms controller lease every 200 ms while a request is queued,
running, returning, or settling. Loss of the GUI/client therefore cancels a
queued request or begins the same conditioned return from an active emote.
Owner loss is an immediate HOLD/reset and can freeze the last command rather
than perform the smooth STOP return. See
[EMOTES.md](src/volt_description/EMOTES.md) for the catalog and safety contract.

## Mirrored face LEDs

The firmware drives two physical 8-LED WS2812B/NeoPixel strips from one Nano
pin. Both `DIN` wires are connected in parallel, so the two strips receive the
same signal and always mirror one another. `NUM_FACE_LEDS` is therefore `8`,
not `16`: logical pixel 0 appears as pixel 0 on both strips and the strips
cannot be addressed independently.

| Connection | Destination | Requirement |
|---|---|---|
| Nano `D6` | Both strip `DIN` inputs in parallel | Change only the clearly marked `LED_PIN` constant if rewired |
| External regulated `+5 V` | Both strip `5V` inputs | Size the supply and wiring for all 16 physical LEDs |
| Common ground | Nano, PCA9685, Jetson/controller, LED supply, both strips | Connect before applying LED data or power |

> **Power warning:** do not power the two strips or the servos from the Nano.
> Use the external regulated 5 V LED supply and a common ground. The global
> `FACE_BRIGHTNESS_LIMIT` (160 by default) caps every effect even if a client
> requests brightness 255; it is a software current reduction, not a substitute
> for correctly sized power wiring.

The Nano owns all animation timing in a nonblocking `millis()` state machine.
Servo parsing and updates run first; face frames are limited to 40 Hz and
static patterns call `show()` only when changed. The first sketch instructions
after the Nano bootloader clear the pixels; successful initialization shows a
short cyan startup pattern, and the face then enters `idle`. Normal `DISARM`
leaves the face on; set the firmware's
`FACE_OFF_ON_DISARM` constant to `true` only when the installation requires
otherwise. Graceful GUI/bridge shutdown sends `FACE shutdown` after motion
HOLD, producing a fade to black.

### Face serial and ROS interfaces

All serial commands remain newline terminated. Existing `FRAME`, `SERVO`,
`PING`, `ARM`, `HOLD`, `DISARM`, `DISABLE`, and `STATUS` behavior is unchanged.
The added face command reference is:

```text
LED COLOR <r> <g> <b>
LED COLOR_B <r> <g> <b>
LED BRIGHTNESS <0-255>
LED EFFECT <solid|breathe|blink|pulse|rainbow|chase|scanner|sparkle|alternate|loading|off>
LED SPEED <milliseconds>
LED PIXEL <index> <r> <g> <b>
LED CLEAR
LED OFF
LED STATUS
FACE <neutral|idle|happy|excited|love|sad|angry|alert|thinking|confused|sleeping|success|error|scared|playful|shutdown>
```

RGB, requested brightness, speed, and pixel index are safely clamped; malformed
values, extra fields, unknown effects, and unknown expressions return a readable
`ERR ...` response. `LED STATUS` reports the requested/effective brightness,
hard limit, color, effect, speed, enabled state, and active expression.

The bridge exposes typed Humble-compatible topics and coalesces changed values
into a bounded, acknowledged serial queue behind servo/safety traffic:

| Topic | Type | Value |
|---|---|---|
| `/volt/face/expression` | `std_msgs/msg/String` | Lowercase preset |
| `/volt/face/color` | `std_msgs/msg/ColorRGBA` | RGB normalized to `0.0..1.0`; alpha ignored |
| `/volt/face/alternate_color` | `std_msgs/msg/ColorRGBA` | Preset RGB B for alternate/chase effects; alpha ignored |
| `/volt/face/brightness` | `std_msgs/msg/UInt8` | Requested `0..255` |
| `/volt/face/effect` | `std_msgs/msg/String` | Lowercase public effect |
| `/volt/face/speed` | `std_msgs/msg/UInt32` | Milliseconds, clamped to `10..60000` |

`/volt/serial_status` adds face connection/support/synchronization, effective
state, queue counters, and recoverable LED errors. On Arduino reconnect the
bridge replays only the latest desired snapshot; the Jetson never streams
animation frames.

### GUI, presets, and emote automation

The control GUI's **Face LEDs** panel provides enable/disable, automatic and
manual-lock toggles, all sixteen preset buttons, expression/effect selectors,
a synchronized QColor picker and RGB controls, live preview, brightness and
speed sliders, Apply, Off, Restore Default, and a nonblocking RGBW test. It
atomically saves the last snapshot to
`$XDG_CONFIG_HOME/volt/face_led_settings.json` or
`~/.config/volt/face_led_settings.json`.

Colors, effects, and behavior mappings live together in
`src/volt_description/config/face_expressions.yaml`:

| Robot behavior | Automatic face |
|---|---|
| Sit | `neutral` ice blue |
| Stand / wake | `success` green, then previous manual face (idle by default) |
| Push-ups | `angry` red pulse |
| Body roll | `playful` rainbow |
| Heart | `love` heartbeat |
| Wave / bow | `happy` warm-yellow pulse |
| Happy dance / dance | `excited` cyan-magenta chase |
| Sleep / lie down | `sleeping` dim-blue breathing |
| Walking / trot | `idle` cyan breathing |
| Calibration | `thinking` purple loading |
| Emergency / critical fault | `error` red flash |
| Reported low voltage | `alert` red/amber |

Manual lock blocks ordinary emote and walking changes but never a safety
override. Completion or cancellation restores the saved manual snapshot.
There is currently no battery-voltage telemetry in VOLT, so the low-voltage
mapping is ready and tested but remains dormant until a monitor reports
`low_voltage` or `undervoltage` status.

To tune an existing preset, edit its entry in `face_expressions.yaml`. To add a
new expression, add a lower-case entry there, add any mappings, and implement
the same name in the firmware's `FaceExpression`, `parseFaceExpression()`,
`printFaceExpressionName()`, and `applyFacePreset()` tables. Restart the GUI
after changing YAML. To move the hardware data signal, change only `LED_PIN`
near the top of `volt_arduino_pca9685.ino`; leave `NUM_FACE_LEDS=8` for the
parallel wiring.

### Raised face-hardware test

Keep VOLT rigidly supported with its feet clear and leave servo power off or
the firmware disarmed. Use a 57600-baud serial terminal with newline endings,
and wait for `OK VOLT_PCA9685_READY ... FACE_SUPPORTED=1` after opening it.

1. Confirm reset clears both strips, cyan startup appears, and idle breathing
   follows. Send `PING`, `STATUS`, and `LED STATUS`.
2. Set `LED BRIGHTNESS 20`; test solid red, green, blue, and white with
   `LED COLOR ...` followed by `LED EFFECT solid`.
3. For indexes 0 through 7, send `LED CLEAR` then
   `LED PIXEL <index> 255 255 255`; the same position must illuminate on both
   strips. Software cannot distinguish them, so this is the mirrored-wiring
   verification.
4. Test requested brightness 0, 20, 80, 160, and 255 and confirm `LED STATUS`
   never reports an effective value above `FACE_BRIGHTNESS_LIMIT`.
5. Exercise every effect and every `FACE` preset listed above, including the
   success/error sequences and shutdown fade.
6. Send malformed, missing/extra-field, unknown-name, and out-of-range inputs;
   expect safe clamps or `ERR ...`, never a reset.
7. Rapidly alternate valid face settings for 30 seconds. Then close the raw
   terminal, run the bridge/GUI, disconnect/reconnect the Arduino, and confirm
   the selected snapshot resynchronizes.
8. Still raised, follow the existing Stand/STOP/ARM procedure and run a slow
   servo stream while changing effects. Confirm smooth servo motion, prompt
   face acknowledgements, emote restoration, manual lock, and an injected
   emergency/fault override before considering any floor test.

The firmware README contains the expanded step-by-step procedure and exact
acknowledgements: [firmware/volt_arduino_pca9685/README.md](firmware/volt_arduino_pca9685/README.md).

With `enable_physical_tests:=true`, the Hardware Gait Diagnostic panel offers:

- A: normal Stand transition
- B: finite leased Slow Squat
- C: finite leased Single Leg Lift
- D: finite leased Step One Leg
- E: stop and select `diagnostic_crawl`
- F: stop and select `real_safe_trot`

B–D renew validated `/volt/physical_test` JSON every 200 ms. Loss of a matching
keepalive for 750 ms causes a smooth one-second commanded return. E/F select a
gait only; after status confirms it, the operator supplies the minimal
joystick input. Diagnostic/emote STOP does not itself change `MOTION` ownership
or firmware ARM, so wait for the commanded return and then use HOLD/DISARM as
required. Full commands and the support-stand ladder are in
[PHYSICAL_TESTS.md](src/volt_description/PHYSICAL_TESTS.md).

## Walking modes

The GUI starts on `spotmicro_video_walk` at a 20% speed limit with a zero
motion command. Available canonical gait names are:

- `spotmicro_video_walk`: slow video-style stable crawl with explicit support
  verification, touchdown, and settle states
- `spot_walk`: conservative, one-foot-at-a-time statically stable crawl,
  shown in the GUI as **VOLT STABLE WALK**
- `diagnostic_crawl`: real-hardware profile crawl used by `REAL_DIAGNOSTIC`
- `real_safe_trot`: bounded real-hardware diagonal gait used by `REAL_SAFE`
  and `REAL_NORMAL`
- `legacy_walk`: the original VOLT walk configuration, preserved explicitly
- `amble`: the existing quasi-static amble
- `slow_trot`, `normal_trot`, `fast_trot`: existing world-locked diagonal
  phase trots

For backward compatibility, `walk` resolves through the explicit alias table
to `spotmicro_video_walk`, and status reports that canonical name. The
pre-existing `spot_walk` remains independently selectable. The legacy `trot`
alias resolves to `normal_trot`. There is no duplicate `spot_trot`: upstream
did not provide a meaningfully different trot worth duplicating, and the
existing VOLT trots already retain the required diagonal pairs:

```text
Pair A: front_left + rear_right
Pair B: front_right + rear_left, one half-cycle after Pair A
```

The tuned full-scale forward envelopes are 0.090 m/s at 1.45 Hz for slow and
0.130 m/s at 1.80 Hz for normal. `fast_trot` is now a separate Cartesian
physical-trot profile: its configured sweep is 55 mm, simulation period is
0.42 s, and hardware starts with the BENCH preset at a 0.75 s period and
20.625 mm full-command effective stride. Those periods are requested clock
rates: a downstream tracking governor stretches the complete Cartesian/body
clock when the velocity/acceleration-limited command falls behind, and a
1-degree endpoint gate prevents an airborne foot from entering scheduled
stance. Status reports the observed period and only counts signed
touchdown-to-liftoff stride when commanded stance stays within 2 mm of ground.
The GUI can deliberately request the FLOOR TEST and WIDE presets only while
stopped. See
[FAST_TROT.md](src/volt_description/FAST_TROT.md) for its limits, diagnostics,
and raised-test sequence. A trot is not the first recommended physical gait.

### `spotmicro_video_walk` behavior

This is the video-matched, deliberately slow stable crawl. Each leg operation
passes through `SHIFT`, `VERIFY_SUPPORT`, `LIFT_AND_SWING`, `TOUCHDOWN`, and
`SETTLE`; completing a shift timer alone never authorizes lift. The
controller's commanded-trajectory completion and joint tracking error provide
the initial support/touchdown checks until contact sensors are available.

The eight principal phases and VOLT body-shift targets are:

| Phase | Body target (+x forward, +y robot-left) | Swing |
|---:|---|---|
| 0 | front-left: `(+0.020, +0.018) m` | none; verify support |
| 1 | hold front-left | `rear_right` |
| 2 | back-left: `(-0.010, +0.018) m` | none; verify support |
| 3 | hold back-left | `front_right` |
| 4 | front-right: `(+0.020, -0.018) m` | none; verify support |
| 5 | hold front-right | `rear_left` |
| 6 | back-right: `(-0.010, -0.018) m` | none; verify support |
| 7 | hold back-right | `front_left` |

Thus the lift order is `rear_right → front_right → rear_left → front_left`;
only one foot lifts, all four feet remain planted during shifts, and the other
three world-locked footholds support each swing. The requested body target is
projected inside the three-foot support region with its configured margin. If
the region is invalid or the commanded shift has not reached its threshold,
`lift_allowed` remains false and the warning is published instead of lifting.

Swing is generated in Cartesian foot space using a zero-endpoint-velocity
horizontal blend and smooth rounded vertical bump. The 28 mm default clearance
is tunable, as are timing, foothold bounds, support shifts, stance offset, and
simulation/hardware time scales. A stop or expired command completes touchdown
and settle, centres the body, and leaves all feet planted. Switching gaits
follows that same grounded transition and then waits for a fresh non-zero
command.

The `/volt/status` fields specific to verification include `phase_index`,
`phase_name`, `body_shift_target_x/y`, `shift_completion`,
`support_polygon_valid`, `lift_allowed`, `projected_targets`,
`clamped_joints`, joint tracking error, and the current warning. The GUI shows
these fields alongside requested/active/pending gait, command ownership, and
Arduino state.

### `spotmicro_video_walk` configuration

The initial profile in `config/gait_controller.yaml` is intentionally slow:

| Parameter | Initial value |
|---|---:|
| `full_cycle_duration` | 5.20 s |
| `shift_duration` / `support_verify_duration` | 0.38 / 0.08 s |
| `swing_duration` / `settle_duration` | 0.68 / 0.16 s |
| `step_height` | 0.028 m |
| `max_step_x` / `max_step_y` / `max_yaw_step` | 0.035 m / 0.015 m / 0.10 rad |
| `forward_body_shift` / `rearward_body_shift` | 0.020 / 0.010 m |
| `lateral_body_shift` / `support_margin` | 0.018 / 0.012 m |
| `shift_completion_threshold` | 0.92 |
| `gait_body_height_offset` / `body_bob_amplitude` | -0.010 / 0.001 m |
| `simulation_time_scale` / `hardware_time_scale` | 1.0 / 1.20 |
| `hardware_speed_scale` | 0.20 |

The hardware time scale makes the same phase trajectory 20% slower, while the
separate 20% hardware speed scale caps command amplitude. These are
conservative starting values, not evidence that the physical robot has been
tested.

### `spot_walk` behavior

The upstream-derived lift order is:

```text
rear_right → front_right → rear_left → front_left
```

One complete cycle has eight alternating phases:

| Phase | Action |
|---:|---|
| 0 | Shift the body projection into the three-foot support region before `rear_right` |
| 1 | Swing `rear_right`; the other three feet remain in stance |
| 2 | Shift before `front_right` |
| 3 | Swing `front_right`; the other three feet remain in stance |
| 4 | Shift before `rear_left` |
| 5 | Swing `rear_left`; the other three feet remain in stance |
| 6 | Shift before `front_left` |
| 7 | Swing `front_left`; the other three feet remain in stance |

The body frame is VOLT's convention: +x forward, +y robot-left, and +z up.
VOLT's URDF, link lengths, nominal feet, joint limits, and IK remain
authoritative; upstream Spot Micro geometry and servo conventions are not
used. The studied upstream kinematics use x forward, y up, and z
robot-right, so the conceptual point mapping is
`(x_VOLT, y_VOLT, z_VOLT) = (x_upstream, -z_upstream, y_upstream)`.
Upstream positive lateral/yaw sense is correspondingly opposite the ROS/VOLT
sense; the implementation preserves VOLT signs instead of copying upstream
commands.

Before each lift, the controller forms a support triangle from the other three
world-locked feet. It selects a bounded body x/y shift toward that support
region, adds conservative travel anticipation, and projects the result inside
the triangle with the configured margin. The support shift completes before
the next control sample is classified as airborne. Support shifts are body
translations, not servo-angle offsets.

The support calculation constrains the sum of the gait shift and the
operator-requested body x/y offset. If an extreme offset cannot be countered
within the configured shift bounds, only its unsafe remainder is temporarily
clipped during that single-foot support phase, then restored while all four
feet settle.

At lift-off, the next foothold is frozen in the world frame. Swing x/y uses a
quintic blend, and z uses a smooth lift/touchdown bump; both have zero endpoint
velocity and acceleration. During swing, the other three feet stay locked in
the world frame and are transformed back into the moving body frame before
VOLT IK. This reduces commanded foot skating without changing the robot's
geometry.

`spot_walk` supports explicit step-in-place, slow forward/reverse travel,
conservative yaw, and limited lateral travel. Simultaneous lateral and yaw
commands are automatically reduced when their normalized combination becomes
too large.

### Stopping and switching gaits

A zero/expired velocity command or normal STOP request does not reset a foot in
midair:

- During a support shift, no new foot is lifted; the controller begins
  settling.
- During a swing, that foot finishes touchdown first.
- All feet then return smoothly to nominal stance over `settle_duration`.

An owner change to `HOLD` or `DISABLED` is the immediate emergency boundary:
the router retains its last valid pose, so it can freeze an airborne foot. For
a planned stop, send `STOP`, wait for status to show four stance feet, and only
then change owner to `HOLD`.

When a different gait is selected during motion, it becomes `pending_gait`.
The controller ramps velocity toward zero, completes any active swing, grounds
all feet, settles, and only then activates the requested gait. Movement does
not resume until a new velocity command arrives. The GUI and `/volt/status`
show the requested, active, and pending names throughout the transition.

## `spot_walk` configuration

The authoritative tuning is
`src/volt_description/config/gait_controller.yaml`. Do not duplicate these
limits in the GUI; the controller publishes effective limits in `/volt/status`.

| Parameter | Initial value | Meaning |
|---|---:|---|
| `cycle_period` | 4.80 s | Four complete shift/swing pairs |
| `support_shift_duration` | 0.60 s | Body shift time before each lift |
| `swing_duration` | 0.60 s | Time for one foot swing |
| `step_height` | 0.014 m | Swing clearance |
| `maximum_step_x` | 0.028 m | Fore/aft foothold bound |
| `maximum_step_y` | 0.010 m | Lateral foothold bound |
| `maximum_yaw_rate` | 0.14 rad/s | Full-scale yaw limit |
| `body_shift_x` | 0.012 m | Maximum fore/aft support shift |
| `body_shift_y` | 0.015 m | Maximum lateral support shift |
| `support_margin` | 0.004 m | Requested inset; infeasible nominal-triangle values are rejected |
| `touchdown_lead` | 0.50 | Stance-time fraction used for foothold look-ahead |
| `settle_duration` | 0.80 s | Return-to-nominal time after stopping |
| `velocity_deadband` | 0.002 | Normalized command activity treated as zero |
| `command_acceleration` | 0.08 | Conservative command ramp parameter |
| `velocity_filter_alpha` | 0.18 | Velocity low-pass blend |
| `joint_smoothing_alpha` | 0.10 | Stable-walk joint smoothing |
| `maximum_body_roll` | 0.08 rad | Stable-walk body-pose bound |
| `maximum_body_pitch` | 0.08 rad | Stable-walk body-pose bound |
| `simulation_speed_scale` | 1.00 | Simulation profile scale |
| `hardware_speed_scale` | 0.25 | Maximum initial hardware profile scale |

These are conservative initial values, not a declaration that an untested
physical robot is safe. Start the raised physical test at only 10–20% on the
GUI speed slider.

## Time behavior

Time selection is owned by launch files, not by the shared gait YAML:

| Mode | Motion/controller time | Serial bridge time |
|---|---|---|
| Ignition-only | Gazebo `/clock`, `use_sim_time:=true` where applicable | Not started |
| Combined | Gazebo `/clock`, `use_sim_time:=true` | System time |
| Hardware-only | System time, `use_sim_time:=false` | System time |

The GUI's user-interface timing does not wait for a simulation clock. A
hardware-only run must not load an old shared configuration that forces
`use_sim_time=true`; its timers and command timeouts must advance without
`/clock`.

## Servo calibration and `FRAME`

`src/volt_description/config/servo_calibration.yaml` is authoritative for:

- PCA9685 channel
- servo direction
- neutral angle
- trim
- minimum and maximum physical angle
- pulse-width metadata

For each canonical ROS joint value, the bridge computes:

```text
servo_deg =
    neutral_deg
    + trim_deg
    + direction * degrees(ros_joint_radians)
```

It then clamps to that servo's physical limits and reorders the results by PCA
channel. Right-side inversion belongs only in this calibration step; never add
a second inversion to the gait, IK, router, or Arduino.

The Arduino protocol remains:

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

`FRAME` contains exactly 12 **physical servo angles in degrees**, already
calibrated and ordered by PCA channel 0 through 11. It is not radians and is
not canonical ROS joint order. Frames are newline-terminated and the bridge
limits transmission to 30 Hz. Gait mathematics remains entirely outside the
Arduino.

## Safe serial dry-run

1. Start the hardware-only command above with `hardware_enabled:=false`,
   `dry_run:=true`, `auto_arm:=false`, and `use_sim_time:=false`.
2. Start the GUI in a second sourced terminal.
3. Inspect `/volt/command_router_status`, `/volt/status`, and
   `/volt/serial_status`.
4. Press `ENABLE MOTION`, then confirm the router receives the documented,
   firmware-aligned 12-joint `WALK_POSE` (`pose_valid=1`).
5. Select `VOLT WALK`, stand through the controlled transition,
   press STOP, and use explicit step-in-place at 10–20%.
6. Inspect the bridge's `Dry-run conversion` table. Each row shows the
   canonical joint name, ROS radians, calibrated degrees, PCA channel, and any
   clamp. The logged line must begin with `FRAME` and contain 12 values.
7. Verify every mapping and direction against the raised robot before changing
   either hardware gate.

Dry-run or `hardware_enabled:=false` never opens the serial port. When live
hardware is eventually enabled, the firmware still starts disarmed with
outputs disabled. Pressing ARM is a separate deliberate action.

## First physical test checklist

1. Complete simulation and the hardware-disabled dry-run with
   `enable_physical_tests:=true`; verify all 12 joint/channel mappings and
   physical directions.
2. Secure VOLT on a rigid stand with every foot clear, keep the power
   disconnect reachable, and confirm `/dev/ttyUSB1` is the actual Arduino.
3. Start exactly one live stack with `auto_arm:=false`, confirm fresh
   controller/router/protocol-2 serial status, and deliberately use **ARM
   SYSTEM SAFELY** to obtain `MOTION` plus firmware ARM.
4. Select **A — STAND** and wait for stopped `STANDING` status.
5. Run one default **Push-ups** emote; wait for its commanded return, STOP, and
   inspect.
6. Run **C — SINGLE LEG LIFT** separately for all four canonical legs,
   stopping and inspecting between runs.
7. Run **D — STEP ONE LEG** separately for all four legs at 6 s or longer.
8. Stop fully, load and Apply `REAL_DIAGNOSTIC`, use **E — SELECT SLOW CRAWL**,
   wait for `diagnostic_crawl`, then give the smallest brief joystick input.
9. Stop fully, load and Apply `REAL_SAFE`, use **F — SELECT SAFE TROT**, wait
   for `real_safe_trot`, then give the smallest brief joystick input.
10. Increase speed or one tuning variable only after clean complete cycles.
    Never jump automatically to `REAL_NORMAL`.
11. End with STOP, wait for every commanded foot to settle, then HOLD and
    DISARM. Keep the stand installed; a passing open-loop test is not floor
    validation.

See [PHYSICAL_TESTS.md](src/volt_description/PHYSICAL_TESTS.md) before live
power. Nothing in this sequence reads actual servo position or contact, and no
documented launch ever auto-arms.

## Validation commands

Run the automated suite after rebuilding:

```bash
colcon test --packages-select volt_description
colcon test-result --verbose
```

With the headless simulator running, useful checks are:

```bash
ros2 topic echo /clock --once
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /joint_states
ros2 topic info /joint_group_position_controller/commands
ros2 node list
```

Expected active controllers are `joint_state_broadcaster` and
`joint_group_position_controller`. The combined launch should contain one each
of `robot_state_publisher`, `controller_manager`,
`volt_joint_command_router`, and `volt_motion_controller`; controller spawners
exit after activating their controllers.

## Troubleshooting

### Feet slide in simulation

- Confirm `/clock` is advancing and the motion controller has
  `use_sim_time=true`.
- Confirm both controllers are active and the router is forwarding a valid
  12-value command.
- Reduce speed, lateral input, and yaw, then inspect `phase_name`,
  `stance_legs`, `body_shift`, and `projected_targets` in `/volt/status`.
- Check Gazebo ground contact/friction and confirm meshes/links loaded. The
  commanded stance feet are world-locked, but this is not measured contact
  feedback.
- Do not compensate for sliding with servo trims or gait-side angle offsets.

### A right-side joint moves in the wrong direction

- Stop and disarm. Verify the joint-to-channel mapping in dry-run.
- Correct `direction` only in `servo_calibration.yaml`.
- Do not invert the same joint in gait/IK or firmware; simulation must continue
  receiving canonical ROS radians.

### IK projection or servo clamping appears

- `projected_targets` in `/volt/status` means a requested foot target was
  projected into the VOLT workspace or joint limits.
- `clamped=` in `/volt/serial_status` means calibrated physical degrees reached
  a servo limit. These are distinct layers.
- Reduce stride, body translation/roll/pitch, step height, or yaw. Recheck URDF
  geometry and physical calibration rather than suppressing the warning.
- Non-finite targets are rejected; do not bypass that validation.

### Controller manager or controllers do not start

- Source ROS Humble and the freshly built workspace.
- Check `ros2 pkg prefix ros_gz_sim`, `ros_gz_bridge`, and
  `gz_ros2_control`.
- Use `ignition.launch.py`, not the legacy Classic launch, then inspect
  `ros2 control list_controllers` and
  `ros2 control list_hardware_interfaces`.
- Confirm the VOLT entity spawned and the
  `gz_ros2_control/GazeboSimSystem` plugin loaded before debugging the
  controller spawners.

### `/clock` is absent

- In simulation, confirm the `clock_bridge` node and Gazebo server are running,
  then use `ros2 topic echo /clock --once`.
- In hardware-only mode, no `/clock` is required or expected. Confirm
  `use_sim_time:=false`; otherwise motion timers appear frozen.

### The router remains in `HOLD`

- `HOLD` is intentional at startup. Press `ENABLE MOTION` or publish the valid
  `MOTION` owner explicitly.
- Inspect `/volt/command_router_status` for `pose_valid=1` and the
  position-controller connection.
- Confirm the motion controller is publishing finite arrays of exactly 12
  values to `/volt/joint_commands/motion`. A stale source returns to `HOLD`.

### The Arduino will not become ready or ARM

- Confirm `start_serial_bridge:=true` when using the combined launch, or use the
  hardware-only launch.
- Confirm `hardware_enabled:=true`, `dry_run:=false`, the correct device, 57600
  baud, and an installed `python3-serial`.
- Check calibration validity and the `PING`/ready response in
  `/volt/serial_status`.
- ARM remains locked until controller status is recent, connected, stopped,
  not stepping, and explicitly certifies the exact calibrated open-loop
  `WALK_POSE`. Sitting and arbitrary `HOLD` poses never qualify; Stand and
  settle before ARM.
- If the GUI reports `DUPLICATE VOLT STACK — ARM LOCKED`, stop the extra
  simulation/hardware launch. Do not work around this lock; one publisher is
  required on each critical status topic.

### Meshes or package resources are missing

- Rebuild with `--symlink-install` and source `install/setup.bash`.
- Confirm `ros2 pkg prefix volt_description` resolves to this workspace.
- Do not add hard-coded home/workspace paths or Gazebo Classic resource
  variables to the active launch.

## Current limitations

- `spot_walk` is a conservative open-loop commanded gait, not a full dynamic
  stability controller. Support is estimated from commanded foot geometry;
  there is no measured center-of-mass, force, slip, or terrain feedback.
- World-locked stance targets reduce commanded skating, but cannot guarantee
  physical ground contact or compensate for compliance, backlash, a loose
  surface, or an incorrect collision/friction model.
- The Arduino/PCA9685 path commands servo positions but does not return physical
  joint-position or contact feedback through the current protocol.
- A normal GUI close and gamepad disconnect issue the documented stop/HOLD
  sequence. An ungraceful GUI process kill cannot publish that sequence:
  the 0.6-second velocity timeout and one-second step keepalive lease stop
  commanded motion, but the router may retain `MOTION` while the still-running
  motion controller publishes stationary poses. Treat process supervision and
  the accessible servo-power disconnect as required safeguards; a future
  dedicated GUI ownership heartbeat/lease would also revoke ownership and ARM.
- The current firmware source sets `MAX_DEG_PER_SECOND = 120.0`; this has not
  been physically validated. The real-profile host limits are 100 deg/s
  (`REAL_DIAGNOSTIC`), 110 deg/s (`REAL_SAFE`), and 120 deg/s
  (`REAL_NORMAL`); the separate physical `fast_trot` path retains its own
  bounded conditioning. If the Nano still runs the older 30 deg/s image,
  reflash the current firmware before using these profiles. Do not raise host
  or firmware limits to force tracking, and complete the suspended progression
  before any floor test.
- The initial tuning targets flat, raised-bench validation. Rough terrain,
  slopes, payload changes, aggressive lateral/yaw combinations, and dynamic
  recovery are outside its current scope.
- `legacy_walk` and the older diagonal trots are preserved for compatibility,
  but they are not substitutes for the `REAL_DIAGNOSTIC` then `REAL_SAFE`
  progression.
- Simulation, automated tests, and serial dry-run do not constitute physical
  testing. Servo calibration and the complete checklist remain mandatory.

## Attribution

The gait sequence and support-shift concepts were cleanly adapted from
`mike4192/spotMicro` at upstream commit
`2a34f5d303dff91b62180031b31ef512a672f3c3`. VOLT retains its own geometry,
kinematics, trajectories, ROS 2 interfaces, safety behavior, and hardware
mapping. See
[src/volt_description/THIRD_PARTY_NOTICES.md](src/volt_description/THIRD_PARTY_NOTICES.md)
for the provenance statement and MIT license.
