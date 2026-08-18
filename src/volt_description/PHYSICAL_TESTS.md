# VOLT Physical Test Procedure

> **STOP — support stand required.** Before the first live physical command,
> secure VOLT on a rigid support stand that can carry the complete robot, with
> every foot clear of the floor. Keep the servo-power disconnect within
> immediate reach. Do not rely on ROS STOP, serial HOLD, or the command-line
> acknowledgement as a physical restraint.

The procedures below exercise open-loop hardware. They have not been run or
validated on the physical robot by this software change. The operator remains
responsible for restraint, clearance, power-system ratings, mechanical
inspection, and deciding whether to proceed.

## Safety contract

`volt_physical_tests.py` runs one explicit finite mode and has no default:

- `--mode` and `--execute` are mandatory.
- Every non-emergency mode requires the exact typed acknowledgement
  `--acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'`.
- Cartesian test support is disabled in normal launches. Test launches must
  explicitly set `enable_physical_tests:=true`.
- Hardware controllers reject `use_sim_time:=true`; finite leases, STOP
  returns, and cleanup deadlines always use advancing wall/system time.
- The tool never sends `ARM`.
- It uses the existing motion-controller, gait, router, and serial interfaces;
  it never publishes raw joint arrays or bypasses IK/calibration limits.
- Normal completion, interruption, and error cleanup send request cancel when
  applicable, zero `/cmd_vel`, and STOP until fresh status confirms three
  grounded/inactive samples (or a six-second timeout), then ROS owner HOLD and
  firmware HOLD.
- The runner requires a fresh physical-profile status, explicit test
  enablement, a ready dry-run or acknowledged live ARM, controller acceptance,
  observed gait activity, and final settling. Rejected or ineffective requests
  exit nonzero.
- Because firmware HOLD clears the armed state, use the guided ARM workflow
  again before a later test.
- The default emergency mode sends zero velocity, STOP, owner HOLD, and
  firmware DISARM continuously for up to four seconds while waiting for DDS
  discovery and router/serial acknowledgement. It exits nonzero if this is not
  confirmed. `--disable-output` sends firmware DISABLE instead and can allow
  the robot to collapse.

Omitting `--execute`, mistyping the acknowledgement, omitting `--leg` for a
single-leg lift/step, or requesting an invalid duration performs no ROS action.
The typed sentence is an auditable operator assertion, not a sensor check.

## Current GUI diagnostic A–F workflow

Launch with `enable_physical_tests:=true` to expose the Hardware Gait
Diagnostic panel. Hardware mode, the explicit test enable, `MOTION` ownership,
a stopped finite standing command, neutral body pose, nominal footprint, zero
requested/filtered velocity, and no gait, transition, emote, or other
diagnostic must all be true before a leased test can start.

| Button | What it does |
| --- | --- |
| A — STAND | Uses the normal `/volt/action` `stand` transition. It is not a leased diagnostic. |
| B — SLOW SQUAT | Starts the finite `slow-squat` Cartesian diagnostic. All four commanded feet stay planted while the body lowers by at most 18 mm. |
| C — SINGLE LEG LIFT | Starts `single-leg-lift` for the selected canonical leg, with at most 20 mm commanded lift. |
| D — STEP ONE LEG | Starts `single-leg-step` for the selected leg, with at most 15 mm lift and 10 mm forward travel. Its valid duration starts at 6 s. |
| E — SELECT AMBLE | Stops, then selects `amble`. It does not start a finite diagnostic; wait for selection and then use minimal joystick input. |
| F — SELECT TROT | Stops, then selects `trot`. It does not start a finite diagnostic; wait for selection and then use minimal joystick input. |

The GUI duration selector is 6–20 s and the leg choices are `front_left`,
`front_right`, `rear_left`, and `rear_right`. B–D use validated JSON on
`/volt/physical_test`. Each start and its subsequent keepalives carry the same
complete tuple:

```json
{"command":"start","mode":"single-leg-lift","duration":8.0,"request_id":"gui-test-1234","leg":"front_left"}
```

The GUI sends a matching `keepalive` every 200 ms. The controller's default
lease expires after 750 ms without a valid keepalive and then takes a bounded,
smooth one-second path back to the nominal commanded stand. A keepalive or
cancel with a mismatched request ID, mode, duration, or leg is ignored.
**STOP DIAGNOSTIC** sends the correlated cancel and the ordinary controller
STOP; it
does not itself change ROS ownership from `MOTION` or disarm firmware. Use
`HOLD`/`DISARM` separately after the commanded return is complete.

The diagnostic display, `/volt/status`, and all foot/body/joint values in this
workflow describe requested or filtered commands and kinematic calculations.
There is no actual servo encoder, foot-contact, load, slip, current, or
voltage feedback in the hobby-servo/PCA9685 path.

## Exact launch paths for `/dev/ttyUSB1`

Use one stack only. The following hardware-only dry-run opens no physical
device but enables the GUI diagnostic controls:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true serial_port:=/dev/ttyUSB1 baud_rate:=57600 \
  hardware_enabled:=false dry_run:=true auto_arm:=false \
  auto_ready_pose:=false use_sim_time:=false \
  enable_physical_tests:=true
```

After all support-stand gates pass, the corresponding live hardware-only
command is:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true serial_port:=/dev/ttyUSB1 baud_rate:=57600 \
  hardware_enabled:=true dry_run:=false auto_arm:=false \
  auto_ready_pose:=false use_sim_time:=false \
  enable_physical_tests:=true
```

For one full Ignition shadow + GUI + physical bridge system, prefer the
runner; do not start `hardware_control.launch.py` alongside it:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1
```

The optional TD-8130MG Ignition dynamics model can be selected without
changing physical calibration:

```bash
ros2 run volt_description volt_run_all.py \
  --physical \
  --serial-port /dev/ttyUSB1 \
  --actuator-profile td8130mg
```

The equivalent explicit combined launch is:

```bash
ros2 launch volt_description volt_start.launch.py \
  gui:=true gazebo_gui:=true start_serial_bridge:=true \
  serial_port:=/dev/ttyUSB1 baud_rate:=57600 \
  use_hardware:=true hardware_enabled:=true dry_run:=false \
  auto_arm:=false auto_ready_pose:=false use_sim_time:=false \
  enable_physical_tests:=true actuator_profile:=td8130mg
```

All examples intentionally retain `auto_arm:=false`. The `--physical` preset
also leaves ARM and automatic ready-pose off; the operator must still use the
guided `ARM SYSTEM SAFELY` action after checking the raised mechanism and
fresh status. Verify that `/dev/ttyUSB1` is the intended Arduino before the
live launch—never substitute a guessed device. At the last local inspection,
the host had no `/dev/ttyUSB*` or `/dev/ttyACM*` node. The documented
`/dev/ttyUSB1` command is therefore the requested target, not proof that the
Arduino is currently connected; do not ARM until the intended node exists and
has been verified.

## Recommended support-stand sequence

This is the current shortest safe progression for the integrated GUI. Do not
begin on the floor and do not jump directly to `REAL_NORMAL`:

1. Run the complete sequence in simulation and hardware-disabled dry-run.
2. Install the rigid support stand, clear all feet, inspect mechanics and
   power, verify `/dev/ttyUSB1`, then start exactly one live stack with
   `auto_arm:=false`.
3. Confirm fresh controller/router/serial status, deliberately use the guided
   ARM workflow, obtain `MOTION` ownership, select **A — STAND**, and wait for
   a stopped `STANDING` state.
4. Run one default **Push-ups** emote. Wait for its smooth commanded return to
   stand, STOP, and inspect. This checks a four-foot command path; it does not
   prove load-bearing contact or walking dynamics.
5. Use **C — SINGLE LEG LIFT** separately for each leg. Stop, wait for the
   commanded return, and inspect direction and clearance after every run.
6. Use **D — STEP ONE LEG** separately for each leg at 6 s or longer. Again
   stop and inspect after each finite leased run.
7. While fully stopped, load and **Apply** `REAL_DIAGNOSTIC`. Select **E —
   SELECT AMBLE**, wait until `amble` is active, then apply the
   smallest brief joystick command. Release, STOP, and wait for all commanded
   feet to settle.
8. While fully stopped, load and **Apply** `REAL_SAFE`. Select **F — SELECT
   TROT**, wait until `trot` is active, then apply the smallest
   brief joystick command. Release and STOP.
9. Increase speed or one tuning variable only after clean complete cycles;
   record the applied profile and result each time. Never treat
   `REAL_NORMAL`, an increased slider, or a larger emote option as an automatic
   next step.
10. Finish with STOP and the complete commanded return, then `HOLD` and
    `DISARM`. Keep the stand installed until every supported stage and power
    check passes.

Passing this sequence means only that the commanded open-loop path survived a
supported test. It does not validate unsupported floor walking.

## Prepare the robot and workspace

Complete this checklist before enabling live hardware:

1. Power the servo rails off.
2. Secure the body on a rigid stand. Confirm it cannot walk off, rotate off,
   or tip the stand and that all links can move without striking it.
3. For the first test, leave all four feet clear of the floor through their
   full expected travel.
4. Inspect servo horns, fasteners, linkages, wires, connectors, fuses, feet,
   and joint mechanical-stop clearance.
5. Confirm the Jetson, Arduino, PCA9685 logic, and both servo-power systems
   share the intended ground reference. Do not power servos from the Nano.
6. Verify both power banks and both converters are correctly set, fused, and
   rated for the connected servo bank before applying power.
7. Place the servo-power disconnect where the operator can reach it without
   entering the leg workspace.
8. Stop unrelated VOLT stacks, GUIs, teleoperation nodes, emote players, and
   CLI publishers.
9. Confirm the Nano runs the repository's protocol-2 firmware. Do not attempt
   live walking with `firmware_compatible=0`, a legacy generic handshake,
   or an older low-slew firmware image.
10. Keep a second sourced terminal ready for the emergency command, while
    treating the physical power disconnect as the final stop.

The current bridge accepts normal live motion only after a protocol-2
capability report and ARM acknowledgement. It sends complete frames at no more
than 30 Hz. The gait engine validates every gait configuration at load
against its servo budgets — 80 deg/s commanded on loaded stance joints,
190 deg/s on the unloaded swing leg, and 6500 deg/s² acceleration — beneath
the firmware's 240 deg/s slew ceiling. Do not raise those values for a first
test.

## Build and source

```bash
source /opt/ros/humble/setup.bash
cd /home/ros2/Documents/volt_ws
colcon build --packages-select volt_description --symlink-install
source install/setup.bash
```

Source `/opt/ros/humble/setup.bash` and
`/home/ros2/Documents/volt_ws/install/setup.bash` in every additional
terminal.

## Gate 1: hardware-disabled dry-run

Start the hardware-mode controller with the physical profile and explicit test
support, but without opening a serial device:

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  enable_physical_tests:=true \
  hardware_enabled:=false \
  dry_run:=true \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false
```

Verify:

```bash
ros2 topic echo --once /volt/status
ros2 topic echo --once /volt/serial_status
ros2 topic info /joint_command_router/output --verbose
ros2 topic info /volt/joint_commands/motion --verbose
```

Expected safety state is hardware disabled, dry-run true, serial disconnected,
firmware disarmed, one intended publisher on each actuator-authority topic,
and `REAL_DIAGNOSTIC` active as the hardware profile. Run every intended
mode in dry-run before the matching live test.
Dry-run checks command generation and cleanup; it cannot check servo direction,
load, power, contact, or mechanics.

## Gate 2: live support-stand stack

> Confirm VOLT is already secured on the support stand before this launch.

```bash
ros2 launch volt_description hardware_control.launch.py \
  gui:=true \
  enable_physical_tests:=true \
  serial_port:=/dev/ttyUSB1 \
  hardware_enabled:=true \
  dry_run:=false \
  auto_arm:=false \
  auto_ready_pose:=false \
  use_sim_time:=false
```

Use the verified device path, such as `/dev/ttyACM0`, if it differs. Do not use
a guessed port.

Before each finite test:

1. Confirm the stand and disconnect again.
2. Confirm `PROTO=2`, `firmware_compatible=1`, serial connected, and no reset
   or duplicate-stack warning.
3. Confirm the GUI reports hardware mode and `REAL_DIAGNOSTIC`.
4. Keep input zero and use `ARM SYSTEM SAFELY` deliberately from the verified
   calibrated open-loop standing seed. The CLI will not ARM for you.
5. Select `STAND`, wait for stable `STANDING`, send STOP, and wait for no
   commanded motion or swing leg.
6. Move clear of every link, then run exactly one command below.
7. After its commanded return, send STOP, then HOLD/DISARM. Inspect diagnostics
   before re-arming for another mode.

Do not set `auto_arm:=true` for this sequence.

## Exact finite test commands

All commands except emergency stop require the exact support-stand
acknowledgement shown. Optional `--duration SECONDS` may be added only within
the limits in the next section.

### 1. Stand

Holds the nominal four-foot Cartesian stance for 5 seconds.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode stand \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 1a. Slow squat

Keeps all four commanded feet fixed while lowering and raising the body by at
most 18 mm with smooth endpoints.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode slow-squat \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 2. Diagonal weight shift

Keeps all feet at ground height while smoothly shifting the body-frame targets
by at most 6 mm fore/aft and 8 mm laterally.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode weight-shift \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

The first live run is still unloaded on the stand and checks direction and
clearance only. A later partial-load check may let the feet touch while the
stand remains installed and capable of carrying the complete robot.

### 3. Single-leg lift

Lifts one selected foot by at most 20 mm and returns it with smooth endpoints.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode single-leg-lift \
  --leg front_left \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

Repeat separately with `front_right`, `rear_left`, and `rear_right`. Do not
change the leg spelling or test more than one leg per invocation.

### 3a. Single-leg step

Lifts one selected foot by at most 15 mm, advances it by at most 10 mm, then
returns it to the nominal commanded stance. Repeat as separate runs; never
change the selected leg during a lease.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode single-leg-step \
  --leg front_left \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 4. Diagonal-pair lift

Lifts `front_left + rear_right`, grounds them, then lifts
`front_right + rear_left`. Maximum lift is 15 mm.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode diagonal-pair-lift \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 5. Zero-stride trot

Selects `trot` and maintains the existing step-in-place heartbeat with
zero `/cmd_vel`.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode zero-stride-trot \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 6. Slow creep

Selects the conservative `amble` gait and requests 0.004 m/s
forward for a finite interval.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode slow-creep \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

### 7. Trot at speed

Selects `trot` and requests 0.030 m/s forward. The mode flag below is the
retained historical CLI identifier; the gait it selects is the canonical
`trot`, using whatever stopped-state profile the operator last applied.

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode trot-speed \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

This support-stand run cannot establish floor propulsion. It checks only the
command path, direction, phasing, clearance, mechanical margin, and gross
power/communication behavior.

### 8. Emergency stop/disarm

This mode does not require or accept the support-stand acknowledgement:

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode emergency-stop \
  --execute
```

If removing PCA9685 output is safer than retaining the last commanded pulses:

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode emergency-stop \
  --execute \
  --disable-output
```

`DISABLE` can remove supporting torque and cause collapse. It is appropriate
only when the stand or another restraint already carries the robot. A newly
started ROS process is not a certified hard emergency stop; DDS discovery,
process scheduling, USB, or the Nano can fail. Cut servo power when continued
torque is unsafe.

## Mode durations and bounds

| Mode | Default | Minimum | Maximum |
|---|---:|---:|---:|
| `stand` | 5 s | 2 s | 20 s |
| `slow-squat` | 7 s | 5 s | 20 s |
| `weight-shift` | 8 s | 6 s | 20 s |
| `single-leg-lift` | 6 s | 4 s | 20 s |
| `single-leg-step` | 8 s | 6 s | 20 s |
| `diagonal-pair-lift` | 10 s | 8 s | 20 s |
| `zero-stride-trot` | 6 s | 3 s | 20 s |
| `slow-creep` | 8 s | 5 s | 20 s |
| `trot-speed` | 6 s | 4 s | 20 s |

Example of a valid finite override:

```bash
ros2 run volt_description volt_physical_tests.py \
  --mode stand \
  --duration 8 \
  --execute \
  --acknowledge-support-stand 'VOLT IS ON A SUPPORT STAND'
```

Longer duration is not a substitute for inspecting between runs. Stop early on
any unexpected motion, sound, heat, smell, voltage behavior, or warning.

## Extended CLI ladder

The integrated GUI's recommended Stand → Push-up → single-leg lift →
single-leg step → Amble → Trot progression appears earlier and is the first
path to follow. The longer ladder below uses only `volt_physical_tests.py`;
it does not replace or accelerate the real-profile progression.

Do not skip ahead to a fully unsupported floor run:

1. Complete build/tests and every planned mode in hardware-disabled dry-run.
2. With power off, inspect mechanics, calibration, both power systems, and
   support clearance.
3. On the unloaded support stand, ARM deliberately and run `stand`.
4. Repeat `single-leg-lift` for each of the four canonical legs. Confirm every
   physical direction and mechanical margin.
5. Run `diagonal-pair-lift`; verify the two exact pairs and that one pair is
   grounded before the other rises.
6. Run unloaded `weight-shift`, then a partially loaded version only while the
   stand remains able to catch the complete robot.
7. Apply `REAL_DIAGNOSTIC` while stopped. Run `zero-stride-trot`.
8. Run `slow-creep` at 0.004 m/s, first unloaded and then only with controlled
   partial foot contact.
9. Apply `REAL_SAFE` while stopped, then run the finite 0.030 m/s trot mode
   while the support stand remains installed.
10. Review the observed behavior and power measurements after every stage.
11. Only after all gates pass may the operator consider partial floor contact
    with the stand/tether still carrying part of the load. Unsupported floor
    walking is a separate, later decision and never applies automatically.

The test runner always requires the robot to remain on the support stand.
Unsupported floor operation is outside this finite-test authorization and has
not been physically validated.

## One-variable-at-a-time tuning

There is no separate sweep helper or live tuning tuple. All stopped-state
tuning goes through the GUI's Real Robot Tuning panel (or an equivalent
complete `/volt/real_robot_tuning` transaction): load a profile, change one
conceptual variable, Apply while fully stopped, run one finite support-stand
test, STOP/HOLD, inspect, and either keep the value or return to the
preceding known-good profile. Record the applied profile, power
measurements, surface, load support, and result each time. Loading a
different profile changes many fields together and is a stage transition,
not a one-variable sweep.

Every Apply is atomic and re-runs the gait engine's servo-budget sweep: a
transaction whose commanded joint speeds would exceed 80 deg/s on loaded
stance joints or 190 deg/s on the unloaded swing leg, or whose commanded
accelerations would exceed 6500 deg/s², is rejected in full and changes
nothing. If the controller rejects a value, restore the preceding value; do
not defeat the check.

Stop tuning when current rises materially, either rail sags, servos buzz or
slow, vibration grows, contact becomes inconsistent, or any software
limit/projection warning persists. Prefer reducing speed or stride when
clamps persist; do not raise velocity, acceleration, calibration, pulse, or
firmware limits to chase an aggressive cycle.

## Authoritative configuration

The two gaits are configured in the `gaits` section of
`config/gait_controller.yaml`: per-gait cycle period, duty factor, step
height, command limits, settle time, body sway, filtering, command
acceleration, hardware speed scale, and the servo velocity/acceleration
budgets. The stopped-state hardware profiles live in
`config/real_robot_profiles.yaml`. Editing either file means stop,
HOLD/DISARM, edit, rebuild/relaunch. Both paths re-run the servo-budget
validation at load; an infeasible configuration refuses to load rather than
being clipped by a downstream limiter.

## Power and wiring checks

There is no battery-voltage, converter-voltage, servo-rail-voltage, or current
sensor in the current workspace. ROS cannot measure these quantities, and the
diagnostic recorder does not invent them. Software slowing may reduce load,
but it must not be used to hide an inadequate supply, current limit, ground,
connector, fuse, or stalled servo.

### Symptoms that point outside the gait code

| Symptom | Check before more tuning |
|---|---|
| Both diagonal pairs or many servos slow together only under load | Battery sag, converter current limiting, common wiring drop, connector heating |
| One servo bank is weaker, slower, or noisier | Compare that bank's battery, converter input/output, rail voltage, current, fuse, wiring, and connectors with the other bank |
| Servo buzzes but does not reach the command | Mechanical bind/stop, excessive load, loose horn, damaged servo, low rail voltage |
| Arduino READY banner repeats, counters reset, or PCA output restarts | Logic brownout, ground disturbance, USB power/cable problem, or supply noise |
| USB serial disconnects/reconnects during a step | Ground reference, cable retention, EMI/noise, Jetson USB power, or Nano reset |
| Converter voltage falls or its indicator changes under load | Converter input sag, thermal protection, or current-limit operation |
| Wires, crimps, plugs, fuse holders, or converters heat | Excess resistance, undersized component, poor termination, or overload |
| One leg is wrong while its bank remains stable | Direction/calibration, horn/indexing, linkage, mechanical bind, or individual servo fault |

Do not continue merely because a lower gait command makes a reset disappear.
Find and correct the electrical or mechanical cause.

### Measurements to record

Use equipment and procedures rated for the expected battery voltage and servo
current. Never place a multimeter in current mode directly across a battery or
rail. Power off before moving connectors, changing meter configuration, or
altering wiring.

For **each of the two banks**, label and record:

1. Battery/power-bank voltage at its terminals at rest.
2. The same battery voltage during the same stand, single-leg, diagonal, and
   trot load event.
3. Converter input voltage at rest and during load.
4. Converter output voltage at rest and during load; compare against the
   designed servo-rail setpoint and the connected servo's rating.
5. Servo-bank voltage at PCA9685 `V+` during load.
6. Voltage at a far servo connector during the same event, so cable and
   connector drop is visible.
7. Bank current using a correctly rated clamp meter or inline monitor.
8. Peak/transient current if the instrument supports it, plus whether the
   converter's current-limit or protection indication activates.

Measure both converters and both servo banks under the same commanded test.
Compare left/right or assigned-bank values rather than accepting a single
healthy reading. A voltage that is correct at the converter but low at the
servo bank indicates wiring/connector/fuse drop; low converter output with
adequate input points toward converter limiting or regulation; low input and
output points upstream toward the battery or input wiring.

Also inspect:

- a deliberate common ground among Jetson, Arduino, PCA9685 logic, and servo
  supplies, without loose or high-resistance return paths;
- wire gauge, run length, strain relief, crimps, solder joints, connector
  current ratings, polarity, and contact heating;
- correctly rated, intact fuses and fuse holders—never bypass a fuse to make a
  test pass;
- both converter thermal states and ventilation;
- serial logs for disconnect/reconnect, firmware READY/PONG repetition,
  blocked frames, and counter resets;
- each servo for buzzing, stalled output, excess heat, damaged gears, a
  slipping horn, or mechanical-stop contact.

The TD-8130MG servos contain their own position controller. This ROS stack
cannot tune their internal PID or command motor torque through the current
protocol.

## Stop criteria and immediate response

Stop the current run on any incorrect direction or pair, collision, toe drag,
mechanical-stop approach, slipping horn, unstable stand, loss of support,
unexpected buzz, simultaneous slowing, voltage sag, current limiting, reset,
serial disconnect, smoke, smell, heat, or persistent diagnostic warning.

If ROS and serial are healthy:

1. Release motion input.
2. Issue the emergency-stop/disarm command or GUI STOP/HOLD.
3. Confirm zero velocity, no active swing, owner HOLD, and disarmed status.

If continued torque, a reset loop, or communication loss makes that response
uncertain, use the physical servo-power disconnect. Do not reach through the
leg workspace to save a software log.

## Rollback

Runtime rollback:

1. STOP and wait for any airborne foot to lower.
2. Confirm a settled, stopped status with four stance feet.
3. Load and Apply `REAL_DIAGNOSTIC` while stopped.
4. Confirm `/volt/status` echoes the applied profile and values.
5. If uncertain, HOLD, DISARM, disconnect servo power, and return to dry-run.

Configuration rollback:

1. Keep servo power disconnected.
2. Restore the complete known-good `config/gait_controller.yaml` and
   `config/real_robot_profiles.yaml` from the user's saved copy or chosen
   version-control revision.
3. Do not mix values from two revisions or restore servo calibration as part
   of a gait-only rollback.
4. Rebuild/relaunch and repeat the dry-run, stand, single-leg, pair,
   zero-stride, and `REAL_DIAGNOSTIC` gates.

The servo-budget validation re-runs whenever a restored configuration loads;
a file that fails it refuses to load and must be corrected, not forced.

## Remaining risks

- No current sensor confirms support load or foot contact.
- No joint encoder confirms servo position, velocity, backlash, or stall.
- No voltage/current telemetry confirms either power bank or converter.
- Commanded-FK stride and ground height are not physical measurements.
- Centre-of-mass location and diagonal support margin are not sensed.
- Surface friction, foot material, compliance, chassis flex, horn security,
  gearbox wear, servo temperature, battery state, and wiring temperature can
  change between tests.
- Firmware protocol 2 and a 30 Hz frame stream prove communication capability,
  not delivered motion or torque.
- HOLD/DISARM retain the last PWM target; DISABLE or power removal may cause
  collapse.
- Passing every support-stand test does not validate unsupported walking on
  the floor.

Record these limitations with every result. Do not report a simulation,
dry-run, command-FK metric, or unloaded support-stand run as physical
load-bearing validation.
