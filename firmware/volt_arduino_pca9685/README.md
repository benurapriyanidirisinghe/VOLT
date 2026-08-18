# VOLT Arduino Nano PCA9685 Firmware

This sketch drives 12 physical PCA9685 channels and two mirrored WS2812B face
strips. It receives final physical servo degrees from the ROS serial bridge; it
does not convert ROS radians. Face animations run on the Nano with a
non-blocking `millis()` state machine, so the host sends settings rather than
animation frames.

## Wiring

| From | To | Note |
| --- | --- | --- |
| Arduino Nano `A4` | PCA9685 `SDA` | Servo I2C data; unchanged |
| Arduino Nano `A5` | PCA9685 `SCL` | Servo I2C clock; unchanged |
| Arduino Nano `5V` | PCA9685 logic `VCC` | Logic only |
| External servo supply `+` | PCA9685 `V+` | Never power servos from the Nano |
| Nano `D6` | **Both** strip `DIN` pins | DIN wires are connected in parallel, optionally through a 330-470 ohm data resistor |
| External regulated LED supply `+5 V` | Both strip `5V` pins | Size the supply and wiring for all 16 physical LEDs |
| Common ground | Nano `GND`, PCA9685 `GND`, Jetson/controller ground, both external-supply negatives, and both strip `GND` pins | Required before applying data or power |

Do not power the servos or the two LED strips from the Arduino Nano. The LED
supply must be regulated 5 V. All grounds must be common; a separate LED
supply without the common-ground connection can cause unreliable data or
damage. A bulk capacitor across the LED supply near the strips is recommended
by the strip manufacturer.

`NUM_FACE_LEDS` is intentionally `8`, although there are 16 physical LEDs.
Both strips receive the same D6 bitstream at the same time, so logical pixel 0
lights physical pixel 0 on both strips, and so on. They are parallel mirrored
outputs, not one 16-pixel daisy chain. If the two strips display different
patterns, check their common data connection, pixel direction, power, and
ground; software cannot address the strips separately.

## Arduino Library

Install these libraries in Arduino IDE 1.8 with **Sketch > Include Library >
Manage Libraries**:

- `Adafruit PWM Servo Driver Library`
- `Adafruit NeoPixel` by Adafruit (tested with 1.15.5)

With Arduino CLI, the equivalent commands are:

```bash
arduino-cli lib install "Adafruit PWM Servo Driver Library"
arduino-cli lib install "Adafruit NeoPixel"
```

This workspace currently has the Arduino IDE 1.8.19 command-line frontend, so
the exact NeoPixel installation command available here is:

```bash
arduino --install-library 'Adafruit NeoPixel' --save-prefs
```

The current Nano/old-bootloader verification command is:

```bash
arduino --verify --board arduino:avr:nano:cpu=atmega328old \
  firmware/volt_arduino_pca9685/volt_arduino_pca9685.ino
```

After confirming the port, upload with:

```bash
arduino --upload --board arduino:avr:nano:cpu=atmega328old \
  --port /dev/ttyUSB0 \
  firmware/volt_arduino_pca9685/volt_arduino_pca9685.ino
```

Use `/dev/ttyACM0` if that is where the Nano appears. Some Nano boards use the
new bootloader; choose the processor matching the actual board if the old
bootloader upload fails.

## Serial Protocol

Baud rate: `57600`

The face-enabled Nano firmware and every ROS bridge launch must use this same
rate. `Adafruit_NeoPixel::show()` briefly masks AVR interrupts; 57600 keeps the
8-pixel transfer within the UART hardware receive margin while the 30 Hz compact
FRAME stream still uses well under half the available bandwidth. After changing
between firmware revisions, flash this sketch and rebuild/restart the ROS stack
together; do not fall back to 115200 for this face-enabled sketch.

The Jetson sends newline-terminated commands:

```text
FRAME d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
SERVO channel degrees
```

`FRAME` values are absolute physical servo degrees in PCA channel order.
The ROS serial bridge performs the named joint radians to physical degrees
conversion using `config/servo_calibration.yaml`. The bridge transmits compact
whole-degree values so every complete frame, including its newline, fits inside
the Nano's 63-byte usable serial receive buffer.

Calibration commands:

```text
ARM
HOLD
DISARM
DISABLE
STATUS
PING
```

Face commands are also newline terminated:

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
FACE <expression_name>
HOST SYNC
```

Command and preset names are case-sensitive and lowercase where shown.
Integer RGB values, requested brightness, speed, and pixel index are clamped to
safe ranges rather than wrapped: RGB/brightness `0..255`, speed
`10..60000 ms`, and pixel index `0..7`. Missing or non-integer fields, extra
fields, unknown effects, and unknown expressions return a readable `ERR ...`
line and do not change the active face. The input line parser also safely
discards oversized lines through the next newline.

`LED COLOR` changes the primary color. When it follows `FACE`, it preserves the
preset identity and its secondary color, so the two-color excited, alert, and
confused patterns remain intact. `LED COLOR_B` independently tunes the
secondary color used by effects such as `alternate` and `chase`; it preserves
the active expression/effect and does not turn an off face on. Both commands
clamp each RGB component to `0..255`. Send `LED EFFECT solid` as well when a
solid result is required. The public `pulse` effect preserves love's internal
heartbeat, and public `blink` preserves the success/error staged sequences when
those presets are active. `LED PIXEL` enters explicit custom per-pixel mode;
the first pixel command after another effect clears the logical buffer, while
subsequent pixel commands accumulate. `LED CLEAR` clears that eight-pixel buffer.
`LED OFF` disables face output immediately. `LED STATUS` returns the requested
and effective brightness, brightness limit, colors, speed, active effect, and
expression. Typical acknowledgements are:

```text
OK LED COLOR 255 0 0
OK LED COLOR_B 255 0 180
OK LED BRIGHTNESS 80 EFFECTIVE=80
OK LED EFFECT breathe
OK LED SPEED 3000
OK LED PIXEL 0 255 0 0
OK LED CLEAR
OK LED OFF
OK FACE happy
```

`READY`, `PONG`, and the full motion `STATUS` preserve protocol version 2 and
now advertise `FACE_SUPPORTED=1 LED_COUNT=8`. `STATUS` and `LED STATUS` expose
these face fields:

```text
LED_ENABLED= LED_COLOR= LED_COLOR_B= LED_BRIGHTNESS=
LED_EFFECTIVE_BRIGHTNESS= LED_LIMIT= LED_EFFECT= LED_SPEED_MS= FACE=
```

### Host visual synchronization

A fresh boot advertises these fields in `READY`, `PONG`, `STATUS`, and
`LED STATUS` responses:

```text
HOST_SYNC_REQUIRED=1 HOST_PING=0 HOST_SNAPSHOT=0 HOST_SYNCED=0
```

The cyan startup/loading animation continues non-blockingly while the bridge
re-establishes its desired face snapshot. A valid `PING` sets `HOST_PING=1`.
Each successfully applied mutating `FACE` or `LED` setting sets
`HOST_SNAPSHOT=1` and clears `HOST_SYNCED`; `STATUS`, `LED STATUS`, and malformed
commands do not count as mutations. The bridge sends this terminal command only
after every desired face-setting acknowledgement has arrived:

```text
HOST SYNC
```

It receives exactly:

```text
OK HOST SYNC HOST_SYNCED=1
```

`HOST SYNC` is safe and idempotent once its prerequisites have been met. Before
the handshake it returns `ERR HOST PING_REQUIRED`; before any valid visual
snapshot it returns `ERR HOST SNAPSHOT_REQUIRED`. If the terminal command
arrives while the loading effect is still active, firmware selects idle as a
safe fallback. Normally the first `FACE` command has already replaced loading
with the host's requested expression.

This is a visual/connection synchronization indicator, **not** a new motion
safety dependency. `ARM` retains the existing protocol-2 behavior so headless
tools and manual serial recovery remain backward-compatible. A legacy host can
still operate motion and apply face commands, but status continues to show
`HOST_SYNCED=0` until it sends the terminal command.

The stored brightness request may be as high as 255, but every rendered effect
passes through `FACE_BRIGHTNESS_LIMIT` (160 by default), so no animation or
preset can bypass the global current limit. The default requested brightness
is 80.

## Face Expressions

| Expression | RGB A | RGB B | Firmware animation |
| --- | --- | --- | --- |
| `neutral` | 80, 180, 255 | same | very slow breathe |
| `idle` | 0, 120, 255 | same | slow breathe |
| `happy` | 255, 180, 20 | same | gentle pulse |
| `excited` | 0, 255, 255 | 255, 0, 180 | fast alternate |
| `love` | 255, 20, 80 | same | heartbeat double-pulse |
| `sad` | 20, 50, 180 | same | slow breathe |
| `angry` | 255, 0, 0 | same | scanner |
| `alert` | 255, 0, 0 | 255, 90, 0 | fast alternate |
| `thinking` | 150, 40, 255 | same | rotating/loading |
| `confused` | 170, 40, 255 | 255, 180, 0 | alternate |
| `sleeping` | 0, 20, 80 | same | very slow, dim-blue breathe |
| `success` | 0, 255, 80 | same | two blinks, then solid |
| `error` | 255, 0, 0 | same | three flashes, then dim solid |
| `scared` | 180, 220, 255 | same | rapid blink |
| `playful` | rainbow | rainbow | rainbow chase |
| `shutdown` | current color | - | smooth fade to off |

At the first sketch instructions after reset (and the Nano bootloader), the
firmware clears the strip and keeps it dark while initializing. Once
servo/PCA9685 initialization succeeds and readiness is announced, it runs a
cyan loading chase until the host face snapshot replaces it and `HOST SYNC`
confirms completion. Loading never delays serial parsing or servo updates.
`DISARM` normally leaves the face active.
Set `FACE_OFF_ON_DISARM` to `true` if a particular installation must turn it
off on `DISARM` and `DISABLE`.

On power-up the firmware starts disarmed with PCA9685 output disabled and does
not move servos. Opening the serial port is not treated as readiness: the bridge
waits for `OK VOLT_PCA9685_READY ...` or an `OK PONG`, and it treats ARM as
pending until `OK ARM` (or `STATUS ARMED=1`) is received. This firmware reports
the same explicit capability fields in `READY`, `PONG`, and `STATUS`:

```text
FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0
```

`FW` identifies the expected sketch, `PROTO` versions the motion-safety
contract, and `MAX_DPS` exposes the firmware's final per-channel slew ceiling.
The normal bridge mode defaults to `required_protocol_version:=2`; an old
generic `OK PONG` can still be parsed as a serial response, but it cannot unlock
ARM or live `FRAME`/`SERVO` output. Reflash this sketch if the bridge status
shows `firmware_compatible=0`.

`HOLD`, `DISARM`, and command timeout clear the firmware's armed state and
reject every new `FRAME`/`SERVO` command while maintaining the last PCA9685
pulses. A fresh `ARM` is required to resume motion. `DISABLE` also turns off
PCA9685 pulses and may allow the robot to collapse.

The bridge mirrors this state machine: it blocks outgoing frames before a stop
acknowledgement and never streams unless readiness and armed state have both
been confirmed by firmware. In normal motion mode, both manual and automatic
ARM requests also require a recent `/volt/status` report certifying the exact
stopped calibrated open-loop `WALK_POSE`. Sitting never qualifies. A
stale or missing report keeps ARM locked. It also rejects duplicate publishers
on the router output, motion-joint output, and controller command topics. More
than one `/cmd_vel` publisher is permitted because GUI and diagnostic velocity
sources are arbitrated upstream rather than being direct actuator authority.

The supported suspended calibration and joint-test launches explicitly set
`require_motion_safe_to_arm:=false`; that specialized mode also bypasses the
version gate so legacy firmware can be identified or stopped without enabling
normal robot motion. `HOLD`, `DISARM`, and `DISABLE` remain available even when
firmware compatibility fails.

## First Values To Tune

In `src/volt_description/config/servo_calibration.yaml`:

- `pca_channel`: which physical PCA9685 output controls each joint.
- `direction`: set to `-1` if the real link moves opposite Gazebo.
- `neutral_deg`: servo command that places the real joint at URDF zero.
- `trim_deg`: small final offset around neutral.
- `min_deg` and `max_deg`: conservative physical limits.
- `min_pulse_us` and `max_pulse_us`: pulse limits for your servo model.

In `volt_arduino_pca9685.ino`:

- `MAX_DEG_PER_SECOND`: lower this for gentler first hardware tests.
- `CHANNEL_SAFE_START_DEG`: channel-ordered calibrated standing angles used as
  the slew origin when PWM is first enabled. Regenerate these values whenever
  standing centers or trims change.
- `LED_PIN`: the shared face-data pin; the default is Nano digital pin D6.
- `DEFAULT_FACE_BRIGHTNESS`: requested brightness after reset; default 80.
- `FACE_BRIGHTNESS_LIMIT`: hard global ceiling applied to every rendered
  pixel; default 160.
- `FACE_OFF_ON_DISARM`: whether `DISARM`/`DISABLE` also turn off the face;
  default `false`.
- `FACE_FRAME_PERIOD_MS`: minimum interval between animated strip writes;
  default 25 ms (at most 40 face frames/s).

To add a firmware expression, add its value to `FaceExpression`, recognize its
lowercase serial name in `parseFaceExpression()`, print the same name in
`printFaceExpressionName()`, and add its colors/effect/speed to
`applyFacePreset()`. Also add the name to the host face-expression
configuration so the GUI and emote mapping can select it. Keep animations in
`updateFaceLeds()` non-blocking; do not add animation loops or `delay()` calls.

The current repository firmware source sets `MAX_DEG_PER_SECOND` to
`120.0f`. That ceiling has not been validated on the physical robot and must
not be raised. The ROS host keeps default and non-fast motion capped at
30 deg/s. The physical `fast_trot` path has a separate 110 deg/s host cap,
which remains below the firmware ceiling.

If the Nano still runs an older image with a 30 deg/s firmware ceiling, reflash
this current firmware before trying the new physical fast-trot profile. If the
Nano already runs the 120 deg/s image, host-side Python or tuning changes alone
do not require another upload. Servo calibration values are unchanged by this
profile. The firmware remains the final safety guard, and calibration tools can
bypass the gait host cap, so use them cautiously.

The ROS bridge clamps `max_send_rate` to at most 30 Hz and schedules frames
against the next ideal deadline, so a 100 Hz source produces 30 Hz rather than
aliasing down to 25 Hz. The firmware updates the 50 Hz PCA9685 output over
400 kHz I2C and skips channels whose commanded angle has stopped changing.

See the
[fast-trot safety and tuning guide](../../src/volt_description/FAST_TROT.md)
before attempting physical fast-trot operation.

Start with the robot suspended or with legs off the ground.

## Jetson ROS 2 Bridge

The Jetson bridge node subscribes to:

```text
/joint_command_router/output
```

and sends `FRAME ...` packets to the Arduino. The command router is the only
node that publishes to `/joint_group_position_controller/commands`.

After building the workspace on the Jetson:

```bash
source install/setup.bash
ros2 run volt_description volt_serial_bridge.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p dry_run:=true \
  -p hardware_enabled:=false
```

For the normal simulator-linked physical test, start Ignition, the controller,
the reorganized GUI, and the live bridge together. Arming remains manual:

```bash
source install/setup.bash
ros2 launch volt_description volt_start.launch.py \
  start_serial_bridge:=true \
  serial_port:=/dev/ttyUSB0 \
  use_hardware:=true dry_run:=false auto_arm:=false
```

For the first supported test, press STAND and wait for the transition, press
STOP, then verify the Arduino panel says `CONNECTED - DISARMED`. The ARM button
stays locked until the firmware handshake and the exact stopped calibrated
`WALK_POSE` are both confirmed. After a completed SIT transition, use Stand Up
while already armed; if disarmed while sitting, Stand while unarmed, settle at
`WALK_POSE`, and only then ARM.

`hardware_control.launch.py` uses wall time and an explicit open-loop canonical
`WALK_POSE` seed because these hobby servos do not report `/joint_states`. That
seed exactly matches `CHANNEL_SAFE_START_DEG`, but it is still an assumption:
before first PWM/ARM, verify the supported robot and servo shafts correspond
to the calibrated standing pose. Set `use_sim_time:=true` only when an external
integration intentionally publishes `/clock`.

For a bridge-only dry run that never opens the serial port:

```bash
ros2 run volt_description volt_serial_bridge.py --ros-args \
  -p port:=/dev/ttyUSB0 -p dry_run:=true -p hardware_enabled:=false
```

If your Arduino appears as `/dev/ttyACM0`, change the `serial_port` or `port`
parameter accordingly.

## Raised-Robot Hardware Test

Perform the first tests with the robot firmly supported so every foot is off
the ground. Keep the servo supply disconnected for LED-only checks, or leave
the firmware disarmed. Use a serial terminal at 57600 baud with **newline**
line endings. Opening the port resets many Nano boards, so wait for
`OK VOLT_PCA9685_READY ... FACE_SUPPORTED=1` before sending commands.

1. Verify reset and capability behavior. The strips must clear immediately and
   show a continuing cyan loading pattern. Confirm READY reports
   `HOST_SYNCED=0`. Send `PING`, `STATUS`, and `LED STATUS`; confirm protocol 2,
   eight logical pixels, `HOST_PING=1`, requested brightness 80, and effective
   brightness no higher than the configured limit. Before a visual setting,
   `HOST SYNC` must return `ERR HOST SNAPSHOT_REQUIRED`. Send `FACE idle`, then
   `HOST SYNC`; confirm the exact success acknowledgement and `HOST_SYNCED=1`.
2. Start at low current with `LED BRIGHTNESS 20`. For solid-color tests, send
   each pair in order: `LED COLOR 255 0 0` then `LED EFFECT solid`; repeat with
   `0 255 0`, `0 0 255`, and `255 255 255`. Confirm both physical strips match.
3. Test all eight logical positions. Before each position, send `LED CLEAR`,
   then `LED PIXEL <index> 255 255 255` for indexes 0 through 7. Exactly the
   corresponding LED on **both** strips should light. A mismatch is a wiring,
   orientation, power, or strip fault because both strips receive identical
   firmware data.
4. Test brightness with solid white at requested values 0, 20, 80, 160, and
   255. Use `LED STATUS` to confirm that 255 is accepted as the requested value
   but effective output remains capped at `FACE_BRIGHTNESS_LIMIT`. Return to a
   safe low value before continuing.
5. Set `LED COLOR 80 180 255` and `LED COLOR_B 255 0 180`, then verify
   `LED EFFECT alternate` uses both colors and `LED STATUS` reports the new
   `LED_COLOR_B`. Exercise every public effect with
   `LED EFFECT <name>`: `solid`, `breathe`, `blink`, `pulse`, `rainbow`,
   `chase`, `scanner`, `sparkle`, `alternate`, `loading`, and `off`. Repeat an
   animated effect after `LED SPEED 10`, `LED SPEED 1000`, and
   `LED SPEED 60000` to check clamping and responsiveness.
6. Send every preset: `FACE neutral`, `idle`, `happy`, `excited`, `love`,
   `sad`, `angry`, `alert`, `thinking`, `confused`, `sleeping`, `success`,
   `error`, `scared`, `playful`, and `shutdown`. Confirm success becomes solid
   green after two blinks, error becomes dim red after three flashes, and
   shutdown fades fully off.
7. Send invalid lines such as `LED COLOR red 0 0`, `LED EFFECT unknown`,
   `LED PIXEL nope 1 2 3`, `FACE unknown`, commands with missing/extra fields,
   and a line longer than 191 characters. Confirm an `ERR ...` response, no
   reset, and no unexpected face change. Also confirm out-of-range numeric
   inputs clamp safely with `LED STATUS`.
8. Rapidly alternate valid FACE, color, effect, brightness, and speed commands
   for at least 30 seconds. Confirm prompt acknowledgements and continued
   animation without a reset or serial disconnect.
9. For the simultaneous motion test, close the raw serial terminal so only the
   ROS bridge owns the port. Keep the robot raised, follow the normal calibrated
   stand/STOP/ARM procedure above, stream a conservative gait, and change face
   presets/effects from the GUI. Confirm servo motion remains smooth and the
   bridge reports no timeouts. Repeat GUI disconnect/reconnect and confirm the
   selected face is restored once firmware readiness returns.
10. Verify integration behavior: run mapped emotes, face-lock restoration, and
    emergency/fault override from the GUI. Confirm the face returns to idle or
    the prior manual selection after each emote, while safety expressions still
    override a manual lock. Do not test walking on the floor until all raised
    servo and LED checks pass.
