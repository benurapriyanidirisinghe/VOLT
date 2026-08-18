# VOLT Emotes

The control GUI's Robot Emotes panel uses a nonblocking, Cartesian emote
engine inside `volt_motion_controller.py`. Emotes therefore use the same body
targets, foot targets, inverse kinematics, joint conditioning, command router,
and serial calibration path as ordinary motion. The GUI does not publish joint
trajectories and does not block while an emote runs.

> Emote status is commanded-state telemetry. The TD-8130MG/PCA9685 hardware
> path has no servo-position or foot-contact feedback, so a displayed foot
> position, planted foot, completed keyframe, or return to stand is a
> controller command/kinematic estimate—not a physical measurement.

## GUI catalog

The ten GUI emotes are defined in `config/cartesian_emotes.yaml`:

| GUI action | Request name | Motion |
| --- | --- | --- |
| Push-ups | `push_ups` | Lowers and raises the body 20 mm by default with all four commanded feet planted. |
| Body roll | `body_roll` | Rolls center/left/right through ±4.5 degrees. |
| Nod | `nod` | Pitches gently through +5 and -4 degrees. |
| Wave left | `wave_left` | Shifts right, lifts and waves the front-left foot, plants it, then recenters. |
| Wave right | `wave_right` | Mirrored front-right wave after a leftward body shift. |
| Heart | `heart` | Uses a 12 mm rearward, 10 mm lower crouch, traces mirrored halves one front foot at a time, and keeps three commanded support feet throughout. |
| Bow | `bow` | Keeps the feet fixed while lowering 10 mm, shifting rearward 6 mm, and pitching 6 degrees. |
| Stretch | `stretch` | A play-bow with both front commanded feet 25 mm forward and a small lowered/pitched body target. |
| Happy dance | `happy_dance` | Small alternating front-foot lifts with a ±3.5-degree body roll. |
| Shake no | `shake_no` | A standing head/body-yaw gesture through ±5.5 degrees. |

The same catalog also contains conservative planted-body-yaw `look_left` and
`look_right` entries for compatibility with older command-line names. They are
not extra GUI buttons.

Every catalog transition uses smootherstep interpolation. The GUI also shows
`SIT` and `STAND UP` beside the emotes. Those actions use a separate
four-foot-planted Cartesian transition: stable stand, rearward shift,
asymmetric rear-leg bend/lower with the front legs supporting, settle, and the
captured path in reverse for Stand Up. The default endpoint is 145 mm body
height, 20 mm rearward shift, and -10 degrees pitch; the complete path is
densely preflighted through IK with joint/workspace margin. They are not extra
emote catalog entries. Repetitions, speed, amplitude, depth, correlated emote
cancel, and emote progress do not apply to those pose actions; use their
ordinary action/STOP state handling.

The GUI options are repetitions 1–5, speed 0.5–2.0x, amplitude 0.5–1.5x, and
depth 0.5–1.5x. Amplitude scales body X/Y/angles and foot XYZ offsets; depth
scales the body-height offset. Push-ups also have a dedicated **Push-up
travel** control: 10–25 mm in 1 mm steps, with a 20 mm default. The GUI maps
that distance to the same validated depth field, so it does not introduce a
second motion path. At 25 mm, the catalog target reaches the conservative
175 mm minimum body height from a 200 mm stand. A lower captured stance may
make the controller reject a large setting during its composed-trajectory
preflight. Begin at the 20 mm default on a rigid support stand. A larger value
is not evidence that the physical robot can safely reach or support the
command.

## Automatic facial expressions

Face behavior is configured independently of joint trajectories in
`config/face_expressions.yaml`. The GUI observes the controller's correlated
emote state and publishes one expression change at start, then restores the
previous saved manual face after completion, cancellation, or the smooth
return-to-stand path. The firmware animates the face locally; no animation
frames are streamed from ROS.

The shipped mappings match the current Cartesian catalog:

| Robot behavior | Face expression |
| --- | --- |
| Push-ups | `angry` red pulse |
| Body roll | `playful` rainbow |
| Wave left/right | `happy` yellow pulse |
| Heart | `love` pink/red pulse |
| Bow | `happy` warm yellow |
| Happy dance / dance alias | `excited` cyan/magenta chase |
| Nod | `neutral` |
| Shake/no | `confused` purple/yellow |
| Stretch | `happy` |
| Look left/right | `thinking` purple loading |
| Sleep/lie-down aliases | `sleeping` blue breathing |
| Wake/wake-up aliases | `success`, then the saved manual face |

Sit uses `neutral`; standing up uses `success` until the controller reaches
standing; walking restores `idle` unless the operator locks a manual
expression; calibration ownership uses `thinking`. The GUI's automatic toggle
disables non-safety changes. Its lock also blocks emote/walking changes, but an
emergency or critical fault still selects `error`, and reported low voltage
selects `alert`. The present stack has no battery-voltage sensor, so low-voltage
override requires another node or firmware revision to report `low_voltage`
or `undervoltage` in status.

To add or tune a face expression:

1. Add a lower-case identifier beneath `expressions` in
   `config/face_expressions.yaml`, with RGB, a supported firmware effect,
   brightness `0..255`, and speed `10..60000` ms. Add `alternate_color` for
   a two-color effect; when omitted, color B mirrors the primary RGB. Quote
   YAML tokens such as `"off"` that YAML 1.1 otherwise treats as booleans.
2. Add the emote/action/state alias under the relevant `automatic_mappings`
   section. The target must name a defined expression.
3. Restart the GUI so the strictly validated catalog is reloaded, then verify
   the preset in dry-run and with **TEST LEDS** before live use.

Face commands use `/volt/face/expression` (`std_msgs/String`),
`/volt/face/color` (`std_msgs/ColorRGBA`, normalized RGB),
`/volt/face/alternate_color` (`std_msgs/ColorRGBA`, normalized secondary RGB),
`/volt/face/brightness` (`std_msgs/UInt8`), `/volt/face/effect`
(`std_msgs/String`), and `/volt/face/speed` (`std_msgs/UInt32`, milliseconds).
The serial bridge deduplicates requests and resynchronizes the desired snapshot
after reconnect without changing the emote controller's joint ownership or
keepalive behavior.

## ROS request contract

`/volt/emote` is `std_msgs/msg/String` containing strict JSON. A start request
contains the complete option set and a caller-generated correlation ID:

```json
{"command":"start","request_id":"gui-1234","name":"push_ups","repetitions":1,"speed":1.0,"amplitude":1.0,"depth":1.0}
```

A cancel request must repeat the active or queued request ID:

```json
{"command":"cancel","request_id":"gui-1234"}
```

A client must renew the same queued or active request at least once within the
default 750 ms lease. The GUI and compatibility client publish every 200 ms:

```json
{"command":"keepalive","request_id":"gui-1234"}
```

Cancel and keepalive requests contain only `command` and `request_id`.
Request IDs are 1–64 characters. Unknown commands, unknown fields, invalid
option bounds, unknown catalog names, and stale/mismatched control IDs are
rejected or ignored without disturbing an authoritative active request. The
controller reports correlation and progress in `/volt/status`
with `emotes_available`, `emote_active`, `emote_pending`, `emote_name`,
`emote_state`, `emote_progress`, `emote_returning`, `emote_settling`,
`emote_keepalive_age`, `emote_keepalive_timeout`, `emote_request_id`,
`emote_result`, `emote_message`, and the active options. Clients must accept
a result only when `emote_request_id` matches the request they sent.

For a direct ROS test, keep the robot supported, obtain `MOTION` ownership,
stand and stop first, then use the compatibility client, which renews the
lease and waits for the correlated terminal result:

```bash
ros2 run volt_description volt_emote_player.py --ros-args \
  -p emote:=push_ups -p repetitions:=1 -p speed_scale:=1.0 \
  -p amplitude:=1.0 -p depth:=1.0
```

Do not use a lone `ros2 topic pub --once` start as a playback client: without
keepalives, the controller cancels it after 750 ms. The GUI is preferred
because it renews the lease, correlates the result, gates controls, and shows
the controller's rejection reason.

## Safety, ownership, and STOP

An integrated emote is accepted only while the controller is standing, ROS
command ownership is `MOTION`, no pose transition or hardware diagnostic is
active, and its request is valid. If locomotion is merely settling, the
controller queues the request, commands a stop, and starts only after velocity,
gait, and feet are settled. It captures the actual *commanded* standing body
and feet as the base and preflights the complete composed trajectory through
IK; any projected foot target or clamped joint rejects the request before
motion begins. Runtime IK guards remain active.

While an emote owns the Cartesian targets:

- gait selection and manual body-pose changes are rejected;
- a nonzero velocity request cancels the emote;
- loss of safe command ownership forces cancellation/HOLD handling;
- a second emote is rejected; and
- loss of its keepalive lease cancels queued motion or starts a smooth return;
- post-filter settling remains leased until the conditioned command reaches
  the captured stand; and
- the normal timer continues publishing conditioned 12-joint commands through
  the single command router.

An ownership loss is emergency-immediate: the controller discards the emote
and the router holds its last valid command, which can freeze a lifted foot.
It does not promise the smooth return used by an ordinary STOP/cancel. For a
planned halt, use STOP first, wait for the commanded return, and only then use
HOLD/DISARM.

`STOP EMOTE` sends a correlated cancel. The ordinary GUI `STOP` action also
cancels an emote. A queued emote cancels immediately; an active emote takes a
smooth, approximately one-second return path to its captured commanded stand.
Wait until status reports the return complete before selecting a gait,
sitting, holding, or disarming. Sit is available only from a settled stand;
ARM from sitting is forbidden, so Stand and settle before re-arming. If
continued physical movement is unsafe, use
the servo-power disconnect—ROS STOP is not a hard emergency stop.

## Recommended physical check

Run every emote in simulation and hardware-disabled dry-run first. For the
first live check, secure VOLT above the floor, keep power isolation within
reach, explicitly ARM through the guided workflow, select `STAND`, wait for a
stopped standing status, and use one default-speed/default-amplitude push-up.
Stop and inspect after it returns. Waves, heart, stretch, and happy dance move
one or both commanded support feet and are more balance-sensitive; they come
only after the four-leg motions pass. Never auto-arm.

## Compatibility command-line client

`volt_emote_player.py` and `emote_player.launch.py` remain as finite
compatibility clients, but they no longer load or publish joint-space YAML.
They require the normal motion controller/router stack, publish zero velocity
and STOP, wait for fresh idle status, translate the requested name, send the
same correlated `/volt/emote` JSON used by the GUI, and wait for the matching
terminal result. An interruption sends correlated cancel plus STOP. The motion
controller remains the only joint-command source.

Legacy aliases are intentionally narrow:

- `stand_ready` becomes the existing `stand` action;
- `sit` remains the existing `sit` action;
- `small_dance` becomes Cartesian `happy_dance`;
- `bow` uses the Cartesian catalog `bow`; and
- `look_left`/`look_right` use the conservative planted Cartesian body-yaw
  catalog entries.

For example, with the normal controller already standing under explicit
`MOTION` ownership:

```bash
ros2 launch volt_description emote_player.launch.py \
  emote:=bow repetitions:=1 speed_scale:=1.0 amplitude:=1.0 depth:=1.0
```

The compatibility options are clamped to the same public bounds as the GUI.
`emote_file` is deprecated and any non-empty custom path is rejected. Old
files under `emotes/` are retained as historical data only and are not a live
playback route. Add new behavior to `config/cartesian_emotes.yaml` so it gets
strict catalog validation, sampled IK preflight, conditioning, correlated
status, and smooth STOP behavior.

MoveIt2's `joint_trajectory_controller/follow_joint_trajectory` mapping is a
future trajectory-import path; it does not change the current integrated
Cartesian safety contract.
