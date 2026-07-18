# VOLT Emotes

VOLT now has a separate emote playback path that keeps walking/trot control
separate from expressive poses.

Walking is still owned by:

- `scripts/volt_motion_controller.py`
- `scripts/volt_gait_controller.py`
- `scripts/volt_kinematics.py`

Emotes are one-shot joint trajectories played by:

- `scripts/volt_emote_player.py`

Both paths publish 12 joint positions to:

```bash
/joint_group_position_controller/commands
```

Do not run an emote while a walking command is active. The emote player
subscribes to `/cmd_vel` and `/volt/status`, publishes a stop request before
playback, and rejects or aborts playback if motion is still active.

## Joint Order

Every emote YAML file must use this exact order:

```yaml
joint_names:
  - front_left_shoulder
  - front_left_leg
  - front_left_foot
  - front_right_shoulder
  - front_right_leg
  - front_right_foot
  - rear_left_shoulder
  - rear_left_leg
  - rear_left_foot
  - rear_right_shoulder
  - rear_right_leg
  - rear_right_foot
```

## Emote Format

Create emotes in `src/volt_description/emotes`. Each file contains a name,
description, joint names, and timed points:

```yaml
name: example
description: Short description of the pose sequence.
joint_names:
  - front_left_shoulder
  - front_left_leg
  - front_left_foot
  - front_right_shoulder
  - front_right_leg
  - front_right_foot
  - rear_left_shoulder
  - rear_left_leg
  - rear_left_foot
  - rear_right_shoulder
  - rear_right_leg
  - rear_right_foot
points:
  - time_from_start: 0.0
    positions: [0.0496, 0.4992, -1.0812, -0.0496, 0.4992, -1.0812, 0.0496, 0.6956, -1.0812, -0.0496, 0.6956, -1.0812]
  - time_from_start: 1.0
    positions: [0.06, 0.7, -1.2, -0.06, 0.7, -1.2, 0.06, 0.8, -1.2, -0.06, 0.8, -1.2]
```

The player validates joint names, point times, point length, and joint limits
before publishing.

## Built-In Emotes

- `stand_ready`
- `sit`
- `bow`
- `look_left`
- `look_right`
- `small_dance`

## Play An Emote

Build and source the workspace:

```bash
cd ~/Documents/volt_ws
colcon build --packages-select volt_description
source install/setup.bash
```

Start a stack that owns the hardware bridge or `ros2_control` position
controller, but do not run another joint-command publisher at the same time.
For Gazebo, `gazebo.launch.py` is suitable because it starts the position
controller without `volt_motion_controller.py`. For hardware-only emote tests,
run `volt_serial_bridge.py` without `hardware_control.launch.py`.

Make sure the joystick or GUI is not publishing a walking command. Then run:

```bash
ros2 launch volt_description emote_player.launch.py emote:=bow
```

The player rejects playback if `/cmd_vel` or `/volt/status` indicates motion,
or if another node is already publishing to
`/joint_group_position_controller/commands`.

Slow down or speed up playback:

```bash
ros2 launch volt_description emote_player.launch.py emote:=small_dance speed_scale:=0.7
ros2 launch volt_description emote_player.launch.py emote:=look_left speed_scale:=1.5
```

Play a custom file:

```bash
ros2 launch volt_description emote_player.launch.py emote_file:=/absolute/path/my_emote.yaml
```

## Why Gait And Emotes Are Separate

Walking is a continuously balanced gait problem. The gait controller keeps feet
world-locked during stance, ramps velocity, clamps IK reach, and reacts to
`/cmd_vel`.

Emotes are timed joint-space trajectories. They should be played only while the
robot is idle, because they are expressive motions rather than locomotion
motions. Keeping these systems separate prevents RViz2 or a saved emote from
accidentally replacing the walking/trot controller.

## MoveIt2 Path Later

The new `config/moveit_controllers.yaml` maps MoveIt2 trajectory execution to
`joint_trajectory_controller/follow_joint_trajectory`. Later, an emote saver can:

1. Receive a `trajectory_msgs/JointTrajectory` from MoveIt2.
2. Validate the same 12-joint order and joint limits.
3. Convert each trajectory point into the YAML `points` format.
4. Save the YAML file into `emotes/`.
5. Play it with `volt_emote_player.py`, or execute it through the trajectory
   controller when that controller is active.

The current hardware bridge path remains `/joint_group_position_controller/commands`.
