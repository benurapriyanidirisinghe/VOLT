# VOLT MoveIt2 Preparation

This directory is a placeholder for a future `volt_moveit_config` package. The
current repository keeps walking and emotes separate:

- Walking/trot: `volt_motion_controller.py` and the gait controller.
- Emotes: YAML trajectory playback or future MoveIt2 trajectory execution.

## Planned Planning Groups

Use these planning groups when generating a MoveIt2 config package:

- `all_legs`
  - all 12 joints in the standard VOLT joint order.
- `front_left_leg`
  - `front_left_shoulder`
  - `front_left_leg`
  - `front_left_foot`
- `front_right_leg`
  - `front_right_shoulder`
  - `front_right_leg`
  - `front_right_foot`
- `rear_left_leg`
  - `rear_left_shoulder`
  - `rear_left_leg`
  - `rear_left_foot`
- `rear_right_leg`
  - `rear_right_shoulder`
  - `rear_right_leg`
  - `rear_right_foot`

## RViz2 MotionPlanning Use

For emotes, use the RViz2 MoveIt MotionPlanning panel only while the robot is
idle:

1. Select `all_legs` for whole-body poses or an individual leg group for a
   small pose edit.
2. Plan to a nearby joint-space goal.
3. Preview the trajectory.
4. Execute only when `joint_trajectory_controller` is active, or save the
   trajectory to an emote YAML file for `volt_emote_player.py`.

Do not use MoveIt2 to command walking. Walking remains a gait-controller task.
