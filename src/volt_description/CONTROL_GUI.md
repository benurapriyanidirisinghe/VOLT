# VOLT motion control

The controller uses the dimensions and joint limits in `volt.urdf.xacro`.
It provides smooth stand/sit transitions, static walk, amble, diagonal trot,
forward/reverse motion, lateral motion, turning, step-in-place, and body-pose
adjustment.

## Start everything

```bash
cd ~/Documents/volt_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select volt_description --symlink-install
source install/setup.bash
ros2 launch volt_description volt_start.launch.py
```

Wait until the GUI says `HOLD` and `connected controller`, then:

1. Click **STAND** and wait for `STANDING`.
2. Select **WALK** for the first test.
3. Set speed to 25-35%.
4. Move the joystick forward briefly and release it.
5. Increase speed only after the robot remains stable.

Normal steering maps joystick left/right to turning. Crab/omnidirectional mode
maps joystick left/right to lateral movement; use Yaw trim to turn at the same
time. Releasing the joystick sends zero velocity. The controller also stops
motion if command messages disappear for 0.6 seconds.

## Run without the combined launcher

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/volt_ws/install/setup.bash
ros2 launch volt_description ignition.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/volt_ws/install/setup.bash
ros2 launch volt_description control.launch.py
```

The standalone pose commands require the motion controller to be running:

```bash
ros2 run volt_description stand_up.py
ros2 run volt_description sit_pose.py
```

## Command topics

- `/cmd_vel` (`geometry_msgs/Twist`): x/y translation and yaw velocity
- `/volt/gait` (`std_msgs/String`): `walk`, `amble`, or `trot`
- `/volt/action` (`std_msgs/String`): `stand`, `sit`, `stop`, or `step`
- `/volt/body_pose` (`geometry_msgs/Twist`): translation/height and roll/pitch/yaw
- `/volt/status` (`std_msgs/String`): JSON controller state and tracking status
