# VOLT Arduino Nano PCA9685 Firmware

This sketch drives 12 physical PCA9685 channels. It receives final physical
servo degrees from the ROS serial bridge; it does not convert ROS radians.

## Wiring

- Arduino Nano `A4` -> PCA9685 `SDA`
- Arduino Nano `A5` -> PCA9685 `SCL`
- Arduino Nano `5V` -> PCA9685 logic `VCC`
- Arduino Nano `GND` -> PCA9685 `GND`
- External servo supply `+` -> PCA9685 `V+`
- External servo supply `-` -> PCA9685 `GND`
- Jetson, Arduino, PCA9685, and servo supply grounds must be common.

Do not power the servos from the Arduino Nano.

## Arduino Library

Install this library in Arduino IDE:

- `Adafruit PWM Servo Driver Library`

## Serial Protocol

Baud rate: `115200`

The Jetson sends newline-terminated commands:

```text
FRAME d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
SERVO channel degrees
```

`FRAME` values are absolute physical servo degrees in PCA channel order.
The ROS serial bridge performs the named joint radians to physical degrees
conversion using `config/servo_calibration.yaml`.

Calibration commands:

```text
HOME
ARM
HOLD
DISARM
DISABLE
STATUS
PING
```

On power-up the firmware starts disarmed with PCA9685 output disabled and does
not move servos. `DISARM` and timeout hold the last target instead of moving to
center. `DISABLE` turns off PCA9685 pulses and may allow the robot to collapse.

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

Or start the motion controller and serial bridge together:

```bash
source install/setup.bash
ros2 launch volt_description hardware_control.launch.py serial_port:=/dev/ttyUSB0
```

For first hardware tests, leave the Arduino disarmed:

```bash
ros2 launch volt_description hardware_control.launch.py \
  serial_port:=/dev/ttyUSB0 \
  dry_run:=true \
  hardware_enabled:=false \
  auto_arm:=false
```

After confirming the physical center pose is correct, run with live commands:

```bash
ros2 launch volt_description hardware_control.launch.py \
  serial_port:=/dev/ttyUSB0 \
  dry_run:=false \
  hardware_enabled:=true \
  auto_arm:=true
```

If your Arduino appears as `/dev/ttyACM0`, change the `serial_port` or `port`
parameter accordingly.
