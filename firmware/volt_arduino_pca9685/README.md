# VOLT Arduino Nano PCA9685 Firmware

This sketch drives 12 servos through a PCA9685 servo driver.

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
RAD r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11
```

`RAD` values are ROS joint angles in radians, in this order:

```text
front_left_shoulder front_left_leg front_left_foot
front_right_shoulder front_right_leg front_right_foot
rear_left_shoulder rear_left_leg rear_left_foot
rear_right_shoulder rear_right_leg rear_right_foot
```

The firmware maps `0.0` radians to the servo neutral position: `90` degrees.

Calibration commands:

```text
HOME
PING
DEG 90 90 90 90 90 90 90 90 90 90 90 90
```

## First Values To Tune

In `volt_arduino_pca9685.ino`:

- `SERVO_CHANNEL`: change if your wiring order is different.
- `SERVO_DIRECTION`: set a joint to `-1` if it moves opposite the simulation.
- `servoTrimDeg`: small offsets so physical zero matches the CAD/URDF zero.
- `SERVO_MIN_US` and `SERVO_MAX_US`: pulse limits for your servo model.
- `MAX_DEG_PER_SECOND`: lower this for gentler first hardware tests.

Start with the robot suspended or with legs off the ground.

## Jetson ROS 2 Bridge

The Jetson bridge node subscribes to:

```text
/joint_group_position_controller/commands
```

and sends `RAD ...` packets to the Arduino.

After building the workspace on the Jetson:

```bash
source install/setup.bash
ros2 run volt_description volt_serial_bridge.py --ros-args -p port:=/dev/ttyUSB0
```

Or start the motion controller and serial bridge together:

```bash
source install/setup.bash
ros2 launch volt_description hardware_control.launch.py serial_port:=/dev/ttyUSB0
```

If your Arduino appears as `/dev/ttyACM0`, change the `serial_port` or `port`
parameter accordingly.
