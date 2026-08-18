# VOLT Standalone Forward Walk

This Arduino Nano sketch moves the calibrated VOLT robot without ROS. It
replays direct PCA9685 servo angles derived from the repository's conservative
`VOLT WALK` trajectory at a 0.004 m/s forward command.

It is intentionally separate from `volt_arduino_pca9685`. Flashing this sketch
replaces the ROS protocol firmware; reflash the production sketch before using
ROS again.

## Behavior

- Power-up leaves all PCA9685 outputs disabled.
- `ARM` enables only the calibrated standing angles.
- `RUN` executes one finite forward cycle.
- `RUN 2` and `RUN 3` execute at most two or three cycles.
- Every run returns slowly to the calibrated standing pose.
- Table interpolation is limited to 18 degrees/second.
- `STOP` finishes the current all-feet-down cycle before returning.
- `HOLD`/`DISARM` stop immediately and retain the current pulses.
- `DISABLE` removes pulses and can make the robot collapse.

There is no contact, encoder, current, or balance feedback. The trajectory is
open-loop and cannot verify that a foot touched the floor. Perform the first
test with the body rigidly supported and every leg clear.

## Direct angle order

Every embedded row is in PCA9685 channel order and stores degrees multiplied
by ten:

| Channel | Joint |
|---:|---|
| 0 | front-right shoulder |
| 1 | front-left leg |
| 2 | front-left foot |
| 3 | front-left shoulder |
| 4 | front-right leg |
| 5 | front-right foot |
| 6 | rear-right shoulder |
| 7 | rear-left leg |
| 8 | rear-left foot |
| 9 | rear-left shoulder |
| 10 | rear-right leg |
| 11 | rear-right foot |

Directions, trims, and calibration are already applied. Do not invert any
embedded angle again.

## Compile

```bash
/usr/bin/arduino --verify \
  --board arduino:avr:nano:cpu=atmega328 \
  /home/ros2/Documents/volt_ws/firmware/volt_standalone_forward_walk/volt_standalone_forward_walk.ino
```

The required Arduino libraries are:

- Adafruit PWM Servo Driver
- Adafruit BusIO

## Upload

Stop every ROS launch and disconnect servo power before flashing. Confirm that
the serial device is not open:

```bash
lsof /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

Then upload:

```bash
/usr/bin/arduino --upload --verbose-upload \
  --board arduino:avr:nano:cpu=atmega328 \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 \
  /home/ros2/Documents/volt_ws/firmware/volt_standalone_forward_walk/volt_standalone_forward_walk.ino
```

If that Nano uses the old bootloader, replace `atmega328` with
`atmega328old`.

## Run without ROS

Open Arduino Serial Monitor at 115200 baud with newline enabled. With the robot
supported and the servo-power disconnect reachable:

```text
ARM
```

Wait at least two seconds, then request one finite cycle:

```text
RUN 1
```

Use `STOP` for a controlled stop. Use `HOLD` only when an immediate freeze is
necessary. Use `DISABLE` only when removing holding torque cannot make the
robot fall.
