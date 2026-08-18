# Third-party notices

## mike4192/spotMicro

- Project: [mike4192/spotMicro](https://github.com/mike4192/spotMicro)
- Upstream revision studied: `2a34f5d303dff91b62180031b31ef512a672f3c3`
- License: MIT
- Copyright: Copyright (c) 2020 mike4192

VOLT cleanly adapts the upstream walking concepts of a four-leg crawl sequence
and a body-support shift before each single-leg swing. In particular, the
upstream-derived leg order is rear right, front right, rear left, front left.

VOLT does not copy Spot Micro geometry, link lengths, joint conventions, servo
calibration, PCA9685 mappings, ROS 1 interfaces, or hardware-control code. The
VOLT implementation uses its own URDF and kinematics, support-triangle
projection, world-locked stance feet, smooth swing trajectories, ROS 2 command
routing, and safe stop/switch behavior. This notice records the provenance of
the adapted gait-sequencing and support-shift concepts; it does not imply that
the Spot Micro source was copied wholesale.

### MIT License

Copyright (c) 2020 mike4192

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
