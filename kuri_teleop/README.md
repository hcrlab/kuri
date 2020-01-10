# Kuri Teleoperation

This package contains code to teleoperate Kuri from a remote computer. Unlike the other packages in this repository, this code is *not* intended to be run on the Kuri, but rather intended to be run on a remote computer. (Keyboard teleop can be run on the Kuri with ssh, joystick teleop cannot be run on the Kuri unless you connect a joystick directly to the Kuri).

## Dependencies
- ROS's joy package http://wiki.ros.org/joy

## Nodes
- `keyboard_teleop` the same node as in [twist_keyboard_teleop](https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py)
- `joystick_teleop` a custom node to control the Kuri with an external joystick.

## Usage

### Keyboard Teleoperation
Run `roslaunch kuri_teleop keyboard_teleop.launch` and follow the instructions on-screen.

### Joystick Teleoperation
To use this package, first ensure that your remote computer can detect the joystick (see http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick). Then, change the "dev" rosparam in `kuri_joystick_teleop.launch` to the appropriate path for your joystick. Finally, run `roslaunch kuri_teleop joystick_teleop.launch`.

This package has been tested with Logitech's F710 joystick. On that joystick (in XInput mode), the left joystick's left-right controls the Kuri's angular velocity, the right joystick's up-down controls its linear velocity, LB and RB are e-stops, start re-enables the robot, the up-down arrow keys control the head tilt, the left-right arrow keys control the head pan, the A button toggles the eyes open/closed, and the X button re-center's the head's pan.

This code will likely need modifications to work with other joysticks. To do so, first determine which joystick control maps to which part of the rosmsg outputted by the `joy` package. Then modify the `joystickCallback` function in `kuri_joystick_teleop.cpp` accordingly. If you add support for a new joystick, we encourage you to submit a poull request to merge it into this repository.

In some cases, one may want the joystick velocity commands to override commands from another topic (e.g. the joystick e-stop can override outputs from the ROS navigation stack, or the joystick can issue corrections to velocities from the ROS navigation stack). An example of how to achieve this is shown with the subscription to the `kuri_navigation_command_velocity` topic in this code (you will need to change that topic to whatever topic is issuing velocity commands that you want to possibly override).

## Notes
- kuri_edu does have a [joystick_teleop node](https://github.com/MayfieldRoboticsPublic/kuri_edu/blob/master/kuri_edu/src/kuri_edu/joystick_teleop.py). However, that node must be run **on the Kuri** (since it sends commands directly to the mobile_base_driver), although the joystick likely has to be connected to a remote computer (due to the difficulty of pairing a joystick directly to the Kuri). On the other hand, our joystick_teleop node sends commands to the Kuri using ros messages, so both the joystick and the joystick_teleop code can be run on the remote computer. To avoid the complex setup in the kuri_edu joystick_teleop node, we do not include that node in this package.

## TODO
- Convert the hardcoded values in `joystick_teleop.cpp`that users may want to change (such as head pan speech, head pan duration, controll loop hertz, etc.) to ros params.
