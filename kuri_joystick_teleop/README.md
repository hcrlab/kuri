# Kuri Joystick Teleoperation

This package contains code to teleoperate Kuri from a remote computer with a joystick. Unlike the other packages in this repo, this code *cannot* be run directly on a Kuri (unless you connect a joystick directly to the Kuri).

## Dependencies
- ROS's joy package http://wiki.ros.org/joy

## Usage
To use this package, first ensure that your remote computer can detect the joystick (see http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick). Then, change the "dev" rosparam in `kuri_joystick_teleop.launch` to the appro[priate path for your joystick. Finally, run `roslaunch kuri_joystick_teleop kuri_joystick_teleop.launch`.

This package has been tested with Logitech's F710 joystick. On that joystick (in XInput mode), the left joystick's left-right controls the Kuri's angular velocity, the right joystick's up-down controls its linear velocity, LB and RB are e-stops, start re-enables the robot, the up-down arrow keys control the head tilt, the left-right arrow keys control the head pan, and the A button toggles the eyes open/closed.

This code will likely need modifications to work with other joysticks. To do so, first determine which joystick control maps to which part of the rosmsg outputted by the `joy` package. Then modify the `joystickCallback` function in `kuri_joystick_teleop.cpp` accordingly.

In some cases, one may want the joystick velocity commands to override commands from another topic (e.g. the joystick e-stop can override outputs from the ROS navigation stack, or the joystick can issue corrections to velocities from the ROS navigation stack). An example of how to achieve this is shown with the subscription to the `kuri_navigation_command_velocity` topic in this code (you will need to change that topic to whatever topic is issuing velocity commands that you want to possibly override).

## TODO
- Convert the hardcoded values that users may want to change (such as head pan speech, head pan duration, controll loop hertz, etc.) to ros params.
