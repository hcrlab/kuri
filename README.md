# Kuri

Packages for working with Mayfield's Kuri robot.

## Usage

### On an external machine

Build this repo's packages in your workspace. You should be set to communicate with the robot. Check out the helpers in `kuri_launch/env_hooks` (accessible in any shell that has the workspace sourced).

You may find [our docs](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Usage) on configuring the robot helpful.

### On the robot

Check out each individual package for tips. Start with `kuri_launch`.

Note that many of these packages are stubs just to provide message types for communicating with the robot. You'll need to remove or ignore these packages when building on the robot to ensure that the full versions of these packages (which include important nodes) can be found while launching. On our Kuri's, we added empty `CATKIN_IGNORE` files to the following directories: `mayfield_msgs`, `mobile_base_driver`, `madmux`, `kuri_gazebo`, `vision_msgs`, `may_nav_msgs`, `audio_msgs`.
