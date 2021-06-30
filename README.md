# Kuri

Packages for working with Mayfield's Kuri robot.

## Compilation

### On an External Machine

Build this repo's packages in your workspace. You should be set to communicate with the robot. Check out the helpers in `kuri_launch/env_hooks` (accessible in any shell that has the workspace sourced).

You may find [our docs](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Usage) on configuring the robot helpful.

### On the Robot

Check out each individual package for tips. Start with `kuri_launch`.

Note that many of these packages are stubs just to provide message types for communicating with the robot. You'll need to remove or ignore these packages when building on the robot to ensure that the full versions of these packages (which include important nodes) can be found while launching. You can put `CATKIN_IGNORE` in these, or ignore them with

    catkin config --blacklist audio_msgs kuri_gazebo madmux may_nav_msgs mayfield_msgs mobile_base_driver vision_msgs

#### Kuri Camera

There is a [known bug](https://github.com/KuriRobot/Kuri-Documentation/issues/33) where the the Kuri has an unnecessary and incomplete [madmux](https://github.com/KuriRobot/Kuri-Documentation/blob/master/reference/ros-packages/madmux.md) library in the `/opt/ros/indigo` folder (the correct one is in `/opt/gizmo`). To prevent ROS from linking to the wrong madmux library, move or delete all remnants of madmux from the `/opt/ros/indigo` folder *before compiling* the code. (If you have already compiled the code, you may have to clean your workspace before re-compiling). Specifically, run the following commands (these will delete the files -- if you would rather move the files for backup, modify the commands appropriately):

`sudo rm -r /opt/ros/indigo/include/madmux`

`sudo rm /opt/ros/indigo/lib/libmadmux.so`

`sudo rm /opt/ros/indigo/lib/libmadmux.so.0`

`sudo rm /opt/ros/indigo/lib/libmadmux.so.0.2.0`

`sudo rm -r /opt/ros/indigo/share/madmux`
