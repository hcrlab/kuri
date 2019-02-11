# kuri_gazebo

This package provides launch files and controller configurations for the Kuri. These configurations
are based on the parameters observed on the actual robot.

The Gazebo plugin is based on that of the [REEM simulation](https://github.com/pal-robotics/reem_simulation).


## Usage

To start the simulated robot in an empty world:

    roslaunch kuri_gazebo kuri_gazebo.launch

Kuri's sensors won't show much or anything in the empty world, so add some items through the Gazebo interface.
