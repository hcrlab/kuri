# kuri_navigation

Nodes for working with Kuri's default navigation software. These are taken almost directly from the [kuri_edu](https://github.com/MayfieldRoboticsPublic/kuri_edu) packages.

## `mapping_controller`

A wrapper for the OORT mapping process. When using this node:

* Use the mapping specific launch file from `kuri_launch`
* Start the node to begin mapping. Pass a name as the only argument
* Teleop the robot through some interface
* Don't drive too quickly or mapping won't work well
* Monitor map quality through Rviz
* Kill the node to save the map

## `safety_controller`

Tells the base driver that we're paying attention to signals from the bumpers. Without this node, the base will
refuse velocity commands.

## `may_nav_controller`

A helper node that starts localization, loads the OORT serialized map off disk into the localization node, and provides
an initial localization guess.

With some changes, it should be possible to use a regular .pgm map.

## `bumper_to_pointcloud` 

Used to integrate the bumper and cliff sensors with a standard Costmap2D obstacle layer.