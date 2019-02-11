# kuri_navigation

Nodes for working with Kuri's default navigation software. These are taken almost directly from the [kuri_edu](https://github.com/MayfieldRoboticsPublic/kuri_edu) packages.

## `mapping_controller`

A wrapper for the OORT mapping process. When using this node:

* Start with robot on dock
* Teleop the robot through some interface
* Don't drive to quickly or mapping won't work well
* Monitor map quality through Rviz
* Drive it back onto the dock to save out the map.

## `safety_controller`

Tells the base driver that we're paying attention to signals from the bumpers. Without this node, the base will
refuse velocity commands.

## `may_nav_controller`

A helper node that starts localization, loads the OORT serialized map off disk into the localization node, and provides
an initial localization guess.

With some changes, it should be possible to use a regular .pgm map.