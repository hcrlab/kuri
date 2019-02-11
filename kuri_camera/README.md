# kuri_camera

This package provides a node that broadcasts Kuri's camera over ROS. You may find it helpful for debugging, but know that
it's expensive (consuming 30-40% of the CPU just serializing data). Prefer running perception on-robot and pulling directly
from madmux.

C++ targets link against madmux, so it will only build on Kuri.


## Usage

    rosrun kuri_camera ros_publisher.py

## Known issues

The C++ targets demonstrate simple API usage but nonetheless hit runtime loader errors when they call
certain madmux functions. Use the Python scripts for now.