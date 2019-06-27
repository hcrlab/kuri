# hcrl_kuri_launch

Launch files for Kuri.

## Usage

To get a vanilla, controllable interface to the robot running

    roslaunch hcrl_kuri_launch kuri.launch

### Mapping Mode

Kuri's default navigation stack comes with a custom SLAM implementation, OORT. To run the mapping process:

    roslaunch hcrl_kuri_launch kuri_mapping.launch

Take a closer look at `kuri_navigation` and the `mapping_controller` node to understand how to map.

Kuri's sensor can only see things up to about 7 feet away and has a FOV of about 170 degrees.
It can be difficult to successfully map large spaces.
