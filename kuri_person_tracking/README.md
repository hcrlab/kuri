## Person Tracking

### Install

Run `source scripts/download_yolo.bash` to download the necessary tiny-yolov3 cfg and weight files. Pass `full` to this script to also download ordinary yolov3 cfg and weight files.

In `config/ros.yaml`, set the `net_cfg_dir` to the directory containing your yolo cfg and weight files (downloaded in the previous step). In that same config file, you can configure the ros topics used by your robot as well as various person_tracking settings. These include detection and non-maximum-suppression thresholds as well as whether you want to display your robots live feed on your screen along with the bounding boxes.

### Usage

Roslaunch `move_towards.launch` to launch Kuri's person detection and have it move towards a person in its field of view. Kuri will passively 'search' for a person by rotating periodically until it sees someone. This makes use of the aforementioned `person_detection.py` bounding box publisher for information on peoples' locations.

After Kuri gets close enough to a person, `move_towards.py` has a 'waiting' state during which Kuri does nothing for several seconds. During this state, you can add whatever behavior you want your robot to exhibit when it reaches a person.

To only launch person detection, roslaunch `person_detection.launch` to publish bounding boxes of detected people to a set topic. 

### Dependencies

This has been confirmed to work with Python versions 3.5 and 3.8, but likely supports any relatively recent version of python3.

You can run `pip install -r requirements.txt` to install the basic dependencies (there may be one or two that you need to install manually but they'll be made clear by error messages).

NOTE: This package requires a version of numpy which is higher than the default one installed by some versions of ROS. However, upgrading the default numpy will uninstall several ROS packages, which is undesired behavior. One way to overcome this is to install a new version of Python that is not currently on your device, install the required version of numpy on that Python, and then have ROS run the node in that version of Python. The way to control which Python version ROS runs your node in is by changing the first line `#!/usr/bin/env python`to whatever command you use to load that version of Python in terminal.
