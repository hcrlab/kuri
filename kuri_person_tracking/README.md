## Person Tracking

### Install

Run `source scripts/download_yolo.bash` to download the necessary tiny-yolov3 cfg and weight files. Pass `full` to this script to also download ordinary yolov3 cfg and weight files.

In `config/ros.yaml`, set the `net_cfg_dir` to the directory containing your yolo cfg and weight files (downloaded in the previous step). In that same config file, you can configure the ros topics used by your robot as well as various person_tracking settings. These include detection and non-maximum-suppression thresholds as well as whether you want to display your robots live feed on your screen along with the bounding boxes.

### Usage

Roslaunch `person_detection.launch` to publish bounding boxes of detected people in Kuri's camera to a set topic. 

Roslaunch `move_towards.launch` to have Kuri move towards a person in its field of view. Kuri will passively 'search' for a person by rotating periodically until it sees someone.
This makes use of the aforementioned `person_detection.py` bounding box publisher for information on peoples' locations (Note: `move_towards.launch` also launches `person_detection.launch` automatically).

`move_towards.py` has a a 'waiting' state during which it does nothing for several seconds. During this state, you can add whatever behavior you want your robot to exhibit when it reaches a person.

### Dependencies

This has been confirmed to work with Python versions 3.5 and 3.8, but likely supports any relatively recent version of python3.

You can run `pip install -r requirements.txt` to install the basic dependencies (there may be one or two that you need to install manually but they'll be made clear by error messages).