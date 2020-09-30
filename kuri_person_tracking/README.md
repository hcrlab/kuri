## Person Tracking

### Install

Run `source scripts/download_yolo.bash` to download the necessary tiny-yolov3 cfg and weight files. Pass `full` to this script to also download ordinary yolov3 cfg and weight files.


### Usage

Roslaunch `person_detection.launch` to publish bounding boxes of detected people in Kuri's camera to a set topic. Pass `display:=true` to display a camera view with bounding boxes.
Configure things like detection threshold and non-maximum-suppression threshold in `person_detection.py`.


Roslaunch `move_towards.launch` to have Kuri move towards a person in its field of view. Kuri will passively 'search' for a person by rotating periodically until it sees someone.

### Dependencies
- `pip install statistics`
- `pip install imutils`
