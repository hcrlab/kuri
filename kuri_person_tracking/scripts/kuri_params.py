#!/usr/bin/env python

import rospy

HEAD_MAX_PAN = 0.75
HEAD_MIN_PAN = -0.75
HEAD_MAX_TILT = 0.30
HEAD_MIN_TILT = -0.86

CAMERA_INFO_WIDTH = None
CAMERA_INFO_HEIGHT = None


def _get_camera_parameters():
    global CAMERA_INFO_WIDTH, CAMERA_INFO_HEIGHT

    print("Getting camera parameters")
    CAMERA_INFO_WIDTH = rospy.get_param('image_width')
    CAMERA_INFO_HEIGHT = rospy.get_param('image_height')
