#!/usr/bin/env python

import rospy
import math
from statistics import mean
import random
import time
from collections import deque
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from kuri_person_tracking.msg import BoundingBoxes

from position_head import position_head

import kuri_params as params

ROTATION_SPEED = rospy.get_param('robot/rotational_speed', 0.8)
FORWARD_SPEED = rospy.get_param('robot/forward_speed', 0.2)

BOUNDING_BOXES_TOPIC = rospy.get_param('subscribers/bounding/box/topic', '/upward_looking_camera/bounding_boxes')
BOUNDING_BOXES_QSIZE = rospy.get_param('subscribers/bounding/box/queue_size', 1)
SCAN_TOPIC = rospy.get_param('subscribers/scan/topic', '/scan')
SCAN_QSIZE = rospy.get_param('subscribers/scan/queue_size', 1)

VELOCITY_TOPIC = rospy.get_param('publishers/velocity/topic', '/mobile_base/commands/velocity') # 'kuri_navigation_command_velocity'
VELOCITY_QSIZE = rospy.get_param('publishers/velocity/queue_size', 1)


CENTER_PICKING_STYLE = 2
_center_picking_styles = [
    'mean',
    'random',
    'highest_probability',
]

bounding_box_persistance = 1 # the number of secs to keep a bounding box center before re-scanning

# set by pick_center()
center = None
center_time_recv = None
latest_positions = None
scan_density = None

scan_window = deque()
scan_timestamps_window = deque()
SCAN_LIFESPAN = 1 # in secs

def main():
    """ Tracks a person if one is in view
    """
    global center, pending_update

    rospy.init_node('move_towards_node')

    _wait_for_time()

    position_head()

    params._get_camera_parameters()

    # Base velocity topic publisher
    base_publisher = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=VELOCITY_QSIZE)

    # Subscribe to the person bounding boxes
    bb_sub = rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, bounding_boxes_callback, queue_size=BOUNDING_BOXES_QSIZE)
    # Subscribe to the scans
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, scan_callback, queue_size=SCAN_QSIZE)

    # Rate in Hz
    rate = rospy.Rate(2)

    state = "idle"
    available_states = ["idle", "following", "waiting"]

    idle_animation_velocities = [-0.4, 0.4]
    num_cycles_remaining_for_idle_animation = 0
    
    following_start = None
    sig_following_duration = 3.0 # secs

    waiting_start = None
    waiting_duration = 5.0 # secs


    while not rospy.is_shutdown():
        print(state)
        if state == "idle":
            # idle animation
            if num_cycles_remaining_for_idle_animation == 0:
                num_cycles_remaining_for_idle_animation = random.randint(6, 16) # 3 - 8 secs at a rate of of 2Hz
                idle_ang_z = random.choice(idle_animation_velocities)

            publish_base_cmd(base_publisher, 0.0, idle_ang_z)
            num_cycles_remaining_for_idle_animation -= 1

            if center:
                state = "following"
                following_start = time.time()
            
        elif state == "following":
            # Immediately transition if no center
            if (close_enough() or not center) \
                    and time.time() - following_start > sig_following_duration:
                state = "waiting"
                publish_base_cmd(base_publisher, 0.0, 0.0)
                waiting_start = time.time()
                continue
            elif not center:
                state = "idle"
                publish_base_cmd(base_publisher, 0.0, 0.0)
                continue

            rotation = pixels_to_rotation(center[0], center[1])
            ang_z = -rotation * ROTATION_SPEED

            publish_base_cmd(base_publisher, FORWARD_SPEED, ang_z)

            num_cycles_remaining_for_idle_animation = 0
        
        elif state == "waiting":
            # pass

            # Transition
            if time.time() - waiting_start > waiting_duration:
                state = "idle"

        rate.sleep()

    rospy.spin()


def bounding_boxes_callback(data):
    """ Processes the bounding box data
    """
    global center, center_time_recv

    bounding_boxes = data.bounding_boxes

    # get bounding box centers
    centers_and_probabilities = list(map(get_center, bounding_boxes))

    # set global center variable
    if len(centers_and_probabilities) > 0:
        center = tuple(pick_center(centers_and_probabilities))
        center_time_recv = time.time()

    # If the person has left the camera frame,
    # stop tracking
    if center_time_recv is not None and time.time() - center_time_recv > bounding_box_persistance:
        center = None
        center_time_recv = None


def joint_states_callback(data):
    global latest_positions
    positions = {}
    for i, name in enumerate(data.name):
        positions[name] = data.position[i]

    latest_positions = positions


def scan_callback(data):
    global scan_window, scan_timestamps_window

    range_max = data.range_max

    ranges = data.ranges
    scan_window.append(ranges)
    original_len = len(ranges)

    timestamp = rospy.Time(secs=data.header.stamp.secs, nsecs=data.header.stamp.nsecs)
    scan_timestamps_window.append(timestamp)

    # check if oldest scan is expired
    if (timestamp.to_sec() - scan_timestamps_window[0].to_sec() > SCAN_LIFESPAN):
        scan_window.popleft()
        scan_timestamps_window.popleft()


def close_enough():
    global scan_window, scan_timestamps_window

    if len(scan_window) <= 1:
        return False

    scan_window_arr = np.array(scan_window)
    scan_window_arr[scan_window_arr == float('inf')] = float('nan') # filter infs

    num_scan_points = scan_window_arr.shape[1]
    scan_window_arr = scan_window_arr[: , num_scan_points // 3 : num_scan_points // 3 * 2] # take front third of scan


    scan_mean = np.nanmean(scan_window_arr, axis=0)     
    scan_mean = scan_mean[~ np.isnan(scan_mean)] # filter nans
    
    scan_mean_mean = np.mean(scan_mean)

    return scan_mean_mean < 1.0 # 1.0 # 2.0


def get_center(bounding_box):
    """ Returns center of given bounding box as a float tuple
    """
    xmax = bounding_box.xmax
    xmin = bounding_box.xmin

    ymax = bounding_box.ymax
    ymin = bounding_box.ymin

    return ((xmax + xmin) / 2, (ymax + ymin) / 2), bounding_box.probability


def pick_center(centers):
    """ Picks a center out of the detected human bounding boxes' centers
        using the strategy specified by the CENTER_PICKING parameter
    """
    style = _center_picking_styles[CENTER_PICKING_STYLE]

    center_x, center_y = 0, 0
    if style == 'mean':
        for center in centers:
            center_x += center[0][0] / len(centers)
            center_y += center[0][1] / len(centers)
    elif style == 'random':
        import random as rand

        center = rand.choice(centers)
        center_x = center[0][0]
        center_y = center[0][1]
    elif style == 'highest_probability':
        max_prob = None
        center_x = None
        center_y = None
        for center in centers:
            prob = center[1]
            if max_prob is None or prob > max_prob:
                max_prob = prob
                center_x = center[0][0]
                center_y = center[0][1]
    return center_x, center_y


def publish_base_cmd(base_publisher, lin_x, ang_z):
    twist = Twist()
    twist.linear.x = lin_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ang_z
    base_publisher.publish(twist)


def pixels_to_rotation(px_x, px_y):
    """ Convert coordinates in pixels to a rotational velocity
        coordinates are from the bottom left corner of the image, with x along
        the horizontal axis and y the vertical.
    """
    return (px_x - 0.5)


def _wait_for_time():
    """ Wait for rospy to spool up
    """
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == '__main__':
    main()
