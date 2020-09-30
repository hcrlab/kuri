#!/usr/bin/env python

import rospy
import math
from statistics import mean
import random
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from simple_person_detection.msg import BoundingBoxes

import kuri_params as params

ROTATION_SPEED = 0.8

BOUNDING_BOXES_TOPIC = '/upward_looking_camera/bounding_boxes'
VELOCITY_TOPIC = 'kuri_navigation_command_velocity'#'/mobile_base/commands/velocity'
HEAD_TOPIC = '/head_controller/command'
JOINTS_TOPIC = '/joint_states'
SCAN_TOPIC = '/scan'

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

def main():
    """ Tracks a person if one is in view
    """
    global center, pending_update

    rospy.init_node('eye_publisher')

    _wait_for_time()

    params._get_camera_parameters()

    # Head joints topic publisher
    head_publisher = rospy.Publisher(HEAD_TOPIC, JointTrajectory, queue_size=1)
    # Base velocity topic publisher
    base_publisher = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=1)

    # Subscribe to the person bounding boxes
    bb_sub = rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, bounding_boxes_callback, queue_size=1)
    # Subscribe to the joint states
    joint_sub = rospy.Subscriber(JOINTS_TOPIC, JointState, joint_states_callback, queue_size=1)
    # Subscribe to the scans
    scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, scan_callback, queue_size=1)

    # Rate in Hz
    rate = rospy.Rate(2)

    idle_animation_velocities = [-0.4, 0.4]
    num_cycles_remaining_for_idle_animation = 0

    while not rospy.is_shutdown():

        if center:
            print("Moving towards person")
            rotation = pixels_to_rotation(center[0], center[1])
            # if abs(rotation) > 0.1:
            ang_z = -rotation * ROTATION_SPEED

            # print('Publishing to base', ang_z)

            if close_enough():
                print("close enough")
                publish_base_cmd(base_publisher, 0.0, ang_z)
            else:
                print("move at 0.2")
                publish_base_cmd(base_publisher, 0.0, ang_z)
            num_cycles_remaining_for_idle_animation = 0
        else:
            print("idle animation")
            # idle animation
            if num_cycles_remaining_for_idle_animation == 0:
                num_cycles_remaining_for_idle_animation = random.randint(6, 16) # 3 - 8 secs at a rate of of 2Hz
                idle_ang_z = random.choice(idle_animation_velocities)

            publish_base_cmd(base_publisher, 0.0, idle_ang_z)
            num_cycles_remaining_for_idle_animation -= 1

        # just to tilt the head at a good angle
        # publish_head_pos(head_publisher, 0.0, -0.6)
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
        # print("bounding box callback set center", center)

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
    global scan_density

    range_max = data.range_max

    ranges = data.ranges
    original_len = len(ranges)

    filtered_ranges = list( filter(lambda x: x != float('inf') and not math.isnan(x), ranges) )
    filtered_len = len(filtered_ranges)

    # print(filtered_ranges)

    scale = original_len / filtered_len

    mean_range = mean(filtered_ranges) * scale

    scan_density = mean_range


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


def close_enough():
    global scan_density

    print("scan_density", scan_density)

    return scan_density < 0.5 # 1.0 # 2.0


def publish_base_cmd(base_publisher, lin_x, ang_z):
    twist = Twist()
    twist.linear.x = lin_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ang_z
    # print("Publish to base", lin_x, ang_z)
    base_publisher.publish(twist)


def publish_head_pos(head_publisher, pan, tilt):

    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    p = JointTrajectoryPoint()
    p.positions = [pan, tilt]
    p.velocities = []
    p.effort = []
    p.time_from_start = rospy.Time(1)
    traj.points = [p]

    head_publisher.publish(traj)


def pixels_to_rotation(px_x, px_y):
    """ Convert coordinates in pixels to a rotational velocity
        coordinates are from the bottom left corner of the image, with x along
        the horizontal axis and y the vertical.
    """
    print("Proportion", px_x)
    return (px_x - 0.5)

def _wait_for_time():
    """ Wait for rospy to spool up
    """
    while rospy.Time().now().to_sec() == 0:
        pass

if __name__ == '__main__':
    main()
