#! /usr/bin/env python

import rospy
import math
import copy
import rospy.rostime
import tf.transformations as tft
from geometry_msgs.msg import Twist
import nav_msgs.msg._Odometry

def normalize_angle_positive(angle):
    """Normalizes angle to be in [0, 2pi)
    """
    return (angle % (2.0 * math.pi)) % (2.0 * math.pi)

def normalize_angle(angle):
    """Normalizes angle to be in [-pi, pi)
    """
    pos = normalize_angle_positive(angle)
    if pos >= math.pi:
      return pos - 2.0 * math.pi
    return pos

def quat_heading(quat):
    x, y, z, w = (quat.x, quat.y, quat.z, quat.w)
    return math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('/mobile_base/command/velocity', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        self.latest_odom = None
        self.latest_pose = None
        self.latest_yaw = None
        self.no_odom_received = True

    def _odom_callback(self, msg):
        self.no_odom_received = False
        self.latest_odom = msg
        self.latest_yaw = quat_heading(msg.pose.pose.orientation)
        self.latest_pose = msg.pose.pose.position
    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """

        # rospy.sleep until the base has received at least one message on /odom
        while self.no_odom_received:
            rospy.loginfo_throttle(10, "Waiting for odom message before moving...")
            rospy.sleep(0.3)

        # Record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_odom)
        start_pose = start.pose.pose.position

        # Check if the robot has traveled the desired distance
        distance_error = abs(distance)
        direction = -1 if distance < 0 else 1
        rate = rospy.Rate(50)
        while distance_error > 0.01 and not rospy.is_shutdown():
            p = min(speed, speed * distance_error + 0.1)
            self.move(direction * p, 0)
            # We're controlling the distance that the robot travels, trying to make it as close to abs(distance) as possible
            distance_error =  abs(distance) - math.sqrt(((self.latest_pose.x - start_pose.x) ** 2)+((self.latest_pose.y - start_pose.y) ** 2))
            rate.sleep()
        self.stop()
        
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """

        # rospy.sleep until the base has received at least one message on /odom
        while self.no_odom_received:
            rospy.loginfo_throttle(10, "Waiting for odom message before moving...")
            rospy.sleep(0.3)

        start_yaw = normalize_angle_positive(quat_heading(self.latest_odom.pose.pose.orientation)) 
        last_yaw = start_yaw       
        target_yaw = normalize_angle_positive(start_yaw + angular_distance) 

        # How much we have left to go. When we haven't moved, we're either
        # already spot on, or the error is the angular distance provided in the command
        yaw_error = normalize_angle_positive(target_yaw - start_yaw)

        # We'll always turn in the direction of the command, even if it would be
        # shorter to go the other way.
        # This will cause issues if we overshoot, because we'll just go back aroud 
        # the long way.
        direction = -1 if angular_distance < 0 else 1   
        rate = rospy.Rate(50)
        # We'll tolerate an error slightly smaller than a degree
        while yaw_error > 0.01 and not rospy.is_shutdown():
            # Simple proportional control with fixed direction
            # Make control proportional to the error, with min of 0.1 and max of `speed`
            p = min(speed, speed * yaw_error + 0.1)
            self.move(0, direction * p)
            last_yaw = normalize_angle_positive(self.latest_yaw)
            yaw_error = normalize_angle_positive(target_yaw - last_yaw)
            rate.sleep()
        self.stop()

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist Message
        msg = Twist()

        # Fill out message
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        # Publish message
        self.pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.pub.publish(msg)

