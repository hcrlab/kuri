#! /usr/bin/python

import random as rand

import rospy
from std_msgs.msg import Float64


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    """ Publishes a random float every few seconds. Purely for testing the slack bot's ability
        to post messages when requested by Kuri over ROS
    """
    rospy.init_node('dummy_publisher')
    wait_for_time()
    torso_pub = rospy.Publisher('dummy_topic',
                                Float64)
    rospy.sleep(0.5)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        torso_pub.publish(Float64(rand.uniform(0.0, 1.0)))
        rate.sleep()


if __name__ == '__main__':
    main()
