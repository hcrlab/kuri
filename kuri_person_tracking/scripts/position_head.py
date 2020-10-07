#! /usr/bin/python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

EYELIDS_TOPIC = '/eyelids_controller/command'
HEAD_TOPIC = '/head_controller/command'
OPEN_EYES = 0.04
HEAD_UP = [0.0, -0.4]


def main():
    rospy.init_node('position_head_node')

    wait_for_time()

    position_head()


def position_head():
    """ Opens Kuri's eyes and raises head
    """
 
    eyes_pub = rospy.Publisher(EYELIDS_TOPIC, JointTrajectory, queue_size=10)
    head_pub = rospy.Publisher(HEAD_TOPIC, JointTrajectory, queue_size=1)
    
    rospy.sleep(0.5)

    publish_eye_pos(eyes_pub, OPEN_EYES)
    publish_head_pos(head_pub, *HEAD_UP)


def publish_eye_pos(eye_publisher, val):
    traj = JointTrajectory()
    traj.joint_names = ["eyelids_joint"]
    p = JointTrajectoryPoint()
    p.positions = [val]
    p.velocities = []
    p.effort = []
    p.time_from_start = rospy.Time(1)
    traj.points = [p]

    eye_publisher.publish(traj)
    

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


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == '__main__':
    main()