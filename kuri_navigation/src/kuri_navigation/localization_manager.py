import math

import amcl
import nav_msgs
import rospy
import std_srvs
from nav_msgs.srv import SetMapRequest

from kuri_navigation.pose_utils import _pose_to_posecov, se2_to_pose


class LocalizationManager:
    def __init__(self):
        self._amcl_start_srv = rospy.ServiceProxy(
            "localization_start",
            std_srvs.srv.Empty
        )
        self._amcl_start_srv.wait_for_service(10)

        self._relocalization_pub = rospy.Publisher(
            "initialpose_cloud",
            amcl.msg.HypothesisSet,
            queue_size=1
        )

    def _got_msg(self, msg):
        self.map_data = msg

    def start_localization(self):
        self.map_data = None
        rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self._got_msg)
        while not self.map_data and not rospy.is_shutdown():
            print("waiting")
            rospy.sleep(1)

        self._amcl_start_srv()
        req = SetMapRequest()
        req.map = self.map_data
        rospy.ServiceProxy("/set_map", nav_msgs.srv.SetMap)(req)

        hset = amcl.msg.HypothesisSet(
            hypotheses=[
                _pose_to_posecov(
                    se2_to_pose((0., 0., 0.)),
                    (.15, .15, math.radians(25))
                )
            ]
        )

        self._relocalization_pub.publish(hset)