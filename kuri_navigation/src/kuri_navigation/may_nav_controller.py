import os.path

import rospy
from tf.transformations import euler_from_quaternion

import mayfield_msgs.msg
import mayfield_utils

import geometry_msgs.msg

from .oort_map_manager import OortMapManager
from robot_api.power import PowerMonitor


class MayNavController:
    '''
    The nav controller is responsible for loading saved maps.  This
    class will look in the home directory for a simlink to a map created
    with kuri_mapping.launch and attempt to load it and start localization
    '''

    MAP_PATH = os.path.expanduser("~/map.map_capnp")

    def __init__(self):

        self._pub = rospy.Publisher(
            "node_online",
            mayfield_msgs.msg.NodeStatus,
            latch=True,
            queue_size=1,
        )

        # Controllers need to be loaded before we can run
        mayfield_utils.wait_for_nodes(
            node_names=['controllers'],
        )

        self._map_manager = OortMapManager()
        self._power_monitor = None  # Created when we start to run


    def run(self):
        '''
        Look in the home directory to see if there's a simlink to an active
        map.  If there is, have OORT load it, and start AMCL localization
        '''
        if os.path.isfile(MayNavController.MAP_PATH):
            self._map_manager.load_map(MayNavController.MAP_PATH)
            self._map_manager.start_localization()
            # Assume that on start-up we're on or near the dock.  It's as good
            # a guess as any. . .
            self._map_manager.localize_on_dock()
            self._power_monitor = PowerMonitor(
                dock_changed_cb=self._dock_changed_cb
            )

        else:
            # If we didn't find a map, that may be OK.  The user may not have
            # created one yet.
            # See kuri_mapping.launch and the assosciated activity to see how
            # to create a map
            rospy.logwarn(
                "Did not find a simlink to a map file at ~/map.map_capnp"
            )

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("nav_controller", True)
        )

        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._map_manager.shutdown()

    def _dock_changed_cb(self, msg):
        '''
        To help AMCL, relocalize to the dock whenever we're on it
        '''
        if msg.dock_present:
            self._map_manager.localize_on_dock()
