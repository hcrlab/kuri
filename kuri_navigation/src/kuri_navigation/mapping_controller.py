import rospy
import signal
import sys

import mayfield_msgs.msg
import mayfield_utils
import mobile_base

from kuri_navigation.oort_map_manager import OortMapManager
from kuri_api.power import PowerMonitor

class MappingController:
    '''
    The mapping controller node allows you to create a map using kuri's
    mapping system, OORT.

    The user drives Kuri using the keyboard teleop node.

    You can monitor the progress of mapping using a tool like rviz.  When the
    map reaches 20 squre meters, Kuri will start to smile.

    To stop mapping, kill the node.  The map will be saved to the user's home
    directory:

    ~/oort_maps/<uuid>/map.map_capnp      - OORT map file
    ~/oort_maps/<uuid>/map.map_meta_capnp - OORT map file
    ~/oort_maps/<uuid>/map.md5            - OORT map file checksum
    ~/oort_maps/<uuid>/map.pgm            - map_saver output (standard format)
    ~/oort_maps/<uuid>/map.yaml           - map_saver output (standard format)

    A different UUID will be used each time the mapping_controller creates a
    new map
    '''

    def __init__(self, map_name):

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

        # Joint states keeps track of the latest position of all the robot's
        # joints
        self._joint_states = mobile_base.JointStates()


        self._power_monitor = None  # Created when we start to run
        self._map_manager = OortMapManager()
        
        # catch ctrl-c and signal the main thread that we should save the map 
        # and quit
        def signal_handler(sig, frame):
            self._mapping_complete = True
        signal.signal(signal.SIGINT, signal_handler)
        self._mapping_complete = False
        self.map_name = map_name
        self._start_mapping()


    def run(self):

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("mapping_controller", True)
        )

        self._power_monitor = PowerMonitor()
        #self._power_monitor.docked_event.connect(self._dock_changed_cb)
        #self._power_monitor.undocked_event.connect(self._dock_changed_cb)

        # _start_mapping and _stop_mapping are called from ROS callbacks
        # once _stop mapping runs, it will set self._mapping_complete
        # to True.  At this point, the main loop below will save the map
        # in a standard format, and then stop

        try:
            while not rospy.is_shutdown():
                if self._mapping_complete:
                    # Telling OORT to close off the map can be a time
                    # consuming process, so we're doing it from the main
                    # thread instead of from a ROS callback
                    self._map_manager.stop_mapping()
                    rospy.logwarn("Mapping complete.  Converting. . .")
                    self._map_manager.convert_map()
                    rospy.logwarn("Map conversion complete")
                    return
                rospy.sleep(0.5)
        except rospy.exceptions.ROSInterruptException:
            return
  

    def shutdown(self):
        self._joint_states.shutdown()

        if self._power_monitor:
            self._power_monitor.shutdown()

        self._map_manager.shutdown()


    def _start_mapping(self):

        map_path = self._map_manager.start_mapping(self.map_name)
        # "dock" is a reserved waypoint name in OORT.  It's used for loop
        # closures as the map is added to
        self._map_manager.save_waypoint("dock", "dock")

        # This is a warning so it's easier to see in the log
        rospy.logwarn("Started mapping.  Map stored at {}".format(map_path))

    def _stop_mapping(self):
        # Notifying OORT that we're docked will give it a hint about how to do
        # the final loop closure
        #self._map_manager.notify_docked()
        # Telling OORT to finish off the map is time consuming.  We'll do that
        # from the main thread
        self._mapping_complete = True
