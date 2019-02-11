import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base

from kuri_navigation.oort_map_manager import OortMapManager
from robot_api.power import PowerMonitor

class MappingController:
    '''
    The mapping controller node allows you to create a map using kuri's
    mapping system, OORT.

    Mapping will start once Kuri detects its on the dock.  The eyes will open
    and Kuri will look forward as an indication that mapping is started

    The user then drives Kuri using the keyboard teleop node.

    You can monitor the progress of mapping using a tool like rviz.  When the
    map reaches 20 squre meters, Kuri will start to smile.

    To stop mapping, driver Kuri back onto the dock.  Kuri's eyes will close
    and the head will go down.  The map will be saved to the user's home
    directory:

    ~/oort_maps/<uuid>/map.map_capnp      - OORT map file
    ~/oort_maps/<uuid>/map.map_meta_capnp - OORT map file
    ~/oort_maps/<uuid>/map.md5            - OORT map file checksum
    ~/oort_maps/<uuid>/map.pgm            - map_saver output (standard format)
    ~/oort_maps/<uuid>/map.yaml           - map_saver output (standard format)

    A different UUID will be used each time the mapping_controller creates a
    new map
    '''

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

        # Joint states keeps track of the latest position of all the robot's
        # joints
        self._joint_states = mobile_base.JointStates()


        self._power_monitor = None  # Created when we start to run
        self._map_manager = OortMapManager()


        self._mapping_complete = False


    def run(self):

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("mapping_controller", True)
        )

        self._power_monitor = PowerMonitor(
            dock_changed_cb=self._dock_changed_cb
        )

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
                    rospy.spin()  # Hang until shutdown
                    return
                rospy.sleep(0.5)
        except rospy.exceptions.ROSInterruptException:
            return

    def shutdown(self):
        self._joint_states.shutdown()

        if self._power_monitor:
            self._power_monitor.shutdown()

        self._map_manager.shutdown()

    def _dock_changed_cb(self, msg):
        '''
        Called when the Power Monitor detects we've moved on or off the
        dock

        This callback will start mapping when we've moved onto the dock,
        and will stop mapping when we're back on the dock with a large enough
        map
        '''

        # Early-out.  If we've already mapped, we're all done here
        if self._mapping_complete:
            return

        if msg.dock_present:
            map_state = self._map_manager.get_map_state()
            if map_state == "not_mapping":
                self._start_mapping()
            elif map_state == "mapping":
                self._stop_mapping()


    def _start_mapping(self):

        map_path = self._map_manager.start_mapping()
        # "dock" is a reserved waypoint name in OORT.  It's used for loop
        # closures as the map is added to
        self._map_manager.save_waypoint("dock", "dock")

        # This is a warning so it's easier to see in the log
        rospy.logwarn("Started mapping.  Map stored at {}".format(map_path))

    def _stop_mapping(self):



        # Notifying OORT that we're docked will give it a hint about how to do
        # the final loop closure
        self._map_manager.notify_docked()
        # Telling OORT to finish off the map is time consuming.  We'll do that
        # from the main thread
        self._mapping_complete = True