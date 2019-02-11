import threading

import geometry_msgs.msg
import rospy

import mayfield_msgs.msg
import mayfield_utils
import mobile_base


class SafetyController:

    # A set of the safety events we're equipped to handle.
    # It's just bumpers so we don't need to deal with compexities
    # like "What happens if a front bump and a rear cliff triggers
    # at the same time"
    HANDLED_EVENTS = {'BPR_bp', 'BPM_bp', 'BPL_bp'}
    SAFETY_HZ = 10

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

        self._safety_client = mobile_base.SafetyClient()
        # Override the safety controller for all events that we don't
        # handle so they'll be ignored by the hardware
        self.UNHANDLED_EVENTS = (
            self._safety_client.safety_statuses()  # All statuses
            - self.HANDLED_EVENTS                  # Minus the ones we handle
        )
        self._safety_client.safety_override(self.UNHANDLED_EVENTS)

        self._sync_lock = threading.Lock()
        self._block_twists = False

        # Set up publishers before subscribers because once subscribers are
        # set up, we can start getting callbacks
        self._cmd_vel_pub = rospy.Publisher(
            "mobile_base/commands/velocity",
            geometry_msgs.msg.Twist,
            queue_size=1
        )

        self._cmd_vel_sub = rospy.Subscriber(
            "cmd_vel",
            geometry_msgs.msg.Twist,
            self._forward_twists
        )

        # May nav comes in on a different topic.  A move advanced robot may
        # want to handle nav commands differently, but for this simple robot,
        # we'll treat them the same as keyboard teleop twist messages
        self._nav_vel_sub = rospy.Subscriber(
            "nav_cmd_vel",
            geometry_msgs.msg.Twist,
            self._forward_twists
        )

    def run(self):

        # Indicate to the world that we're running and ready to go:
        self._pub.publish(
            mayfield_msgs.msg.NodeStatus("safety_controller", True)
        )

        rate = rospy.Rate(self.SAFETY_HZ)  # 10hz by default
        while not rospy.is_shutdown():
            try:

                current_status = self._safety_client.get_safety_status()

                if self.HANDLED_EVENTS.intersection(current_status):
                    # Lock is necessary to make sure the _forward_twists
                    # method is not in between checking the _block_twists bool
                    # and sending out another twist.  Once we've set
                    # _block_twists, the _forward_twists method won't race
                    # anymore.
                    with self._sync_lock:
                        self._block_twists = True

                    # Just in case a forward twist is still in the out queue:
                    self._stop()

                    # Override the safety status and back the robot up
                    self._safety_client.safety_override(current_status)
                    self._back_up(rate)
                    self._safety_client.safety_override(self.UNHANDLED_EVENTS)
                    self._safety_client.safety_clear(current_status)

                    # There's no race with _forward_twists when re-enabling
                    # this
                    self._block_twists = False

                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                return

    def shutdown(self):
        self._safety_client.shutdown()

    def _back_up(self, rate):
        '''
        Backs the robot up for ~ half a second
        '''
        for _ in range(self.SAFETY_HZ / 2):
            self._cmd_vel_pub.publish(
                geometry_msgs.msg.Twist(
                    linear=geometry_msgs.msg.Vector3(-0.1, 0, 0),
                    angular=geometry_msgs.msg.Vector3(0, 0, 0)
                )
            )
            rate.sleep()
        self._stop()

    def _forward_twists(self, msg):
        '''
        This callback listens for twist messages come on standard topics.
        If we're not handling a safety event, we forward the message on
        to the wheels.  If we're currently handling a safety event, then
        we block the message from going to the wheels
        '''
        with self._sync_lock:
            if not self._block_twists:
                self._cmd_vel_pub.publish(msg)

    def _stop(self):
        '''
        Sends a zero velocity twist to the mobile base driver
        '''
        self._cmd_vel_pub.publish(
            geometry_msgs.msg.Twist(
                linear=geometry_msgs.msg.Vector3(0, 0, 0),
                angular=geometry_msgs.msg.Vector3(0, 0, 0)
            )
        )
        # We don't have a closed-loop way of figuring out when the zero
        # velocity has made it down to the wheels, so we'll open-loop
        # wait here:
        rospy.sleep(0.5)
