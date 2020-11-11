import logging
import rospy
import threading

logger = logging.getLogger(__name__)


class Timeout(threading.Thread):
    """
    Cancel able timeout thread (not available in rospy)
    This is to avoid dangling threads
    """
    DEFAULT_RATE = 10

    def __init__(self, duration=1.0, timeout_cb=None):
        super(Timeout, self).__init__(name='Timeout')
        self.start_time = rospy.get_time()
        self._duration = duration
        self.timeout_cb = timeout_cb
        self._shutdown = False
        self.start()

    def run(self):
        r = rospy.Rate(self.DEFAULT_RATE)
        while not rospy.is_shutdown() and rospy.get_time() - self.start_time < self._duration and not self._shutdown:
            try:
                r.sleep()
            except rospy.exceptions.ROSInterruptException:
                pass

        if not self._shutdown and not rospy.is_shutdown():
            self.timeout_cb()

    def shutdown(self, blocking=True):
        self._shutdown = True
        if blocking:
            try:
                self.join()
            except RuntimeError:
                pass


class EventTimeout(Timeout):
    """
    If no event is recieved for some duration a timeout occurs
    """

    def __init__(self, duration=1.0, timeout_cb=None):
        super(EventTimeout, self).__init__(duration=duration, timeout_cb=timeout_cb)
        self.name = 'EventTimeout'

    def notify_of_event(self):
        self.start_time = rospy.get_time()


class TaskTimeout(object):
    """
    Convenience class to insert into the constructor of state machines/states
    to enforce timeouts.
    """

    def __init__(self, interface, duration, callback):
        """
        :params interface: gizmo Context.
        :params duration: wait before timing out in seconds.
        :params callback: function to call once the task time out.
        """
        self._timeout = EventTimeout(duration=duration, timeout_cb=callback)
        self._events = [
            interface.sensors.kidnapped_event,
            interface.sensors.pickup_event,
            interface.sensors.putdown_event,
            interface.touch.touch_event,
            interface.power.docked_event,
            interface.nav_client.nav_event,
            interface.wheels_mux.teleop.move_event,
            interface.head_ctl.move_event,
            interface.comm_interface.command_event,
            interface.voice.wake_event]
        for event in self._events:
            event.connect(self._reset_timeout)

    def _reset_timeout(self, msg):
        self._timeout.notify_of_event()

    def shutdown(self):
        for event in self._events:
            event.disconnect(self._reset_timeout)

        self._timeout.shutdown()
