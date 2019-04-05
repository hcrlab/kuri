import rospy

from kuri_api.utils import Events
import mobile_base_driver.msg

import logging, rospy
from numpy import clip
import mobile_base_driver.msg

logger = logging.getLogger(__name__)


class PowerMonitor(Events):
    """
    Interface to docking/power management behaviors

    Events:
    docked_event - Event indicating the robot has connected to the dock
    undocked_event - Event indicating the robot has disconnected from the dock
    critical_power_event - Event indicating that power is at a critical level
    """
    DEFAULT_POWER_TOPIC = 'mobile_base/power'
    DEFAULT_LOW_BATTERY_PERCENTAGE = 30
    DEFAULT_CRITICAL_BATTERY_PERCENTAGE = 16
    UNKNOWN_LEVEL = 0
    docked_event = Events.source()
    undocked_event = Events.source()
    critical_power_event = Events.source()

    def __init__(self, power_anims=None, low_power=None, critical_power=None):
        super(PowerMonitor, self).__init__()
        self.low_power = low_power or self.DEFAULT_LOW_BATTERY_PERCENTAGE
        self.critical_power = critical_power or self.DEFAULT_CRITICAL_BATTERY_PERCENTAGE
        self._anim = power_anims
        self._is_docked = False
        self._is_critical = False
        self._is_low = False
        self._power_level = 0
        self._last_docked_time = 0
        self.dock_sub = rospy.Subscriber(self.DEFAULT_POWER_TOPIC, mobile_base_driver.msg.Power, self._power_cb)

    def is_docked(self):
        return self._is_docked

    def power_level(self):
        if self._power_level:
            return self._power_level
        return self.UNKNOWN_LEVEL

    def is_critical(self):
        return self._is_critical

    def is_low(self):
        return self._is_low

    def get_last_docked_time(self):
        return self._last_docked_time

    def _power_cb(self, msg):
        """
        Tracks the dock state and triggers an event when we
        transition from not being on the dock to being on the dock.
        It also will fire a 'battery_low' event when the power level
        drops to low. This will be used start docking behaviors.
        :param msg:
        :return:
        """
        last_power_level = self._power_level
        self._power_level = clip(msg.battery.rounded_pct, 0, 100)
        dock_changed = self._check_docking_events(msg)
        self._check_for_low_power(msg.battery)
        self._check_for_critical_power(msg.battery)
        if dock_changed or last_power_level != self._power_level:
            #self._status_srv.log_power(self.power_status(), self._power_level)
            pass

    def _check_docking_events(self, msg):
        """
        Determines if a docking event has occurred.

        Updates the internal state of the service, emits a docked or undocked
        event when the docked state changes, and returns a bool indicating
        whether a docking event was triggered.
        """
        if not self._is_docked and msg.dock_present:
            self._is_docked = True
            self._last_docked_time = rospy.get_time()
            self.docked_event('docked')
            logger.info('docked')
            #self._anim.cancel()
            return True
        if self._is_docked and not msg.dock_present:
            self._is_docked = False
            self.undocked_event('undocked')
            if self._is_critical and not self._anim.is_playing:
                pass#self._anim.critical_battery()
            return True
        return False

    def _check_for_critical_power(self, battery_msg):
        """
        Determines if the battery is at a critical power level.

        Updates the internal state of the service, emits an event on the first
        drop below critical power level, and returns a bool indicating
        whether the robot's power level is critical.

        This implicitly relies on the power level not going up off the dock

        Does not emit an event if docked.
        """
        if not self._is_critical and battery_msg.rounded_pct < self.critical_power:
            self._is_critical = True
            if not self._is_docked:
                #self._anim.critical_battery()
                self.critical_power_event('battery_critical')
        if self._is_critical and battery_msg.rounded_pct > self.critical_power:
            self._is_critical = False
            pass#self._anim.cancel()
        return self._is_critical

    def _check_for_low_power(self, battery_msg):
        """
        Determines if the battery is at a low power level.

        Updates the internal state of the service, returns a bool indicating
        whether the robot's power level is low.

        This implicitly relies on the power level not going up off the dock,
        """
        if not self._is_low and battery_msg.rounded_pct < self.low_power:
            self._is_low = True
        if self._is_low and battery_msg.rounded_pct > self.low_power:
            self._is_low = False
        return self._is_low

    def shutdown(self):
        if self.dock_sub:
            self.dock_sub.unregister()