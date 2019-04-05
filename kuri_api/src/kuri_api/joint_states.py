import threading, rospy
from sensor_msgs.msg import JointState

class JointStates(object):
    """
    JointStates keeps track of the latest position of the robot's eyes,
    head pan, head tilt, and wheels
    """
    EYE_CLOSE_THRESHOLD = 0.075
    EYE_OPEN_THRESHOLD = 0.04

    def __init__(self):
        self._lock = threading.RLock()
        self._eye_pos = 0.0
        self._tilt_pos = 0.0
        self._pan_pos = 0.0
        self._left_wheel_pos = 0.0
        self._right_wheel_pos = 0.0
        self._sub = rospy.Subscriber('joint_states', JointState, self.update)

    def shutdown(self):
        if self._sub:
            self._sub.unregister()
            self._sub = None
        return

    def get_eye_pos(self):
        with self._lock:
            return self._eye_pos

    def get_pan_pos(self):
        with self._lock:
            return self._pan_pos

    def get_tilt_pos(self):
        with self._lock:
            return self._tilt_pos

    def get_head_pos(self):
        with self._lock:
            return (self.get_pan_pos(), self.get_tilt_pos(), self.get_eye_pos())

    def get_wheel_pos(self):
        with self._lock:
            return (self._left_wheel_pos, self._right_wheel_pos)

    def update(self, joint_state):
        with self._lock:
            joint_pos = dict(((joint, angle) for joint, angle in zip(joint_state.name, joint_state.position)))
            if not self._is_identical(joint_pos):
                self._right_wheel_pos = joint_pos['wheel_right_joint']
                self._left_wheel_pos = joint_pos['wheel_left_joint']
                self._pan_pos = joint_pos['head_1_joint']
                self._tilt_pos = joint_pos['head_2_joint']
                self._eye_pos = joint_pos['eyelids_joint']

    def _is_identical(self, joint_pos):
        return self._right_wheel_pos == joint_pos['wheel_right_joint'] and self._left_wheel_pos == joint_pos['wheel_left_joint'] and self._pan_pos == joint_pos['head_1_joint'] and self._tilt_pos == joint_pos['head_2_joint'] and self._eye_pos == joint_pos['eyelids_joint']