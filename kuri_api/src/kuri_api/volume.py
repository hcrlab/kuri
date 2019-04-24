import os.path, rospy
from kuri_api.msg import Volume as VolMsg
from std_msgs.msg import Bool, Empty
import logging
sounds_path = '/opt/gizmo/share/assets/sounds'
logger = logging.getLogger(__name__)

class Volume(object):
    """
    A thin wrapper that is in charge of delegating requested volume
    changes to the volume_interface and that listens for HW button presses
    to play the volume sound.
    """
    VOLUME_NAMESPACE = 'volume'

    def __init__(self, sound_src):
        self._sound_src = sound_src
        self._pub = rospy.Publisher(self.VOLUME_NAMESPACE + '/app_set', VolMsg, queue_size=1)
        self._ducking_pub = rospy.Publisher(self.VOLUME_NAMESPACE + '/duck_set', Bool, queue_size=1)
        self._store_volume_pub = rospy.Publisher(self.VOLUME_NAMESPACE + '/save', Empty, queue_size=1)
        self._vol_button_sub = rospy.Subscriber(self.VOLUME_NAMESPACE + '/hw_set', VolMsg, self._vol_hw_cb)

    def shutdown(self):
        if self._vol_button_sub:
            self._vol_button_sub.unregister()
            self._vol_button_sub = None
        return

    def set_volume(self, level, relative=False):
        vol_change = VolMsg(level=level, is_relative=relative, mute=False, unmute=False)
        self._pub.publish(vol_change)

    def set_ducking(self, should_duck):
        self._ducking_pub.publish(Bool(should_duck))

    def store_volume(self):
        self._store_volume_pub.publish(Empty())

    def mute(self):
        mute = VolMsg(level=0, is_relative=False, mute=True, unmute=False)
        self._pub.publish(mute)

    def unmute(self):
        unmute = VolMsg(level=0, is_relative=False, mute=False, unmute=True)
        self._pub.publish(unmute)

    def _vol_hw_cb(self, msg):
        """
        The hardware buttons have been pressed - update levels and beep.
        """
        self._play_volume_beep(msg.level)

    def _play_volume_beep(self, level):
        """
        Plays the volume beep sound.
        """
        beep = os.path.join(sounds_path, 'Volume.wav')
        if level >= 100:
            beep = os.path.join(sounds_path, 'S97_VOLUME_CHANGE_MAX.wav')
        self._sound_src.play(beep)