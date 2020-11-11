from random import normalvariate

from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class BlinkAnimations(AnimationGroup):
    """
    Blink animations.
    """
    PAUSE_MIN = 0.3
    PAUSE_MAX = 0.4
    PAUSE_MEAN = 0.34
    PAUSE_STD_DEV = 0.03

    def blink(self):
        tk = Track()
        tk.add(0.0, self.head_mot.blink())
        return tk

    def double_blink(self):
        blink_time = max(self.PAUSE_MIN, min(self.PAUSE_MAX, normalvariate(self.PAUSE_MEAN, self.PAUSE_STD_DEV)))
        tk = Track()
        tk.add(0.0, self.head_mot.blink())
        tk.add(blink_time, self.head_mot.blink())
        return tk

    def triple_blink(self):
        tk = Track()
        blink_time_1 = max(self.PAUSE_MIN, min(self.PAUSE_MAX, normalvariate(self.PAUSE_MEAN, self.PAUSE_STD_DEV)))
        blink_time_2 = max(self.PAUSE_MIN, min(self.PAUSE_MAX, normalvariate(self.PAUSE_MEAN, self.PAUSE_STD_DEV)))
        tk.add(0.0, self.head_mot.blink())
        tk.add(blink_time_1, self.head_mot.blink())
        tk.add(blink_time_1 + blink_time_2, self.head_mot.blink())
        return tk
