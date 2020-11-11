import random
from math import radians

from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from kuri_api.utils.social import random_photo_tilt


class SocialAnimations(AnimationGroup):
    """
    Animations for interacting with people.
    """

    def face_detected(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyposture())
        return tk

    def face_lost(self):
        tk = Track()
        tk.add(0.0, self.head_mot.sadposture())
        tk.add(0.7, self.head_mot.neutral())
        tk.add(0.7, self.head_mot.openeyes())
        return tk

    def reposition(self):
        """
        Basic animation for repositioning during a photoshoot.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.blinkeyes())
        if random.random() < 0.65:
            tk.add(0.35, self.head_mot.blinkeyes())
        tilt = random_photo_tilt()
        init_pan = self.head.cur_pan
        PAN_CUTOFF = 0.5
        if init_pan > PAN_CUTOFF:
            pan_dir = -1
        else:
            if init_pan < -1 * PAN_CUTOFF:
                pan_dir = 1
            else:
                pan_dir = random.choice([1, -1])
        rand_pan = init_pan + radians(pan_dir * random.uniform(15, 45))
        tk.add(0.35, self.head_mot.pantilt(rand_pan, tilt, 0.33))
        return tk
