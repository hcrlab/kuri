from math import radians

from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class TestAnimations(AnimationGroup):

    def seizure(self):
        """
        Actuate joints through range of motions
        """
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(0.8, 0.6, 0.5))
        tk.add(0.01, self.head_mot.moveeyes(0.0, 0.5))
        tk.add(0.5, self.head_mot.pantilt(-0.8, -0.6, 0.5))
        tk.add(0.51, self.head_mot.moveeyes(0.4, 0.5))
        tk.add(3.0, self.lights_mot.white_glow(255, 0.1, float('inf')))
        return tk

    def test_pan(self):
        """
        Move pan axis independently.
        """
        tk = Track()
        tk.add(1.41, self.head_mot.neutral())
        tk.add(3.1, self.head_mot.lookat(radians(90), 0.35))
        tk.add(5.1, self.head_mot.lookat(radians(0), 0.35))
        tk.add(7.1, self.head_mot.lookat(radians(90), 0.35))
        return tk

    def test_tilt(self):
        """
        Move tilt axis independently.
        """
        tk = Track()
        tk.add(1.41, self.head_mot.neutral())
        tk.add(3.1, self.head_mot.pantilt(0, radians(40), 0.35))
        tk.add(5.1, self.head_mot.pantilt(0, radians(0), 0.35))
        tk.add(7.1, self.head_mot.pantilt(0, radians(40), 0.35))
        return tk

    def test_eyes(self):
        """
        Move eyelids independently.
        """
        tk = Track()
        tk.add(3.1, self.head_mot.moveeyes(radians(30), 0.35))
        tk.add(5.1, self.head_mot.moveeyes(radians(0), 0.35))
        tk.add(7.1, self.head_mot.moveeyes(radians(30), 0.35))
        return tk
