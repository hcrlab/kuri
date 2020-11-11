from random import choice

from kuri_api import Head
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class SleepAndWakeAnimations(AnimationGroup):

    def twitch_1(self):
        random_side = choice([-1, 1])
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.4, duration=0.3))
        tk.add(0.1, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(0.4, self.head_mot.pantilt(pan=-0.2 * random_side, tilt=Head.TILT_DOWN, duration=0.1))
        tk.add(0.75, self.head_mot.pantilt(pan=0.2 * random_side, tilt=0.45, duration=0.2))
        tk.add(1.0, self.head_mot.pantilt(pan=-0.2 * random_side, tilt=Head.TILT_DOWN, duration=0.2))
        tk.add(1.25, self.head_mot.pantilt(pan=0.4 * random_side, tilt=0.4, duration=0.1))
        tk.add(1.4, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=1.1))
        return tk

    def twitch_2(self):
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.2))
        tk.add(0.1, self.head_mot.moveeyes(0.25, 0.4))
        tk.add(0.15, self.head_mot.pantilt(pan=0.3, tilt=0.35, duration=0.6))
        tk.add(1.7, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(1.94, self.head_mot.moveeyes(0.26, 0.6))
        tk.add(2.86, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(2.45, self.head_mot.pantilt(pan=0.25, tilt=0.4, duration=0.2))
        tk.add(2.9, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.45, duration=0.4))
        tk.add(3.4, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.2))
        return tk

    def twitch_3(self):
        random_side = choice([-1, 1])
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.1))
        tk.add(0.1, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(0.15, self.head_mot.pantilt(pan=0.5 * random_side, tilt=Head.TILT_DOWN, duration=1.1))
        tk.add(1.45, self.head_mot.pantilt(pan=-0.2 * random_side, tilt=0.45, duration=0.2))
        tk.add(1.9, self.head_mot.pantilt(pan=0.2 * random_side, tilt=Head.TILT_DOWN, duration=0.3))
        tk.add(2.15, self.head_mot.pantilt(pan=-0.2 * random_side, tilt=Head.TILT_DOWN, duration=0.3))
        tk.add(2.4, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.1))
        return tk

    def twitch_4(self):
        random_side = choice([-1, 1])
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.3))
        tk.add(0.1, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(0.85, self.head_mot.pantilt(pan=0.2 * random_side, tilt=0.45, duration=0.25))
        tk.add(1.9, self.head_mot.pantilt(pan=-0.2 * random_side, tilt=Head.TILT_DOWN, duration=0.3))
        tk.add(2.15, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.1, duration=0.3))
        tk.add(2.2, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.2))
        tk.add(2.55, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.35, duration=1.0))
        tk.add(2.7, self.head_mot.moveeyes(0.3, 1.0))
        tk.add(5.25, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.12))
        tk.add(5.4, self.head_mot.moveeyes(0.25, 0.22))
        tk.add(5.85, self.head_mot.moveeyes(0.32, 0.12))
        tk.add(5.9, self.head_mot.pantilt(pan=-0.4 * random_side, tilt=0.4, duration=0.3))
        tk.add(6.0, self.head_mot.moveeyes(0.35, 0.32))
        tk.add(7.05, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.12))
        tk.add(7.1, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.55))
        tk.add(7.2, self.head_mot.moveeyes(0.25, 0.32))
        tk.add(7.75, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.12))
        tk.add(7.8, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.2))
        return tk

    def wakeup_auto(self):
        tk = Track()
        tk.add(0, self.sound_mot.open('Wake.wav'))
        tk.add(0, self.head_mot.openeyes())
        tk.add(0.5, self.head_mot.blinkeyes())
        tk.add(0.5, self.head_mot.neutral())
        tk.add(0.9, self.head_mot.blinkeyes())
        look_right = choice([1, -1])
        tk.add(1.0, self.head_mot.lookat(look_right * 0.8, 0.5))
        tk.add(1.5, self.head_mot.blinkeyes())
        tk.add(1.6, self.head_mot.lookat(look_right * -0.7, 0.7))
        tk.add(2.0, self.head_mot.blinkeyes())
        tk.add(2.65, self.head_mot.blinkeyes())
        tk.add(2.7, self.head_mot.neutral())
        return tk

    def wakeup_fast(self):
        tk = Track()
        tk.add(0.0, self.lights_mot.off(length=1.0))
        tk.add(0.5, self.sound_mot.open('Wake.wav'))
        tk.add(0.5, self.head_mot.neutral())
        tk.add(0.8, self.head_mot.openeyes())
        return tk
