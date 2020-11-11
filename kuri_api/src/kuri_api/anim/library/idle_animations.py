import logging
import random

from kuri_api import Head
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from kuri_api.utils.social import random_adult_tilt, random_child_tilt

logger = logging.getLogger(__name__)


class IdleAnimations(AnimationGroup):
    """
    Animations for Kuri's idle behaviors.

    Currently holds scripted idle animation.
    """

    def scripted_idle(self, move_base=True):
        """
        Straight-ahead scripted idle animation (Doug v3)
        """
        tk = Track()
        add = 0
        random_side = random.choice([-1, 1])
        random_side2 = random.choice([-1, 1])
        multiplier = random.uniform(0.8, 1)
        on_off = random.randint(0, 1)
        random_range = random.randint(2, 7)
        keep_alive_speed = -1.5
        spacing = keep_alive_speed + 1
        speed = (keep_alive_speed + 10) * 0.1
        speed_amount = (-0.075 * keep_alive_speed + 10) * 0.1
        end = 16.5
        end2 = 19.5
        for _ in range(0, 5):
            for _ in range(0, random_range):
                tk.add(0.7 + add, self.head_mot.openeyes(time=0.14, amplitude=-0.35 * speed))
                tk.add(0.8 + add,
                       self.head_mot.pantilt(pan=0.2 * random_side * speed_amount, tilt=0, duration=0.7 * speed))
                tk.add(0.9 + add, self.head_mot.openeyes(time=0.25, amplitude=1.1))
                tk.add(2.8 + add, self.head_mot.pantilt(pan=0.3 * random_side * speed_amount, tilt=0.15 * speed_amount,
                                                        duration=0.15 * speed))
                tk.add(4.1 + add, self.head_mot.pantilt(pan=0.2 * random_side * speed_amount, tilt=0.15 * speed_amount,
                                                        duration=0.15 * speed))
                tk.add(4.7 + add + spacing, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
                tk.add(4.8 + add + spacing, self.head_mot.pantilt(pan=0.3 * random_side * speed_amount,
                                                                  tilt=-0.2 * random_side2 * multiplier * speed_amount,
                                                                  duration=0.53 * speed))
                tk.add(4.9 + add + spacing, self.head_mot.openeyes(time=0.25, amplitude=1.1))
                tk.add(5.8 + add + spacing, self.head_mot.pantilt(pan=0.1 * random_side * speed_amount,
                                                                  tilt=-0.2 * random_side2 * multiplier * speed_amount,
                                                                  duration=0.15 * speed))
                tk.add(7.3 + add + spacing * 2, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
                tk.add(7.38 + add + spacing * 2,
                       self.head_mot.pantilt(pan=-0.8 * random_side * speed_amount, tilt=0, duration=0.6 * speed))
                tk.add(7.48 + add + spacing * 2, self.head_mot.openeyes(time=0.25, amplitude=1.1))
                if move_base:
                    tk.add(7.7 + add + spacing * 2, self.wheels_mot.rotate(-0.8 * random_side, 1 * speed))
                tk.add(9.0 + add + spacing * 2,
                       self.head_mot.pantilt(pan=-0.6 * random_side * speed_amount, tilt=0, duration=0.15 * speed))
                tk.add(10.5 + add + spacing * 3, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
                tk.add(10.68 + add + spacing * 3, self.head_mot.openeyes(time=0.25, amplitude=1.1))
                tk.add(11.3 + add + spacing * 3,
                       self.head_mot.pantilt(pan=-0.7 * random_side * speed_amount, tilt=0, duration=0.15 * speed))
                tk.add(12.9 + add + spacing * 4, self.head_mot.openeyes(time=0.15, amplitude=-1.5 * on_off + 1))
                tk.add(13.0 + add + spacing * 4, self.head_mot.pantilt(pan=0.2 * random_side * speed_amount,
                                                                       tilt=-0.2 * random_side2 * multiplier * speed_amount,
                                                                       duration=0.5 * speed))
                tk.add(13.08 + add + spacing * 4, self.head_mot.openeyes(time=0.25, amplitude=1.1))
                if move_base:
                    tk.add(13.15 + add + spacing * 4, self.wheels_mot.rotate(0.8 * random_side, 1 * speed))
                tk.add(15 + add + spacing * 5, self.head_mot.pantilt(pan=0 * random_side, tilt=0, duration=0.4 * speed))
                tk.add(end + add + spacing * 5, self.head_mot.pantilt(pan=0 * random_side, tilt=0, duration=0.1))
                add = add + end + spacing * 5
                random_side = random.choice([-1, 1])
                random_side2 = random.choice([-1, 1])
                multiplier = random.uniform(0.5, 2)
                on_off = random.randint(0, 1)

            random_body = random.uniform(0.9, 1.2)
            tk.add(0.2 + add, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
            tk.add(0.3 + add, self.head_mot.pantilt(pan=0.2 * random_side, tilt=0.1 * random_side, duration=0.3))
            tk.add(0.4 + add, self.head_mot.openeyes(time=0.25, amplitude=1.1))
            tk.add(2 + add, self.head_mot.pantilt(pan=-0.1 * random_side, tilt=0.4 * random_side, duration=0.3 * speed))
            tk.add(4.1 + add,
                   self.head_mot.pantilt(pan=0.5 * random_side, tilt=0.4 * random_side, duration=0.4 * speed))
            tk.add(5.4 + add,
                   self.head_mot.pantilt(pan=0.3 * random_side, tilt=0.4 * random_side, duration=0.2 * speed))
            tk.add(7.6 + add, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
            tk.add(7.7 + add,
                   self.head_mot.pantilt(pan=0.9 * random_side, tilt=0.4 * random_side, duration=0.8 * speed))
            tk.add(7.78 + add, self.head_mot.openeyes(time=0.25, amplitude=1.1))
            if move_base:
                tk.add(8.0 + add, self.wheels_mot.rotate(random_body * random_side, 1.5 * speed))
            tk.add(10.7 + add + spacing,
                   self.head_mot.pantilt(pan=0.7 * random_side, tilt=0.4 * random_side, duration=0.3 * speed))
            tk.add(11.8 + add + spacing,
                   self.head_mot.pantilt(pan=0.8 * random_side, tilt=0.4 * random_side, duration=0.2 * speed))
            tk.add(12.9 + add + spacing * 2, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
            tk.add(13.0 + add + spacing * 2,
                   self.head_mot.pantilt(pan=-0.55 * random_side, tilt=0.2 * random_side, duration=0.6 * speed))
            if move_base:
                tk.add(13.2 + add + spacing * 2, self.wheels_mot.rotate(-1 * random_body * random_side, 1.5 * speed))
            tk.add(13.7 + add + spacing * 2,
                   self.head_mot.pantilt(pan=-0.2 * random_side, tilt=0.1 * random_side, duration=0.4 * speed))
            tk.add(13.3 + add + spacing * 2, self.head_mot.openeyes(time=0.15, amplitude=1.1))
            tk.add(15.9 + add + spacing * 3, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
            tk.add(18.42 + add + spacing * 3, self.head_mot.pantilt(pan=-0.4, tilt=0, duration=0.4 * speed))
            tk.add(16.08 + add + spacing * 3, self.head_mot.openeyes(time=0.15, amplitude=1.1))
            tk.add(18.42 + add + spacing * 4, self.head_mot.pantilt(pan=0, tilt=0, duration=0.4 * speed))
            tk.add(end2 + add + spacing * 5, self.head_mot.pantilt(pan=0, tilt=0, duration=0.1))
            add = add + end2 + spacing * 5
            keep_alive_speed = keep_alive_speed + 2
            if keep_alive_speed >= 4:
                keep_alive_speed = 3
            spacing = keep_alive_speed + 1
            speed = (keep_alive_speed + 10) * 0.1

        return tk

    def look_around_common(self, tilt_func=lambda: 0):
        start_pan = random.uniform(Head.PAN_RIGHT, Head.PAN_LEFT)
        start_tilt = tilt_func()
        CENTER_THRESHOLD = 0.25
        pan_2 = Head.PAN_RIGHT * random.uniform(0.5, 1.0)
        tilt_2 = tilt_func()
        pan_3 = Head.PAN_LEFT * random.uniform(0.5, 1.0)
        tilt_3 = tilt_func()
        if start_pan > Head.PAN_LEFT * CENTER_THRESHOLD:
            pan_2 = Head.PAN_NEUTRAL
            pan_3 = Head.PAN_RIGHT * random.uniform(0.5, 1.0)
        else:
            if start_pan < Head.PAN_RIGHT * CENTER_THRESHOLD:
                pan_2 = Head.PAN_NEUTRAL
                pan_3 = Head.PAN_LEFT * random.uniform(0.5, 1.0)
        tk = Track()
        tk.add(1.5, self.head_mot.pantilt(pan=start_pan, tilt=start_tilt, duration=0.45))
        tk.add(1.6, self.head_mot.blinkeyes())
        tk.add(4.5, self.head_mot.pantilt(pan=pan_2, tilt=tilt_2, duration=0.45))
        tk.add(4.7, self.head_mot.blinkeyes())
        tk.add(7.5, self.head_mot.pantilt(pan=pan_3, tilt=tilt_3, duration=0.45))
        tk.add(7.6, self.head_mot.blinkeyes())
        tk.add(9.2, self.head_mot.blinkeyes())
        return tk

    def look_around_adults(self):
        """
        Look around for adult sized objects.
        Length: 10 seconds
        """
        return self.look_around_common(random_adult_tilt)

    def look_around_kids_pets(self):
        """
        Look around for kid sized objects, such as dogs!
        Length: 10 seconds
        """
        return self.look_around_common(random_child_tilt)
