import math
import random

from kuri_api import Head
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track


class OnboardingAnimations(AnimationGroup):

    def greeting_first_capture(self):
        """
        A short smile and Kuri's iconic greeting sound.
        Length: 2 seconds
        """
        tk = Track()
        tk.add(0.0, self.head_mot.happyeyes())
        tk.add(0.0, self.sound_mot.open('Greeting.wav'))
        tk.add(2.0, self.head_mot.openeyes())
        return tk

    def search_first_capture(self):
        """
        Search for a person during first moment capture.
        0 - 4 seconds - look up and straight ahead
        4 - 8 seconds - look up and left
        8 - 12 seconds - look up and right
        12 - 14 seconds - turn left 45 degrees
        14 - 20 seconds - look around with head
        20 - 22.5 seconds - turn right 90 degrees
        22.5 - 28 seconds - look around with head
        28 - 30 seconds - look up, reset orientation with dock behind robot
        Length: 30 seconds
        """
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_UP * 0.75, duration=0.45))
        tk.add(0.1, self.head_mot.blinkeyes())
        tk.add(2.8, self.head_mot.blinkeyes())
        tk.add(4.0, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.8, tilt=Head.TILT_UP * 0.7, duration=0.4))
        tk.add(4.1, self.head_mot.blinkeyes())
        tk.add(6.9, self.head_mot.blinkeyes())
        tk.add(8.0, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.7, tilt=Head.TILT_UP * 0.6, duration=0.5))
        tk.add(8.0, self.head_mot.blinkeyes())
        tk.add(11.7, self.head_mot.blinkeyes())
        turn_angle_radians = 0.79
        tilt_radians = Head.TILT_UP * 0.65
        turn_start = 12.0
        time_to_turn = 2.0
        if random.random() < 0.85:
            tk.add(turn_start + 0.2, self.head_mot.blinkeyes())
        turn_start_time = turn_start + 0.2
        tk.add(turn_start, self.head_mot.pantilt(pan=turn_angle_radians, tilt=tilt_radians, duration=0.45))
        tk.add(turn_start_time, self.wheels_mot.rotate_by(angle=turn_angle_radians, duration=time_to_turn))
        tk.add(turn_start_time + time_to_turn - 0.5, self.head_mot.pantilt(pan=0, tilt=tilt_radians, duration=0.5))
        if random.random() < 0.65:
            tk.add(turn_start_time + time_to_turn + 0.4, self.head_mot.blinkeyes())
        tk.add(16.0, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.75, tilt=Head.TILT_UP * 0.8, duration=0.35))
        tk.add(16.2, self.head_mot.blinkeyes())
        tk.add(17.7, self.head_mot.blinkeyes())
        tk.add(18.0, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.9, tilt=Head.TILT_UP * 0.72, duration=0.45))
        tk.add(19.5, self.head_mot.blinkeyes())
        turn_start = 20.0
        turn_angle_radians = -1.58
        time_to_turn = 2.5
        if random.random() < 0.45:
            tk.add(turn_start + 0.3, self.head_mot.blinkeyes())
        turn_start_time = turn_start + 0.2
        tk.add(turn_start, self.head_mot.pantilt(pan=turn_angle_radians, tilt=tilt_radians, duration=0.45))
        tk.add(turn_start_time, self.wheels_mot.rotate_by(angle=turn_angle_radians, duration=time_to_turn))
        tk.add(turn_start_time + time_to_turn - 0.5, self.head_mot.pantilt(pan=0, tilt=tilt_radians, duration=0.5))
        if random.random() < 0.75:
            tk.add(turn_start_time + time_to_turn + 0.3, self.head_mot.blinkeyes())
        tk.add(24.5, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.55, tilt=Head.TILT_UP * 0.6, duration=0.3))
        tk.add(24.6, self.head_mot.blinkeyes())
        tk.add(26.2, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.7, tilt=Head.TILT_UP * 0.4, duration=0.25))
        turn_angle_radians = 0.79
        tilt_radians = Head.TILT_UP * 0.78
        turn_start = 28.0
        time_to_turn = 2.0
        tk.add(turn_start + 0.5, self.head_mot.blinkeyes())
        turn_start_time = turn_start + 0.2
        tk.add(turn_start, self.head_mot.pantilt(pan=turn_angle_radians, tilt=tilt_radians, duration=0.45))
        tk.add(turn_start_time, self.wheels_mot.rotate_by(angle=turn_angle_radians, duration=time_to_turn))
        tk.add(turn_start_time + time_to_turn - 0.5, self.head_mot.pantilt(pan=0, tilt=tilt_radians, duration=0.5))
        return tk

    def undock_and_scan(self):
        """
        Drives forward off of dock, spins 90 degrees to one direction and then
        180 degrees in the opposite direction.
        Length: 20.8 seconds
        """
        random_pan_side = random.choice([-1, 1])
        sound_indeces = [1, 2]
        first_sound_index = random.choice(sound_indeces)
        sound1 = 'Humm' + str(first_sound_index) + '.wav'
        sound_indeces.remove(first_sound_index)
        sound2 = 'Humm' + str(sound_indeces[0]) + '.wav'
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN * 0.8, duration=0.3))
        tk.add(0.0, self.wheels_mot.inch(0.275, 6.5))
        tk.add(0.3, self.head_mot.blinkeyes())
        tk.add(0.8, self.head_mot.pantilt(pan=0.2 * random_pan_side, tilt=Head.TILT_DOWN * 0.5, duration=0.3))
        tk.add(0.9, self.head_mot.blinkeyes())
        tk.add(1.4, self.head_mot.pantilt(pan=-0.25 * random_pan_side, tilt=Head.TILT_DOWN * 0.2, duration=0.45))
        tk.add(2.0, self.head_mot.pantilt(pan=0.1 * random_pan_side, tilt=Head.TILT_DOWN * 0.6, duration=0.45))
        tk.add(2.7, self.head_mot.pantilt(pan=-0.15 * random_pan_side, tilt=Head.TILT_DOWN * 0.4, duration=0.1))
        tk.add(3.3, self.head_mot.pantilt(pan=-0.25 * random_pan_side, tilt=Head.TILT_DOWN * 0.2, duration=0.45))
        tk.add(3.4, self.head_mot.blinkeyes())
        tk.add(3.9, self.head_mot.pantilt(pan=0.1 * random_pan_side, tilt=Head.TILT_DOWN * 0.4, duration=0.45))
        tk.add(4.4, self.head_mot.pantilt(pan=-0.1 * random_pan_side, tilt=Head.TILT_DOWN * 0.6, duration=0.1))
        tk.add(5.2, self.head_mot.pantilt(pan=0.25 * random_pan_side, tilt=Head.TILT_DOWN * 0.1, duration=0.45))
        tk.add(6.4, self.head_mot.blinkeyes())
        tk.add(6.5, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.7 * random_pan_side, tilt=Head.TILT_DOWN * 0.6,
                                          duration=0.55))
        tk.add(7.0, self.sound_mot.open(sound1))
        tk.add(7.0, self.wheels_mot.rotate_by(angle=math.pi / 2 * random_pan_side, duration=3.0))
        tk.add(7.6, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.55 * random_pan_side, tilt=Head.TILT_DOWN * 0.1,
                                          duration=0.4))
        tk.add(8.7, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.75 * random_pan_side, tilt=Head.TILT_DOWN * 0.6,
                                          duration=0.4))
        tk.add(8.8, self.head_mot.blinkeyes())
        tk.add(9.6, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.45 * random_pan_side, tilt=Head.TILT_DOWN * 0.3,
                                          duration=0.4))
        tk.add(9.7, self.head_mot.blinkeyes())
        tk.add(9.9, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN * 0.6, duration=0.3))
        tk.add(10.5, self.sound_mot.open(sound2))
        tk.add(10.5, self.head_mot.blinkeyes())
        tk.add(10.5, self.wheels_mot.rotate_by(angle=-math.pi * random_pan_side, duration=6.0))
        tk.add(10.7, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.7 * random_pan_side, tilt=Head.TILT_DOWN * 0.15,
                                           duration=0.4))
        tk.add(11.3, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.3 * random_pan_side, tilt=Head.TILT_DOWN * 0.6,
                                           duration=0.55))
        tk.add(12.0, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.55 * random_pan_side, tilt=Head.TILT_DOWN * 0.1,
                                           duration=0.4))
        tk.add(12.1, self.head_mot.blinkeyes())
        tk.add(12.7, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.75 * random_pan_side, tilt=Head.TILT_DOWN * 0.6,
                                           duration=0.5))
        tk.add(14.2, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.45 * random_pan_side, tilt=Head.TILT_DOWN * 0.7,
                                           duration=0.5))
        tk.add(14.7, self.head_mot.blinkeyes())
        tk.add(15.3, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.8 * random_pan_side, tilt=Head.TILT_DOWN * 0.1,
                                           duration=0.3))
        tk.add(16.0, self.head_mot.blinkeyes())
        tk.add(16.1, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.5 * random_pan_side, tilt=Head.TILT_DOWN * 0.6,
                                           duration=0.55))
        tk.add(16.6, self.wheels_mot.rotate_by(angle=math.pi / 2 * random_pan_side, duration=3.0))
        tk.add(17.2, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.75 * random_pan_side, tilt=Head.TILT_DOWN * 0.1,
                                           duration=0.4))
        tk.add(18.3, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.55 * random_pan_side, tilt=Head.TILT_DOWN * 0.3,
                                           duration=0.4))
        tk.add(18.4, self.head_mot.blinkeyes())
        tk.add(19.2, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.35 * random_pan_side, tilt=Head.TILT_DOWN * 0.7,
                                           duration=0.4))
        tk.add(19.3, self.head_mot.blinkeyes())
        tk.add(19.5, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN * 0.3, duration=0.3))
        tk.add(20.0, self.head_mot.openeyes())
        tk.add(20.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.8))
        return tk
