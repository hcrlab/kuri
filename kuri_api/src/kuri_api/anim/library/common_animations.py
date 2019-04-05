from math import radians, pi
from random import choice, random
from numpy import clip
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from kuri_api.head import Head
import logging
logger = logging.getLogger(__name__)

class CommonAnimations(AnimationGroup):

    def no(self):
        """
        Perform a no animation (maintaining tilt orientation),
        and return the head to the previous pan location.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.3, self.sound_mot.open('No.wav'))
        init_pan = self.head.cur_pan
        init_tilt = self.head.cur_tilt
        init_direction = choice([-1, 1])
        do_extra_shake = False
        PAN_CUTOFF = 0.5
        if init_pan > PAN_CUTOFF:
            init_direction = -1
            do_extra_shake = True
        else:
            if init_pan < PAN_CUTOFF * -1:
                init_direction = 1
                do_extra_shake = True
        tk.add(0.0, self.head_mot.pantilt(clip(init_pan + init_direction * 0.15, Head.PAN_RIGHT, Head.PAN_LEFT), init_tilt, 0.2))
        tk.add(0.3, self.head_mot.pantilt(clip(init_pan + init_direction * -0.3, Head.PAN_RIGHT, Head.PAN_LEFT), init_tilt, 0.3))
        if do_extra_shake:
            tk.add(0.6, self.head_mot.pantilt(clip(init_pan + init_direction * 0.3, Head.PAN_RIGHT, Head.PAN_LEFT), init_tilt, 0.3))
            tk.add(0.9, self.head_mot.pantilt(clip(init_pan + init_direction * -0.3, Head.PAN_RIGHT, Head.PAN_LEFT), init_tilt, 0.3))
            tk.add(1.3, self.head_mot.pantilt(init_pan, init_tilt, 0.3))
        else:
            tk.add(0.7, self.head_mot.pantilt(init_pan, init_tilt, 0.3))
        return tk

    def reset_head(self, lights_fade_duration=None):
        tk = Track()
        tk.add(0.0, self.head_mot.reset())
        if lights_fade_duration:
            tk.add(0, self.lights_mot.off(lights_fade_duration))
        tk.add(0.4, self.head_mot.openeyes())
        return tk

    def sad(self):
        tk = Track()
        tk.add(0.0, self.head_mot.sadposture())
        tk.add(3.0, self.head_mot.reset())
        return tk

    def tap_head_find_face(self):
        tk = Track()
        tk.add(0.0, self.head_mot.moveeyes(-0.07))
        return tk

    def smile(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyeyes())
        tk.add(3.0, self.head_mot.openeyes())
        return tk

    def greeting_face_no_sound(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyeyes())
        tk.add(3.0, self.head_mot.openeyes())
        return tk

    def greeting_face_sound(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyeyes())
        tk.add(0.0, self.sound_mot.open('Greeting2.wav'))
        tk.add(3.0, self.head_mot.openeyes())
        return tk

    def attention_look_around_3(self):
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        if random() < 0.65:
            tk.add(0.2, self.head_mot.blinkeyes())
        tk.add(0.0, self.head_mot.pantilt(pan=0.5, tilt=-0.5, duration=0.45))
        tk.add(0.2, self.wheels_mot.rotate_by(angle=0.5, duration=0.7))
        tk.add(0.5, self.head_mot.pantilt(pan=0, tilt=-0.55, duration=0.4))
        if random() < 0.35:
            tk.add(0.9, self.head_mot.blinkeyes())
        tk.add(1.8, self.head_mot.blinkeyes())
        tk.add(1.9, self.head_mot.pantilt(pan=-0.3, tilt=-0.7, duration=0.45))
        tk.add(2.2, self.wheels_mot.rotate_by(angle=-0.5, duration=0.7))
        tk.add(2.5, self.head_mot.pantilt(pan=0, tilt=-0.55, duration=0.4))
        if random() < 0.35:
            tk.add(2.9, self.head_mot.blinkeyes())
        return tk

    def search_user_capture(self, starting_pose=(
 Head.PAN_NEUTRAL,
 Head.TILT_UP * 0.8, 0)):
        """
        Search for a user by looking up and around.
        :param: starting pose in radians, (pan, tilt, heading)
        Length: 12.67 seconds
        """
        start_pan = starting_pose[0]
        start_tilt = starting_pose[1]
        CENTER_THRESHOLD = 0.25
        pan_2 = Head.PAN_RIGHT * 0.8
        tilt_2 = Head.TILT_UP * 0.74
        pan_3 = Head.PAN_LEFT * 0.75
        tilt_3 = Head.TILT_UP * 0.8
        if start_pan > Head.PAN_LEFT * CENTER_THRESHOLD:
            pan_2 = Head.PAN_NEUTRAL
            tilt_2 = Head.TILT_UP * 0.8
            pan_3 = Head.PAN_RIGHT * 0.75
            tilt_3 = Head.TILT_UP * 0.7
        else:
            if start_pan < Head.PAN_RIGHT * CENTER_THRESHOLD:
                pan_2 = Head.PAN_NEUTRAL
                tilt_2 = Head.TILT_UP * 0.75
                pan_3 = Head.PAN_LEFT * 0.75
                tilt_3 = Head.TILT_UP * 0.8
        tk = Track()
        tk.add(0.1, self.head_mot.blinkeyes())
        tk.add(2.1, self.head_mot.blinkeyes())
        tk.add(3.0, self.head_mot.pantilt(pan=start_pan, tilt=Head.TILT_UP * 0.8, duration=0.35))
        tk.add(3.1, self.head_mot.blinkeyes())
        tk.add(5.9, self.head_mot.blinkeyes())
        tk.add(6.0, self.head_mot.pantilt(pan=pan_2, tilt=tilt_2, duration=0.25))
        tk.add(6.3, self.head_mot.blinkeyes())
        tk.add(9.0, self.head_mot.pantilt(pan=pan_3, tilt=tilt_3, duration=0.45))
        tk.add(9.5, self.head_mot.blinkeyes())
        tk.add(12.0, self.head_mot.pantilt(pan=start_pan, tilt=start_tilt, duration=0.5))
        tk.add(12.1, self.head_mot.blinkeyes())
        tk.add(12.5, self.head_mot.openeyes())
        return tk

    def stop(self, reset_head=False):
        """
        Stop the robot's base from moving (optionally reset the head too!).
        """
        tk = Track()
        tk.add(0.0, self.wheels_mot.stop())
        if reset_head:
            tk.add(0.0, self.head_mot.neutral())
            tk.add(0.0, self.head_mot.openeyes())
        return tk

    def inch(self, speed, length):
        tk = Track()
        tk.add(0.0, self.head_mot.neutral())
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.0, self.wheels_mot.inch(speed, length))
        return tk

    def turn_to(self, turn_angle_radians=0.0, tilt_radians=-0.5, periphery=radians(32.0)):
        """
        Turn and tilt (look at) animation.
        If the turn is within the the robot's periphery (~30 degrees), it will
        just move its head. Otherwise it will look with the head, the body, and
        center the head on the desired angle with a specified tilt.
        Duration is 0.7 to 1.67 seconds (for just a head look in periphery) or
        1.53 to 2.82 seconds for a full turn (depending on the angle).
        """
        TURN_TIME_BASE = 1.0
        if turn_angle_radians > pi:
            turn_angle_radians -= pi
            logger.warning(('angle outside [-pi, pi]: {}').format(turn_angle_radians))
        else:
            if turn_angle_radians < pi * -1:
                turn_angle_radians += pi
                logger.warning(('angle outside [-pi, pi]: {}').format(turn_angle_radians))
        outside_periphery = abs(turn_angle_radians) > periphery
        time_to_turn = TURN_TIME_BASE + abs(turn_angle_radians) * 0.5
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        if random() < 0.65:
            tk.add(0.2, self.head_mot.blinkeyes())
        if outside_periphery:
            turn_start_time = 0.2
            tk.add(0.0, self.head_mot.pantilt(pan=turn_angle_radians, tilt=tilt_radians, duration=0.45))
            tk.add(turn_start_time, self.wheels_mot.rotate_by(angle=turn_angle_radians, duration=time_to_turn))
            tk.add(turn_start_time + time_to_turn - 0.5, self.head_mot.pantilt(pan=0, tilt=tilt_radians, duration=0.5))
            if random() < 0.35:
                tk.add(turn_start_time + time_to_turn + 0.4, self.head_mot.blinkeyes())
        else:
            tk.add(0.0, self.head_mot.pantilt(pan=turn_angle_radians, tilt=tilt_radians, duration=0.6))
            tk.add(0.5, self.head_mot.openeyes())
            if random() < 0.15:
                tk.add(0.7, self.head_mot.blinkeyes())
                tk.add(1.1, self.head_mot.blinkeyes())
                tk.add(1.5, self.head_mot.openeyes())
        return tk

    def yes(self):
        """
        Do a yes animation (maintaining pan orientation),
        and return the head to the previous tilt location.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.29, self.sound_mot.open('Yes.wav'))
        init_tilt = self.head.cur_tilt
        init_pan = self.head.cur_pan
        tk.add(0.0, self.head_mot.pantilt(init_pan, clip(init_tilt - 0.2, Head.TILT_UP, Head.TILT_DOWN), 0.2))
        tk.add(0.2, self.head_mot.pantilt(init_pan, clip(init_tilt + 0.3, Head.TILT_UP, Head.TILT_DOWN), 0.2))
        tk.add(0.4, self.head_mot.pantilt(init_pan, clip(init_tilt - 0.3, Head.TILT_UP, Head.TILT_DOWN), 0.25))
        tk.add(0.65, self.head_mot.pantilt(init_pan, clip(init_tilt + 0.25, Head.TILT_UP, Head.TILT_DOWN), 0.25))
        tk.add(0.9, self.head_mot.pantilt(init_pan, init_tilt, 0.3))
        return tk

    def pantilt(self, x, y, length=0.5):
        """
        @param x: radians from the neutral position for pan
        @param y: radians from the neutral position for tilt
        """
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(x, y, length))
        return tk

    def pan(self, x):
        """
        @param x: radians from the neutral position (positive to the right)
        """
        tk = Track()
        tk.add(0.0, self.head_mot.lookat(x, 0.5))
        return tk

    def tilt(self, x):
        """
        @param x: radians from the neutral position (positive looking down)
        """
        tk = Track()
        tk.add(0.0, self.head_mot.pantilt(self.head.cur_pan, x, 0.5))
        return tk

    def eyes(self, x, spd=0.5):
        """
        @param x: radians from the neutral position (positive closing eyes,
                                                     negative happy face)
        """
        tk = Track()
        tk.add(0.0, self.head_mot.moveeyes(x, spd))
        return tk

    def capture_start(self):
        tk = Track()
        tk.add(0.0, self.lights_mot.off(0.05))
        tk.add(0.0, self.sound_mot.open('capture_start.wav'))
        tk.add(0.05, self.lights_mot.pulse(color=(90, 90, 90), pulse_up_time=0.4, pulse_down_time=0.2))
        tk.add(1.05, self.lights_mot.pulse(color=(120, 120, 120), pulse_up_time=0.35, pulse_down_time=0.2))
        tk.add(2.05, self.lights_mot.pulse(color=(155, 155, 155), pulse_up_time=0.3, pulse_down_time=0.2))
        return tk