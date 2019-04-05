import math, random
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from kuri_api import Head

class DockingAnimations(AnimationGroup):
    """
    Animation class for having Kuri perform docking animations while wheels
    are being controlled by the docking service.
    
    Any animation called in this class will cancel any active docking head
    animation.
    """

    def docking_turn_ccw(self, mood=0.0):
        """
        Plays while Kuri is rotating counter-clockwise to find dock LEDs.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        random_tilt = random.uniform(0.1, 0.5)
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(0.1 * speed, self.head_mot.pantilt(pan=0.8, tilt=random_tilt, duration=0.3 * speed))
        tk.add(0.2 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.3))
        tk.add(2.5 * speed, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.45, duration=0.4 * speed))
        tk.add(3.1 * speed, self.head_mot.pantilt(pan=0.5, tilt=0.35, duration=0.4 * speed))
        if random.random() < 0.5:
            tk.add(3.2 * speed, self.head_mot.blinkeyes())
        tk.add(5.5 * speed, self.head_mot.pantilt(pan=-0.1, tilt=0.45, duration=0.4 * speed))
        tk.add(5.5 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.2))
        return tk

    def docking_turn_cw(self, mood=0.0):
        """
        Plays while Kuri is rotating clockwise to find dock LEDs.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        random_pan = random.uniform(0.1, 0.6) * -1
        random_tilt = random.uniform(0.1, 0.65)
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(0.1 * speed, self.head_mot.pantilt(pan=-0.8, tilt=random_tilt, duration=0.3 * speed))
        tk.add(0.2 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.3))
        tk.add(2.8 * speed, self.head_mot.pantilt(pan=random_pan, tilt=0.45, duration=0.4 * speed))
        tk.add(3.9 * speed, self.head_mot.pantilt(pan=-0.5, tilt=0.35, duration=0.35 * speed))
        if random.random() < 0.5:
            tk.add(4.0 * speed, self.head_mot.blinkeyes())
        tk.add(6.5 * speed, self.head_mot.pantilt(pan=-0.1, tilt=0.45, duration=0.4 * speed))
        tk.add(6.5 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.2))
        return tk

    def docking_preparing_to_approach(self, mood=0.0):
        """
        Plays before Kuri starts to approach the near-field LED.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        random_side = random.choice([-1, 1])
        random_pan_1 = random.uniform(-0.4, -0.1) * random_side
        random_pan_2 = random.uniform(0.08, 0.14) * random_side
        random_tilt = random.uniform(0.0, 0.2)
        tk.add(0.1 * speed, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(0.1 * speed, self.head_mot.pantilt(pan=random_pan_1, tilt=Head.TILT_NEUTRAL, duration=0.35 * speed))
        tk.add(0.3 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.3))
        tk.add(1.3 * speed, self.head_mot.pantilt(pan=random_pan_2, tilt=random_tilt, duration=0.15 * speed))
        tk.add(1.9 * speed, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.15 * speed))
        tk.add(2.8 * speed, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.1 * speed))
        return tk

    def docking_approaching(self, mood=0.0):
        """
        Plays while Kuri is approaching the dock's near-field LED.
        """
        tk = Track()
        random_side = random.choice([-1, 1])
        speed = (mood + 10) * 0.1
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_OPEN))
        tk.add(1.0 * speed, self.head_mot.pantilt(pan=0.2 * random_side, tilt=0.35, duration=0.3 * speed))
        tk.add(2.3 * speed, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(2.4 * speed, self.head_mot.pantilt(pan=-0.3 * random_side, tilt=0.45, duration=0.22 * speed))
        tk.add(2.5 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.3))
        tk.add(3.3 * speed, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL * random_side, tilt=Head.TILT_NEUTRAL, duration=0.3 * speed))
        tk.add(5.1 * speed, self.head_mot.pantilt(pan=0.1 * random_side, tilt=Head.TILT_NEUTRAL, duration=0.3 * speed))
        return tk

    def docking_preparing_to_rotate_180(self, cw=True, mood=0.0):
        """
        Plays after Kuri has approached the dock (can see the close-field
        LED), before rotating to align the bot's charging contacts.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        side = 1 if cw else -1
        tk.add(0.1 * speed, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(0.1 * speed, self.head_mot.pantilt(pan=-0.8 * side, tilt=Head.TILT_NEUTRAL, duration=0.3 * speed))
        tk.add(0.3 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.35))
        tk.add(0.5 * speed, self.head_mot.pantilt(pan=-0.8 * side, tilt=Head.TILT_NEUTRAL, duration=0.1 * speed))
        return tk

    def docking_rotate_180(self, cw=True, mood=0.0):
        """
        Plays while Kuri is rotating 180 degrees, aligning the bot's charging
        contacts with the dock.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        side = 1 if cw else -1
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_OPEN))
        tk.add(0.8 * speed, self.head_mot.pantilt(pan=0.2 * side, tilt=0.35, duration=0.4 * speed))
        tk.add(2.3 * speed, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(2.4 * speed, self.head_mot.pantilt(pan=-0.6 * side, tilt=0.45, duration=0.3 * speed))
        tk.add(2.6 * speed, self.head_mot.moveeyes(Head.EYES_OPEN, 0.25))
        return tk

    def docking_preparing_to_back_in(self, mood=0.0):
        """
        Plays before Kuri starts backing onto the dock.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        random_side = random.choice([-1, 1])
        random_time_add = random.uniform(0.0, 0.4)
        random_tilt = random.uniform(0.25, 0.55)
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_OPEN))
        tk.add(0.0, self.head_mot.pantilt(pan=0.8 * random_side, tilt=random_tilt, duration=0.45 * speed))
        tk.add(0.6 * speed + random_time_add, self.head_mot.pantilt(pan=0.8 * random_side, tilt=random_tilt, duration=0.1 * speed))
        return tk

    def docking_back_in(self, mood=0.0):
        """
        Plays while Kuri is backing up onto the dock.
        """
        tk = Track()
        speed = (mood + 10) * 0.1
        random_side = random.choice([-1, 1])
        random_pan = random.uniform(-0.8, 0.3) * random_side
        random_time_add = random.uniform(0.0, 0.6)
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_OPEN))
        tk.add(1.5 * speed, self.head_mot.pantilt(pan=random_pan, tilt=0.35, duration=0.4 * speed))
        tk.add(2.2 * speed + random_time_add, self.head_mot.moveeyes(Head.EYES_CLOSED, 0.15))
        tk.add(2.3 * speed + random_time_add, self.head_mot.pantilt(pan=0.8 * random_side, tilt=0.25, duration=0.5 * speed))
        tk.add(2.5 * speed + random_time_add, self.head_mot.moveeyes(Head.EYES_OPEN, 0.3))
        tk.add(3.8 * speed + random_time_add, self.head_mot.pantilt(pan=0.2 * random_side, tilt=0.25, duration=0.5 * speed))
        return tk

    def docking_reset(self):
        """
        Reset the head to a neutral position.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.reset())
        return tk


class DockingLEDAnimations(AnimationGroup):
    """
    Animation class for having Kuri perform docking LED animations
    Does LED only so it can run independently of head and wheel movements
    
    Any animation called in this class will cancel any active docking LED
    animation.
    """

    def docking_looking_for_dock(self):
        tk = Track()
        tk.add(0.0, self.movies.to_animated('docking_looking_for_dock.mov'))
        return tk

    def docking_back_up(self):
        tk = Track()
        tk.add(0.0, self.movies.to_animated('docking_back_up.mov'))
        return tk


class DockingSupportAnimations(AnimationGroup):
    """
    Support animations for docking. These are called when the docking service
    is not controlling any DOFs.
    """

    def docking_waypoint_reached(self, mood=0.0):
        """
        This animation runs when Kuri reaches the docking waypoint, before
        the docking homing proceudre has started.
        """
        tk = Track()
        speed = (mood * 1.8 + 10) * 0.1
        random_side = random.choice([-1, 1])
        sound_time = random.uniform(0.2, 0.4) * speed
        tk.add(0.0, self.head_mot.moveeyes(Head.EYES_OPEN))
        tk.add(0.1 * speed, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.1, duration=0.2 * speed))
        tk.add(sound_time, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=0.1, duration=0.2 * speed))
        tk.add(sound_time, self.sound_mot.open('Yes.wav'))
        tk.add(0.9 * speed, self.head_mot.pantilt(pan=0.1 * random_side, tilt=-0.1, duration=0.2 * speed))
        tk.add(1.2 * speed, self.head_mot.pantilt(pan=-0.1 * random_side, tilt=-0.1, duration=0.2 * speed))
        tk.add(1.8 * speed, self.head_mot.pantilt(pan=0.1 * random_side, tilt=-0.1, duration=0.15 * speed))
        tk.add(2.2 * speed, self.head_mot.pantilt(pan=0.1 * random_side, tilt=-0.1, duration=0.1 * speed))
        return tk

    def docking_complete_happy(self, speed=1.0):
        """
        Plays after Kuri has successfully docked.
        This only plays the docking sound and a happy face
        """
        tk = Track()
        random_side = random.choice([-1, 1])
        tk.add(0.0, self.movies.to_animated('starburst-green.mov'))
        tk.add(0.0, self.sound_mot.open('docked.wav'))
        tk.add(0.0, self.head_mot.pantilt(pan=-0.4 * random_side, tilt=Head.TILT_NEUTRAL, duration=0.45 * speed))
        tk.add(0.1 * speed, self.head_mot.moveeyes(Head.EYES_HAPPY, 0.3))
        tk.add(1.1 * speed, self.head_mot.pantilt(pan=0.3 * random_side, tilt=-0.1, duration=0.3 * speed))
        return tk

    def docking_complete_asleep(self, mood=0.0):
        """
        Plays after Kuri has successfully docked.
        This version goes to sleep.
        """
        speed = (mood + 10) * 0.1
        tk = self.docking_complete_happy(speed=speed)
        tk.add(1.6 * speed, self.head_mot.blinkeyes())
        if random.random() < 0.4:
            tk.add(2.8 * speed, self.head_mot.blinkeyes())
        if random.random() < 0.2:
            tk.add(4.0 * speed, self.head_mot.blinkeyes())
        tk.add(5.0, self.head_mot.neutral())
        tk.add(5.0, self.head_mot.blinkeyes(close_time=0.18, open_time=0.28))
        tk.add(6.0, self.head_mot.blinkeyes())
        tk.add(6.0, self.head_mot.pantilt(0.0, 0.1, 0.3))
        tk.add(6.0, self.sound_mot.open('Sleep.wav'))
        tk.add(6.5, self.head_mot.blinkeyes(open_time=0.12, open_amplitude=0.9, close_time=0.12))
        tk.add(6.6, self.head_mot.pantilt(0.0, 0.2, 0.2))
        tk.add(6.8, self.head_mot.blinkeyes(open_time=0.1, open_amplitude=0.8, close_time=0.1))
        tk.add(7.0, self.head_mot.pantilt(0.0, 0.25, 0.2))
        tk.add(7.1, self.head_mot.blinkeyes(open_time=0.1, open_amplitude=0.7, close_time=0.1))
        tk.add(7.8, self.head_mot.blinkeyes(open_time=0.1, open_amplitude=0.6, close_time=0.1))
        tk.add(8.0, self.head_mot.closeeyes())
        tk.add(8.0, self.head_mot.pantilt(0.0, self.head.TILT_DOWN, 0.5))
        return tk

    def undock_from_old_home(self):
        """
        Drives forward off of dock, spins 180 degrees.
        180 degrees in the opposite direction.
        """
        random_pan_side = random.choice([-1, 1])
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.0, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN * 0.8, duration=0.3))
        tk.add(0.0, self.wheels_mot.inch(0.275, 8.0))
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
        tk.add(8.0, self.head_mot.blinkeyes())
        tk.add(8.1, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.7 * random_pan_side, tilt=Head.TILT_DOWN * 0.15, duration=0.4))
        tk.add(8.5, self.wheels_mot.rotate_by(angle=-math.pi * random_pan_side, duration=4.0))
        tk.add(6.8, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.3 * random_pan_side, tilt=Head.TILT_DOWN * 0.6, duration=0.55))
        tk.add(7.5, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.55 * random_pan_side, tilt=Head.TILT_DOWN * 0.1, duration=0.4))
        tk.add(8.2, self.head_mot.blinkeyes())
        tk.add(8.2, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.75 * random_pan_side, tilt=Head.TILT_DOWN * 0.6, duration=0.5))
        tk.add(9.3, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.45 * random_pan_side, tilt=Head.TILT_DOWN * 0.7, duration=0.5))
        tk.add(10.3, self.head_mot.blinkeyes())
        tk.add(10.8, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.8 * random_pan_side, tilt=Head.TILT_DOWN * 0.1, duration=0.3))
        tk.add(11.5, self.head_mot.openeyes())
        tk.add(11.5, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.8))
        tk.add(12.5, self.head_mot.blinkeyes())
        tk.add(12.5, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN * 0.8, duration=0.4))
        tk.add(13.9, self.head_mot.blinkeyes())
        tk.add(13.9, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.5))
        tk.add(16.9, self.head_mot.blinkeyes())
        return tk

    def docking_complete_awake(self, mood=0.0):
        """
        Plays after Kuri has successfully docked.
        This version just plays the docked sound and the happy face
        """
        speed = (mood + 10) * 0.1
        tk = self.docking_complete_happy(speed=speed)
        return tk

    def redock(self):
        """
        This is designed to redock a robot that has inched off the dock
        when asleep. It needs to be a short motion so we don't grind the
        dock, especially if 'undocking' actually happened due to a power
        outage. The user should still be able to push Kuri off the dock.
        If it becomes not wheels-only make sure sleep coordinates it with
        twitches
        """
        tk = Track()
        tk.add(0.5, self.wheels_mot.inch(-0.3, 0.2))
        return tk

    def undock(self):
        """
        This is designed to get the robot off the dock before it starts
        to navigate
        """
        tk = Track()
        tk.add(0.5, self.wheels_mot.inch(0.5, 0.5))
        return tk

    def live_undock(self):
        """
        This is designed to get the robot off the dock before it starts
        to navigate, in live mode (so it should never be made 'cute')
        """
        tk = Track()
        tk.add(0.5, self.wheels_mot.inch(0.5, 0.5))
        return tk
