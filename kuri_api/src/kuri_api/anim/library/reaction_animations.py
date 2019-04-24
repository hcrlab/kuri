import math, random
from kuri_api.utils import interp
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track
from kuri_api import Head, Lights

class ReactionAnimations(AnimationGroup):

    def at_attention_look_around(self, pan_face=Head.PAN_NEUTRAL, tilt_face=-0.5, move_base=True):
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        if move_base:
            tk.add(0.0, self.wheels_mot.inch(0.08, 0.7))
        tk.add(0.08, self.head_mot.happyeyes(0.17))
        tk.add(0.25, self.head_mot.pantilt(pan_face, tilt_face, 0.25))
        return tk

    def at_attention_reset(self):
        tk = Track()
        tk.add(0.0, self.head_mot.reset())
        tk.add(0.4, self.head_mot.openeyes())
        return tk

    def battery_low(self):
        tk = Track()
        tk.add(0.0, self.lights_mot.glow(255, 255, 255, 0.3, 0.2))
        tk.add(0.0, self.head_mot.pantilt(pan=0.0, tilt=0.0, duration=0.4))
        tk.add(0.1, self.head_mot.openeyes(time=0.3, amplitude=1.0))
        tk.add(2.0, self.head_mot.pantilt(pan=0.0, tilt=0.1, duration=0.4))
        tk.add(2.0, self.head_mot.openeyes(time=0.3, amplitude=0.85))
        tk.add(2.5, self.lights_mot.glow(100, 100, 100, 0.4, 0.4))
        return tk

    def battery_critical(self):
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes(time=0.2, amplitude=0.7))
        tk.add(0.0, self.head_mot.pantilt(pan=-0.2, tilt=0.35, duration=0.35))
        add = 0
        for counter in range(0, 6):
            tk.add(0 + add, self.lights_mot.glow(100, 100, 100, 0.2, 0.2))
            add = add + 0.5

        tk.add(0.8, self.head_mot.pantilt(pan=0.2, tilt=0.35, duration=0.35))
        tk.add(1.25, self.head_mot.openeyes(time=0.2, amplitude=-0.5))
        tk.add(1.4, self.head_mot.pantilt(pan=0.0, tilt=0.5, duration=0.35))
        tk.add(3.0, self.lights_mot.glow(0, 0, 0, 1.0, 1.0))
        return tk

    def bump(self, new_direction=Head.PAN_NEUTRAL):
        """
         #Robot comes to stop
         00:00 Regular Eye Blink
        *00:00 Very small subtle translate Back .21sec.
         00:10 Head pans to (value range of 15 to 45) multiplied randomly by (-1
            or 1) .33 sec
         00:10 Head rotates down -20 degrees .33 sec
         00:58 Regular Eye Blink
         00:68 Head pans to (current value) multiplied by (-1) .45sec
        *02:17 #pause here longer for internal calculations
         02:17 #no eye blink here
         02:17 Head Pans in direction of new corrected path speed: 60
            degrees/sec
         02:17 Head rotates Up to neutral .3sec
        *03:00 Body pans/rotates to value of Head Pan value speed: 100
            degrees/sec
        *04:00 Head pans again if need be to old end point for navigation.
        *04:16 Navigate
         """
        tk = Track()
        tk.add(0, self.head_mot.blinkeyes())
        rand_pan = Head.PAN_NEUTRAL + math.radians(random.choice([1, -1]) * random.uniform(15, 45))
        tk.add(0.1, self.head_mot.pantilt(rand_pan, Head.TILT_NEUTRAL + math.radians(20), 0.33))
        tk.add(0.58, self.head_mot.blinkeyes())
        tk.add(0.68, self.head_mot.pantilt(-rand_pan, Head.TILT_NEUTRAL + math.radians(20), 0.33))
        tk.add(2.17, self.head_mot.pantilt(new_direction, Head.TILT_NEUTRAL, 0.3))
        tk.add(0, self.sound_mot.open('Bump.wav'))
        return tk

    def dance_music_detected(self):
        """
        Reaction when music is detected and kuri is about to dance.
        Length: ~3.1 seconds
        """
        tk = Track()
        random_side = random.choice([-1, 1])
        tk.add(0.0, self.head_mot.happyeyes())
        tk.add(0.0, self.wheels_mot.inch(0.28, 0.3))
        pan1 = random_side * Head.PAN_LEFT * random.uniform(0.5, 0.7)
        pan2 = random_side * Head.PAN_RIGHT * random.uniform(0.5, 0.7)
        tilt1 = Head.TILT_UP * random.uniform(0.45, 0.6)
        tk.add(0.1, self.head_mot.pantilt(pan=pan1, tilt=tilt1, duration=0.3))
        tk.add(0.5, self.head_mot.blink(speed=-1.0, done_eye_position=Head.EYES_HAPPY))
        if random.random() < 0.3:
            tk.add(1.0, self.head_mot.blink(speed=-1.0, done_eye_position=Head.EYES_HAPPY))
        tk.add(1.75, self.head_mot.pantilt(pan=pan2, tilt=tilt1 + 0.1, duration=0.6))
        tk.add(1.85, self.head_mot.blink(speed=-1.0, done_eye_position=Head.EYES_HAPPY))
        tk.add(2.85, self.head_mot.blink(speed=-1.0))
        return tk

    def dance_done(self):
        """
        Reaction when music is no longer detected and kuri has stopped dancing.
        Length: 4.15 seconds
        """
        tk = Track()
        tk.add(0.0, self.head_mot.moveeyes(0.12, 0.3))
        rand_pan = random.choice([-1, 1]) * 0.2
        tilt_up = Head.TILT_NEUTRAL - math.radians(25)
        tk.add(0.0, self.head_mot.pantilt(pan=rand_pan, tilt=tilt_up, duration=0.3))
        tk.add(0.1, self.wheels_mot.rotate_by(angle=rand_pan * 4, duration=0.6))
        tk.add(0.4, self.head_mot.pantilt(pan=rand_pan * 0.1, tilt=tilt_up, duration=0.3))
        tk.add(0.75, self.head_mot.blinkeyes())
        tk.add(1.25, self.head_mot.moveeyes(0.12, 0.3))
        tk.add(1.25, self.head_mot.pantilt(pan=rand_pan * -1, tilt=tilt_up * 1.2, duration=0.45))
        tk.add(1.35, self.wheels_mot.rotate_by(angle=rand_pan * -5, duration=0.7))
        tk.add(1.75, self.head_mot.pantilt(pan=rand_pan * -0.1, tilt=tilt_up * 0.7, duration=0.3))
        tk.add(2.25, self.head_mot.pantilt(pan=rand_pan * -0.7, tilt=Head.TILT_DOWN * 0.5, duration=0.4))
        tk.add(2.25, self.wheels_mot.rotate_by(angle=rand_pan * -2, duration=0.4))
        tk.add(2.85, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_NEUTRAL, duration=0.6))
        tk.add(2.95, self.wheels_mot.rotate_by(angle=rand_pan * 2, duration=0.5))
        tk.add(3.2, self.head_mot.blinkeyes())
        if random.random() < 0.4:
            tk.add(3.6, self.head_mot.blinkeyes())
        tk.add(3.85, self.head_mot.neutral(0.3))
        tk.add(3.85, self.head_mot.openeyes(0.3))
        return tk

    def fart(self):
        """
        The juice is loose.
        """
        tk = Track()
        tk.add(0.0, self.sound_mot.open('Fart.wav'))
        tk.add(0.0, self.wheels_mot.inch(0.28, 0.3))
        blink = self.head_mot.blinkeyes(close_time=0.12, open_time=0.24)
        tk.add(0.0, blink)
        tk.add(0.5, self.head_mot.pantilt(pan=0.0, tilt=0.6, duration=0.3))
        tk.add(0.7, self.wheels_mot.inch(-0.09, 0.7))
        tk.add(0.9, self.head_mot.pantilt(pan=0.6, tilt=0.6, duration=0.3))
        tk.add(1.0, self.wheels_mot.rotate(-3.0, 0.2))
        tk.add(1.2, self.head_mot.pantilt(pan=-0.6, tilt=0.6, duration=0.3))
        tk.add(1.3, self.wheels_mot.rotate(3.0, 0.2))
        tk.add(1.7, self.head_mot.pantilt(pan=0.0, tilt=0.3, duration=0.6))
        tk.add(2.1, blink)
        tk.add(2.5, blink)
        tk.add(2.8, self.head_mot.happyposture())
        tk.add(3.2, self.sound_mot.open('Giggle.wav'))
        tk.add(3.2, self.wheels_mot.rotate(-4.0, 0.2))
        tk.add(3.4, self.wheels_mot.rotate(4.0, 0.2))
        tk.add(3.6, self.head_mot.neutral())
        tk.add(3.6, self.head_mot.openeyes())
        return tk

    def old_giggle(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyposture())
        tk.add(0.4, self.sound_mot.open('Giggle.wav'))
        tk.add(0.4, self.wheels_mot.rotate(-4.0, 0.2))
        tk.add(0.7, self.wheels_mot.rotate(4.0, 0.2))
        tk.add(1.0, self.head_mot.neutral())
        tk.add(1.0, self.head_mot.openeyes())
        return tk

    def gotit_docked(self):
        tk = Track()
        init_tilt = self.head.cur_tilt
        init_pan = self.head.cur_pan
        tk.add(0.0, self.head_mot.happyeyes())
        if init_tilt + 0.45 > Head.TILT_DOWN:
            temp_init_tilt = Head.TILT_DOWN - 0.45
            tk.add(0.1, self.head_mot.pantilt(init_pan, temp_init_tilt, 0.2))
            tk.add(0.3, self.head_mot.pantilt(init_pan, temp_init_tilt + 0.45, 0.3))
            tk.add(0.7, self.head_mot.pantilt(init_pan, temp_init_tilt, 0.3))
            tk.add(1.1, self.head_mot.pantilt(init_pan, temp_init_tilt + 0.35, 0.3))
            tk.add(1.5, self.head_mot.pantilt(init_pan, temp_init_tilt, 0.2))
            tk.add(2.2, self.head_mot.openeyes())
        else:
            tk.add(0.1, self.head_mot.pantilt(init_pan, init_tilt + 0.45, 0.2))
            tk.add(0.4, self.head_mot.pantilt(init_pan, init_tilt, 0.3))
            tk.add(0.7, self.head_mot.pantilt(init_pan, init_tilt + 0.35, 0.3))
            tk.add(1.1, self.head_mot.pantilt(init_pan, init_tilt, 0.2))
            tk.add(1.8, self.head_mot.openeyes())
        got_it_sound = self.sound_mot.open('Got_It.wav')
        tk.add(0, got_it_sound)
        tk.add(0, self.movies.to_animated('starburst-green.mov'))
        return tk

    def gotit(self):
        tk = self.gotit_docked()
        return tk

    def head_touch(self):
        """
        Runs when the robot's head is touched and held.
        """
        tk = Track()
        random_side = random.choice([-1, 1])
        tk.add(0.0, self.head_mot.openeyes(time=0.2, amplitude=-0.5))
        tk.add(0.1, self.head_mot.pantilt(pan=0.2 * random_side, tilt=-0.6, duration=0.7))
        tk.add(0.27, self.head_mot.openeyes(time=0.25, amplitude=1.5))
        tk.add(1.2, self.head_mot.pantilt(pan=0.4, tilt=-0.6, duration=0.4))
        tk.add(1.7, self.head_mot.pantilt(pan=-0.4, tilt=-0.55, duration=0.4))
        tk.add(2.3, self.head_mot.pantilt(pan=-0.2, tilt=-0.6, duration=0.4))
        tk.add(1.8, self.head_mot.openeyes(time=0.25, amplitude=-0.5))
        tk.add(3.7, self.head_mot.pantilt(pan=0.3, tilt=-0.6, duration=0.4))
        tk.add(4.2, self.head_mot.pantilt(pan=0, tilt=-0.5, duration=0.4))
        tk.add(4.65, self.head_mot.pantilt(pan=0.15, tilt=-0.6, duration=0.4))
        tk.add(4.6, self.head_mot.openeyes(time=0.5, amplitude=1.35))
        tk.add(6.45, self.head_mot.pantilt(pan=0.1, tilt=-0.45, duration=0.3))
        tk.add(7.2, self.head_mot.pantilt(pan=0.4, tilt=-0.3, duration=0.6))
        tk.add(7.9, self.head_mot.pantilt(pan=-0.4, tilt=-0.25, duration=0.6))
        tk.add(8.6, self.head_mot.pantilt(pan=-0.2, tilt=-0.3, duration=0.7))
        tk.add(10.6, self.head_mot.openeyes(time=0.25, amplitude=-0.5))
        tk.add(10.9, self.head_mot.pantilt(pan=-0.3, tilt=-0.25, duration=0.6))
        tk.add(11.6, self.head_mot.pantilt(pan=-0.2, tilt=-0.3, duration=0.7))
        tk.add(11.6, self.head_mot.openeyes(time=0.5, amplitude=1.1))
        tk.add(12.9, self.head_mot.openeyes(time=0.2, amplitude=-0.5))
        tk.add(13.0, self.head_mot.pantilt(pan=0, tilt=-0.2, duration=0.3))
        tk.add(13.15, self.head_mot.openeyes(time=0.3, amplitude=1.1))
        tk.add(13.3, self.head_mot.pantilt(pan=-0.2, tilt=0, duration=0.5))
        return tk

    def head_touch_end(self):
        """
        Runs when a long head-touch ends.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
        tk.add(0.1, self.head_mot.pantilt(pan=0, tilt=0, duration=0.3))
        tk.add(0.2, self.head_mot.openeyes(time=0.15, amplitude=1.1))
        tk.add(1.4, self.head_mot.openeyes(time=0.15, amplitude=-0.5))
        tk.add(1.5, self.head_mot.pantilt(pan=-0.1, tilt=0, duration=0.3))
        tk.add(1.6, self.head_mot.openeyes(time=0.15, amplitude=1.1))
        return tk

    def tickle(self):
        return self.old_giggle()

    def tickle_end(self):
        tk = Track()
        tk.add(0.0, self.head_mot.neutral())
        return tk

    def huh1(self, pan_face=Head.PAN_NEUTRAL):
        tk = self.huh1_docked(pan_face=pan_face)
        tk.add(0.63, self.wheels_mot.inch(0.08, 1.5))
        return tk

    def huh1_offline(self, pan_face=Head.PAN_NEUTRAL):
        tk = self.huh1_docked(pan_face=pan_face, chest_movie='error-red.mov')
        tk.add(0.63, self.wheels_mot.inch(0.08, 1.5))
        return tk

    def huh1_docked(self, pan_face=Head.PAN_NEUTRAL, chest_movie='starburst-orange.mov'):
        """
          - Modified by Patrick & Paul on 2016-04-20, out of sync with
            Doug's scriptk.
        
          From Doug Dooley's Notes:
        
          #HUH? -1st time it happens
          #Robot stops if it hasn't already
          #Robot may or may not be facial tracking
          00:00 Chest Light OFF .08 Sec # This chest light blink has pause
          00:00 Head Rotates UP 12 degrees from current position .41sec # add on
              top of facial tracking number if need be
        
          00:00 EyeLid drops to neutral if it isn't already in neutral.
        * 00:10 Light turns from blue to orange
          00:25 Chest Light ON .17sec
          00:41 Chest Light Blink REGULAR
          00:91 Head rotates Left 5degrees .38sec # add on top of facial tracking
                number if need be
          01.29 Head rotate Right 10degrees .41sec # add on top of facial
                tracking number if need be
          01:71 Head rotates Left 10degrees .58sec # add on top of facial
                tracking number if need be
          02:29 Head rotates Right 5degrees .38secs # add on top of facial
                tracking number if need be
        * 03:09 Start Thinking animation #Put thinking animation on top of
                facial tracking, if robot currently tracking face
        
          * => Unimplemented
          """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0, self.sound_mot.open('Huh1.wav'))
        tk.add(0.0, self.movies.to_animated(chest_movie))
        head_up = Head.TILT_NEUTRAL - math.radians(12)
        tk.add(0, self.head_mot.pantilt(pan_face, head_up, 0.41))
        tk.add(0, self.head_mot.moveeyes(Head.EYES_NEUTRAL, 0.41))
        tk.add(0.91, self.head_mot.pantilt(pan_face + math.radians(5), head_up, 0.38))
        tk.add(1.29, self.head_mot.pantilt(pan_face - math.radians(5), head_up, 0.41))
        tk.add(1.71, self.head_mot.pantilt(pan_face + math.radians(5), head_up, 0.58))
        tk.add(2.29, self.head_mot.pantilt(pan_face - math.radians(5), head_up, 0.38))
        tk.add(2.67, self.head_mot.neutral())
        tk.add(2.67, self.head_mot.openeyes())
        return tk

    def huh1_offline_docked(self, pan_face=Head.PAN_NEUTRAL):
        return self.huh1_docked(pan_face=pan_face, chest_movie='error-red.mov')

    def huh2(self, pan_face=Head.PAN_NEUTRAL, move_base=True):
        """
         - Modified by Patrick & Paul on 2016-04-20, out of sync with
           Doug's scriptk.
        
         00:00 Chest Light OFF .08 Sec # This chest light blink has pause
         00:00 Head Rotates UP 12 degrees from current position .41sec # add on
               top of facial tracking number if need be
         00:00 EyeLid drops to neutral if it isn't already in neutral.
                 (duration?)
        *00:10 Light turns from blue to yellow
         00:25 Chest Light ON .17sec
         00:41 Chest Light Blink REGULAR
         00:54 Head rotates Left 5 degrees .12sec # add on top of facial
                tracking number if need be
         00:88 EyeLid shuts .12sec
         00:88 Head rotates Down to bottom .54sec
         01:04 EyeLid opens to less than half lid (very sad eye) .2sec
         01:08 Head Rotates Right 10degrees .29sec
         01:37 Head Rotates Left 10degrees .38sec
         01:54 Body translates fwd 2-3 inches 1.5sec
         01:75 Head Rotate Right 10degrees .33sec
        *02:00 Sends signal to phone to ring with message. Who's phone though.
               I'm guessing there is no solution
         02:08 Head Rotates Left 5 degrees .29sec
         03:54 EyeLid Shuts .08sec
         03:58 Head Rotates back Up to previous value .21sec #start facial
               tracking again
         03:62 EyeLid Opens to Fully open eye .21sec
        *04:32 Start "Thinking animation" #Put thinking animation on top of
               facial tracking, if robot currently tracking face
        
         * => Unimplemented
         """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(1.1, self.sound_mot.open('Huh2.wav'))
        tk.add(0, self.lights_mot.off(0.08))
        tk.add(0.1, self.lights_mot.animate(Lights.ALL_OFF, Lights.all_leds(Lights.YELLOW), interp.quad_in, 0.15))
        tk.add(0.41, self.lights_mot.blink_regular(Lights.YELLOW))
        tk.add(2.83, self.lights_mot.off(0.1))
        if move_base:
            tk.add(1.54, self.wheels_mot.inch(0.08, 1.5))
        tilt_up = Head.TILT_NEUTRAL - math.radians(12)
        pan_left = pan_face + math.radians(5)
        pan_right = pan_face - math.radians(5)
        d = 1.0
        tk.add(d * 0, self.head_mot.pantilt(pan_face, tilt_up, d * 0.41))
        tk.add(d * 0, self.head_mot.moveeyes(Head.EYES_NEUTRAL, d * 0.41))
        tk.add(d * 0.54, self.head_mot.pantilt(pan_left, tilt_up, d * 0.12))
        tk.add(d * 0.88, self.head_mot.closeeyes(d * 0.12))
        tk.add(d * 1.04, self.head_mot.moveeyes(Head.EYES_SUPER_SAD, d * 0.2))
        tk.add(d * 1.08, self.head_mot.pantilt(pan_right, tilt_up, d * 0.29))
        tk.add(d * 1.37, self.head_mot.pantilt(pan_left, tilt_up, d * 0.38))
        tk.add(d * 1.75, self.head_mot.pantilt(pan_right, tilt_up, d * 0.33))
        tk.add(d * 2.08, self.head_mot.pantilt(pan_left, tilt_up, d * 0.29))
        tk.add(d * 3.54, self.head_mot.closeeyes(d * 0.08))
        tk.add(d * 3.58, self.head_mot.pantilt(pan_face, tilt_up, d * 0.21))
        tk.add(d * 3.62, self.head_mot.openeyes(d * 0.21))
        tk.add(d * 3.83, self.head_mot.neutral())
        return tk

    def listening(self):
        tk = Track()
        tk.add(0.0, self.lights_mot.glow(0, 204, 255, 0.8, float('inf')))
        return tk

    def listening_pose(self):
        TILT_LOOK_USER_RADIANS = -0.8
        tk = Track()
        tk.add(0.04, self.head_mot.happyeyes(0.17))
        tk.add(0.0, self.head_mot.pantilt(Head.PAN_NEUTRAL, TILT_LOOK_USER_RADIANS, 0.25))
        return tk

    def lost(self):
        """
        Kuri is lost (nav failure, etc.)
        Length: 7.26 seconds
        """
        tk = Track()
        tk.add(0.0, self.head_mot.openeyes())
        tk.add(0.0, self.movies.to_animated('error-red.mov'))
        tk.add(0.1, self.wheels_mot.inch(-0.35, 0.4))
        tk.add(0.1, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_DOWN, duration=0.4))
        tk.add(0.57, self.head_mot.blinkeyes())
        tk.add(0.8, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.4, tilt=Head.TILT_DOWN * 0.3, duration=0.3))
        tk.add(0.9, self.wheels_mot.rotate(-4.0, 0.25))
        tk.add(1.2, self.head_mot.blinkeyes())
        tk.add(1.5, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.7, tilt=Head.TILT_DOWN * 0.1, duration=0.5))
        tk.add(1.55, self.head_mot.blinkeyes())
        tk.add(1.6, self.wheels_mot.rotate(4.0, 0.25))
        tk.add(2.2, self.head_mot.pantilt(pan=Head.PAN_NEUTRAL, tilt=Head.TILT_UP * 0.4, duration=0.3))
        tk.add(2.2, self.wheels_mot.inch(0.4, 0.2))
        tk.add(2.6, self.sound_mot.open('Lost.wav'))
        tk.add(3.1, self.head_mot.blinkeyes())
        tk.add(3.5, self.lights_mot.off(1.5))
        if random.random() < 0.5:
            tk.add(4.1, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.6, tilt=Head.TILT_NEUTRAL, duration=0.4))
            tk.add(4.2, self.wheels_mot.rotate_by(angle=math.radians(60), duration=0.8))
            tk.add(4.7, self.head_mot.neutral())
            tk.add(6.3, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.5, tilt=Head.TILT_NEUTRAL, duration=0.4))
            tk.add(6.4, self.wheels_mot.rotate_by(angle=math.radians(-60), duration=0.5))
        else:
            tk.add(4.1, self.head_mot.pantilt(pan=Head.PAN_RIGHT * 0.6, tilt=Head.TILT_NEUTRAL, duration=0.4))
            tk.add(4.2, self.wheels_mot.rotate_by(angle=math.radians(-60), duration=0.8))
            tk.add(4.7, self.head_mot.neutral())
            tk.add(6.3, self.head_mot.pantilt(pan=Head.PAN_LEFT * 0.5, tilt=Head.TILT_NEUTRAL, duration=0.4))
            tk.add(6.4, self.wheels_mot.rotate_by(angle=math.radians(60), duration=0.5))
        tk.add(4.6, self.head_mot.blinkeyes())
        tk.add(5.5, self.head_mot.blinkeyes())
        tk.add(6.4, self.head_mot.blinkeyes())
        tk.add(6.8, self.head_mot.neutral())
        return tk

    def pickup(self):
        """
        Animation that runs when the robot is picked up.
        """
        tk = Track()
        tk.add(0.0, self.head_mot.reset())
        tk.add(0, self.sound_mot.open('Pickup.wav'))
        return tk

    def putdown(self):
        """
        Animation that runs when the robot is put down.
        """
        tk = Track()
        tk.add(0, self.sound_mot.open('Putdown.wav'))
        return tk

    def reset_sad(self):
        """
        Animation that runs when the robot is reset
        """
        tk = Track()
        tk.add(0.0, self.head_mot.sadposture())
        tk.add(0.0, self.sound_mot.open('Sad3.wav'))
        tk.add(1.0, self.head_mot.closeeyes())
        tk.add(1.0, self.head_mot.pantilt(self.head.PAN_NEUTRAL, self.head.TILT_DOWN, 0.5))
        return tk

    def sheep(self):
        tk = Track()
        tk.add(0.0, self.head_mot.happyposture())
        head_wiggle = self.head_mot.headwiggle(self.head.cur_pan, Head.TILT_NEUTRAL - 0.18)
        tk.add(0.3, head_wiggle)
        baa = self.sound_mot.open('sheep.wav')
        tk.add(0.0, baa)
        tk.add(baa.length(), baa)
        tk.add(3.0, self.head_mot.reset())
        return tk

    def waypoint_reached(self, head_pose=None):
        """
        Runs when a commanded waypoint is reached.
        """
        tk = Track()
        if head_pose:
            tk.add(0, self.head_mot.pantilt(head_pose.pan, head_pose.tilt, 0.5))
        return tk
