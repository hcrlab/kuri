from math import radians
from kuri_api.anim import AnimationGroup
from kuri_api.anim import Track

class RomojiAnimations(AnimationGroup):

    def happy_birthday(self):
        tk = Track()
        song = self.sound_mot.open('Happy_Birthday.wav')
        tk.add(0, song)
        tk.add(0, self.lights_mot.happy_birthday(song.length(), period=5.0))
        return tk

    def i_love_you(self):
        """
        #I LOVE YOU
        #Twist (-5) degrees .33 sec,
               (3.4) degrees .38sec,
               (-2.85) .38sec
               (2.15) .25sec
        00:00 -Rotate eyelid up to Happy .25sec
        00:04 -Head Side to Side Rotate Twist
        00:12 -Head Rotate (Current Position + Up 5 degrees) .29sec
        00:21 -Body Rotate Side to Side (Twist) * -2
        01:04 -Head Rotate (Current Position - Down 5 degrees) .5sec
        """
        tk = Track()
        tk.add(0, self.head_mot.happyeyes(time=0.25))
        tk.add(0, self.sound_mot.open('i_love_you.wav'))
        tk.add(0, self.lights_mot.off(0.4))
        head_wiggle = self.head_mot.headwiggle(self.head.cur_pan, self.head.cur_tilt)
        tk.add(0.04, head_wiggle)
        tk.add(0.04, self.wheels_mot.bodywiggle(direction=-1))
        tk.add(head_wiggle.length() + 0.12, self.head_mot.pantilt(self.head.cur_pan, self.head.cur_tilt - radians(5), 0.29))
        tk.add(head_wiggle.length() + 1.04, self.head_mot.pantilt(self.head.cur_pan, self.head.cur_tilt + radians(5), 0.5))
        tk.add(head_wiggle.length() + 3.0, self.head_mot.reset())
        return tk

    def lullaby_song(self):
        tk = Track()
        song = self.sound_mot.open('Lullaby.wav')
        tk.add(0, song)
        tk.add(0, self.lights_mot.night_light(song.length(), period=10.0))
        return tk

    def night_light(self):
        tk = Track()
        tk.add(0.0, self.head_mot.neutral())
        lullaby = self.sound_mot.open('Lullaby.wav')
        tk.add(0, lullaby)
        tk.add(0, self.lights_mot.night_light(lullaby.length(), period=10.0))
        return tk