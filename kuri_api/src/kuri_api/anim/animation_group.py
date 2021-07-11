from kuri_api.anim.primitives.head import HeadMotions
from kuri_api.anim.primitives.light import Light
from kuri_api.anim.primitives.movies import Movies
from kuri_api.anim.primitives.sound import Sound
from kuri_api.anim.primitives.wheels import WheelMotions


class AnimationGroup(object):
    """
    All classes that define animations should subclass this.
    It instantiates the motion primitives depending on what is passed into
    the constructor.
    """

    def __init__(self, head=None, wheels=None, chest_leds=None, sound_srcs=None, movies=None):
        self.head = head
        self.head_mot = None if self.head is None else HeadMotions(self.head)
        self.wheels_mot = None if wheels is None else WheelMotions(wheels)
        self.lights_mot = None if chest_leds is None else Light(chest_leds)
        self.sound_mot = None if sound_srcs is None else Sound(sound_srcs)
        if movies:
            self.movies = movies
        else:
            if self.lights_mot and self.sound_mot:
                self.movies = Movies(lights=self.lights_mot, sound=self.sound_mot)
            else:
                self.movies = None
        return
