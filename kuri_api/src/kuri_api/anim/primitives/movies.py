from PIL import Image

from assets import memoize
from assets import mov_to_pixels, mov_to_wav
import os

assets_path = "/opt/gizmo/share/assets"
if os.path.isdir(assets_path):
    assets_path = "/tmp/"


class Movies(object):

    def __init__(self, lights, sound):
        self.lights = lights
        self.sound = sound

    def to_animated(self, mov_name, loop=False):
        fps = 60.0
        pixels = mov_to_pixels(mov_name)
        return self.lights.animate_patterns(pixels, fps, loop=loop)

    def to_sound(self, mov_name):
        return self.sound.open(mov_to_wav(mov_name))

    @memoize(lambda : assets_path)
    def get_pixels(self, filename):
        r"""
        \param filename: the animated gif file to extract the pixels from
        \output:         a list of (time, [(r, g, b), ...]) entries.
                         the pixel order is:
                            0. center
                            1. rightmost
                            2. ... and then counter-clockwise
        """
        gif = Image.open(filename)
        pixels = [
         (256, 256),
         (386, 255),
         (320, 144),
         (192, 144),
         (128, 258),
         (192, 366),
         (320, 366)]
        led_frames = []
        fps = None
        for f in frames(gif):
            pal = palette(gif.getpalette())
            led_frames.append(([ pal[gif.getpixel(p)] for p in pixels ], True))
            fps = 1000.0 / f.info['duration']

        return self.lights.Animated(led_frames, fps, len(led_frames) / fps)


def frames(gif):
    """
        A generator for gif frames
        Example:
            im = PIL.open('image.gif')
            for frame in frames(im):
                ...
    """
    while True:
        yield gif
        try:
            gif.seek(gif.tell() + 1)
        except EOFError:
            raise StopIteration


def palette(p):
    r"""
    \param p: a flat list of [r, g, b, r, g, b, ...]
    \output : a list of tuples [(r, g, b), (r, g, b), ...]
    """
    return zip(p[0::3], p[1::3], p[2::3])