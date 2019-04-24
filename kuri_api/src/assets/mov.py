import glob, os, subprocess
from PIL import Image
import numpy as np
from . import config
from . import memoize
MOVIE_WIDTH_HEIGHT = 720

@memoize(lambda : config.get_assets_path())
def mov_to_pixels(mov_name):
    """
    These pixels are generated at build-time and cached through memoization.
    """
    mov_path = os.path.join(config.get_movies_path(), mov_name)
    return _pixels_for_file(mov_path)


@memoize(lambda : config.get_assets_path())
def mov_to_wav(mov_name):
    """
    These wavs are generated at build-time and cached through memoization.
    """
    mov_path = os.path.join(config.get_movies_path(), mov_name)
    return _wav_for_file(mov_path)


def _pixels_for_file(mov_path):
    """
    Parse a video file to pixel values for the LED
    :param: mov_path Absolute path to movie file
    :return: Pixel values
    """
    _, fname = os.path.split(mov_path)
    mov_name = os.path.splitext(fname)[0]
    subprocess.call([
     'avconv', '-y', '-i', mov_path, ('/tmp/{}%06d.png').format(mov_name)])
    imgs_list = glob.glob(('/tmp/{}*.png').format(mov_name))
    imgs_list.sort()
    pixels = [ img_to_pix(img_name) for img_name in imgs_list ]
    for img_name in imgs_list:
        subprocess.call(['rm', img_name])

    return pixels


def _wav_for_file(mov_path):
    """
    Extract a wav file from a video file.
    :param: mov_path Absolute path to movie file
    :return: String representing name of wav file
    """
    _, fname = os.path.split(mov_path)
    mov_name = os.path.splitext(fname)[0]
    wav_name = os.path.splitext(mov_name)[0] + '.wav'
    wav_loc = os.path.join(config.get_sounds_path(), wav_name)
    ret = subprocess.call([
     'avconv', '-y', '-i', mov_path, '-vn', '-ar', '48000', '-ac', '2',
     '-ab', '192', '-f', 'wav', wav_loc])
    if ret:
        raise RuntimeError(('Unable to convert {} to wav at {}.').format(mov_path, wav_loc))
    return wav_name


def _add_pixel_ring(width, height, count, radius, angle):
    pixels = []
    x = float(width) / 2
    y = float(height) / 2
    for i in range(count):
        a = float(angle) + float(i) * 2 * np.pi / float(count)
        pixels.append((int(x - float(radius) * np.cos(a) + 0.5),
         int(y - float(radius) * np.sin(a) + 0.5)))

    return pixels


def pixel_positions():
    pixels = []
    width = height = MOVIE_WIDTH_HEIGHT
    pixels.extend(_add_pixel_ring(width, height, 1, 0, 0))
    pixels.extend(_add_pixel_ring(width, height, 6, 96, 8 * np.pi / 6))
    pixels.extend(_add_pixel_ring(width, height, 8, 280, 9 * np.pi / 8))
    return pixels


def img_to_pix(fname):
    pixels = pixel_positions()
    img = Image.open(fname)
    return [ img.getpixel(p) for p in pixels ]
