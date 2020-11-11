import random
import rospy
import threading

from assets import mov_to_pixels
from kuri_api.lights import Lights
from kuri_api.utils.dance import get_bpm_range
from kuri_api.utils.rate import Rate
from numpy import interp


class MusicLedPlayer(threading.Thread):
    """
    Specialized thread (mocks track player's interface) for realtime LED
    control during music playback.

    This pattern picks from 6 random snake animations and picks a random
    color for the snake. The snake runs around the chest LED over a blue
    background.
    """
    DEFAULT_FRAMERATE = 24
    FRAMERATE_RANGE = (20, 60)
    NUM_SNAKES = 6
    PINK = (255, 0, 128)
    ORANGE = (255, 78, 0)
    MINT = (0, 249, 184)
    YELLOW = (255, 224, 0)
    SNAKE_COLORS = [PINK, ORANGE, MINT, YELLOW]
    NUM_SNAKE_COLORS = len(SNAKE_COLORS)
    INNER_COLOR = (0, 0, 160)
    OUTER_COLOR = (0, 0, 42)

    def __init__(self, content, chestlight):
        super(MusicLedPlayer, self).__init__()
        self._content = content
        self._chestlight = chestlight
        self._background_color = self.INNER_COLOR
        self._snakes = self._build_snakes()
        self._random_snake()
        self._base_pattern = self._build_base()
        self._framerate = self.DEFAULT_FRAMERATE
        self._signal_cancel = threading.Event()
        self._content_lock = threading.RLock()
        self._bpm = 0
        self._energies = None
        return

    def cancel(self):
        self._signal_cancel.set()
        self.join(timeout=0.5)

    def run(self):
        r = Rate(self._framerate)
        with self._chestlight as (light):
            while True:
                with self._content_lock:
                    bpm = self._bpm
                pattern = list(self._base_pattern)
                pattern = self._apply_snake(pattern)
                try:
                    light.put_pixels(pattern)
                except rospy.ROSException:
                    return

                self._update()
                self._framerate = self._framerate_for_bpm(bpm)
                r = Rate(self._framerate)
                if r.sleep(self._signal_cancel.wait):
                    break

    def set_info(self, energies=None, bpm=None):
        """
        Sets the parameters of the generated pattern.
        :param: level The volume level in the range [0:1.0].
        :param: bpm The beats-per-second of the detected music
        """
        with self._content_lock:
            if energies:
                self._energies = energies
            if bpm:
                self._bpm = bpm

    def _update(self):
        """
        Updates some internal state after each pass of the run() loop.
        Determines whether the snake has finished. If so, generates a new
        random snake for the next pass.
        """
        self._current_snake_frame_index += 1
        num_frames_in_snake = len(self._current_snake)
        snake_finished = self._current_snake_frame_index == num_frames_in_snake
        if snake_finished:
            self._random_snake()

    def _build_snakes(self):
        """
        Builds snake pixels from cached movies.
        """
        snakes = []
        for i in range(1, self.NUM_SNAKES + 1):
            snake_mov = ('snake{}.mov').format(i)
            snake = mov_to_pixels(snake_mov)
            snakes.append(snake)

        return snakes

    def _build_base(self):
        """
        Constructs a base that the visualization will overlay on top of.
        """
        pattern = list(Lights.ALL_OFF)
        for out_ring_idx in Lights.LED_OUTER_RING:
            pattern[out_ring_idx] = self.OUTER_COLOR

        for in_ring_idx in Lights.LED_MID_RING:
            pattern[in_ring_idx] = self.INNER_COLOR

        return pattern

    def _random_snake(self):
        """
        Updates the internal state with a new random snake.
        """
        self._current_snake_index = random.randint(0, self.NUM_SNAKES - 1)
        self._current_snake = self._snakes[self._current_snake_index]
        self._current_snake_frame_index = 0
        self._current_snake_color = self.SNAKE_COLORS[random.randint(0, self.NUM_SNAKE_COLORS - 1)]

    def _apply_snake(self, pattern):
        """
        Applies a snake over a base pattern.
        """
        snake_frame = self._current_snake[self._current_snake_frame_index]
        active_pixels = []
        for led_index in range(len(snake_frame)):
            pixel = snake_frame[led_index]
            if pixel != (0, 0, 0):
                active_pixels.append((led_index, pixel))

        for idx, px in active_pixels:
            pattern[idx] = self._scale_color(px)

        return pattern

    def _scale_color(self, pixel):
        """
        Scales a snake color to an intensity.
        """
        return (
            int(pixel[0] / 255.0 * self._current_snake_color[0]),
            int(pixel[1] / 255.0 * self._current_snake_color[1]),
            int(pixel[2] / 255.0 * self._current_snake_color[2]))

    def _framerate_for_bpm(self, bpm):
        """
        Maps a bpm to an LED update framerate.
        """
        return int(interp(bpm, get_bpm_range(), self.FRAMERATE_RANGE))
