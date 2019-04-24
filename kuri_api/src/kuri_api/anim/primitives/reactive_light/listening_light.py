import threading, rospy
from numpy import clip
from kuri_api.lights import Lights
from assets import mov_to_pixels
from kuri_api.utils.pulse_utils import PeakMonitor
from kuri_api.utils.rate import Rate
import logging
logger = logging.getLogger(__name__)

class ListeningLedPlayer(threading.Thread):
    """
    Specialized thread (mocks track player's interface) for realtime LED
    control during listening state.
    """
    MOVIE_FREQUENCY = 60
    UPDATE_FREQUENCY = 60
    WINDOW_SIZE = 60
    MIC_SOURCE = 'system-in.monitor'
    MIC_CHANNELS = 1
    LOW_COLOR = (0, 26, 77)
    HI_COLOR = (0, 85, 255)
    STAGE_INTRO = 0
    STAGE_REACT = 1
    REACTIVE_LEDS = [
     Lights.IDX_CENTER,
     Lights.IDX_INNER_BOTTOM_LEFT,
     Lights.IDX_INNER_BOTTOM_RIGHT,
     Lights.IDX_INNER_UPPER_RIGHT,
     Lights.IDX_INNER_UPPER_LEFT]

    def __init__(self, content, chestlight):
        super(ListeningLedPlayer, self).__init__()
        self._content = content
        self._chestlight = chestlight
        self._signal_cancel = threading.Event()
        self._content_lock = threading.RLock()
        self._stage = self.STAGE_INTRO
        self._intro_index = 0
        self._react_base = self._build_react_base()
        self._intro_pixels = self._build_intro()
        self._num_frames_intro = len(self._intro_pixels)
        self._level = 0
        self._last_display_level = 0
        self._peak_monitor = PeakMonitor(source_name=self.MIC_SOURCE, source_chans=self.MIC_CHANNELS, window_hz=self.WINDOW_SIZE)

    def cancel(self):
        self._peak_monitor.shutdown()
        self._signal_cancel.set()
        self.join(timeout=0.5)

    def run(self):
        self._peak_monitor.start()
        with self._chestlight as (light):
            r = Rate(self.MOVIE_FREQUENCY)
            while True:
                level = 0.0
                with self._content_lock:
                    self._level = self._peak_monitor.level()
                    level = float(self._level)
                pattern = list(Lights.ALL_OFF)
                if self._stage == self.STAGE_INTRO:
                    pattern = self._overlay_intro(pattern)
                    self._intro_index += 1
                else:
                    if self._stage == self.STAGE_REACT:
                        pattern = list(self._react_base)
                        pattern = self._apply_reactive(pattern, level)
                    try:
                        light.put_pixels(pattern)
                    except rospy.ROSException:
                        return

                if self._intro_index == self._num_frames_intro:
                    r = Rate(self.UPDATE_FREQUENCY)
                    self._stage = self.STAGE_REACT
                    self._last_display_level = level
                    self._intro_index = 0
                if r.sleep(self._signal_cancel.wait):
                    break

    def _build_intro(self):
        """
        Builds a framebuffer of pixels for the listening intro.
        """
        return mov_to_pixels('listening_ack.mov')

    def _build_react_base(self):
        """
        Builds a base for reactive liistening state.
        """
        pattern = list(Lights.ALL_OFF)
        for out_ring_idx in Lights.LED_OUTER_RING:
            pattern[out_ring_idx] = self.LOW_COLOR

        pattern[Lights.IDX_CENTER] = self.LOW_COLOR
        return pattern

    def _overlay_intro(self, pattern):
        """
        Applies the frames from the intro to a base pattern.
        """
        intro_frame = self._intro_pixels[self._intro_index]
        for led_index in range(len(intro_frame)):
            pixel = intro_frame[led_index]
            if pixel != (0, 0, 0):
                pattern[led_index] = pixel

        return pattern

    def _apply_reactive(self, pattern, level):
        """
        Applies the reactive level to the base pattern.
        Audio levels below 0.025 are basically quiet, voice spikes above
        that. Loud, direct voice usually gets over 0.1
        Spike instantly but decay over time
        """
        DECAY_MAX = 0.035
        new_level = clip(level * 2.9, 0.0, 1.0)
        if new_level < self._last_display_level - DECAY_MAX:
            new_level = self._last_display_level - DECAY_MAX
        for led in self.REACTIVE_LEDS:
            pattern[led] = self._color_for_level(new_level)

        self._last_display_level = new_level
        return pattern

    def _color_for_level(self, level):
        """
        Scales the reactive listening color by the current sound level.
        """
        return (
         int(self.HI_COLOR[0] * level),
         int(self.HI_COLOR[1] * level),
         int(self.HI_COLOR[2] * level))