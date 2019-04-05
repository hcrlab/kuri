import math, time, random, rospy, threading as tr
from gizmo.utils import interp as lo
from kuri_api.anim import track
from kuri_api import Lights
from gizmo.utils.rate import Rate
from .reactive_light.listening_light import ListeningLedPlayer
from .reactive_light.dance_light import MusicLedPlayer
import logging
logger = logging.getLogger(__name__)
ANIM_FREQUENCY = 60

class Light(object):
    """
    Top-level container for chest LED primitives
    
    Parameters
    ----------
    lights_svc:     light service
    """

    def __init__(self, lights_svc):
        self._lights_svc = lights_svc

    def circle_boot(self, length):
        """ spins forever (until canceled or preempted) """
        pts = []
        for i in range(Lights.NUM_LEDS - 1):
            p = [
             (0, 0, 0)] * Lights.NUM_LEDS
            p[i + 1] = (255, 255, 255)
            pts.append(p)

        return LedPattern(self._lights_svc, pts, length, frequency=5)

    def off(self, length):
        return self.animate(self._lights_svc.get_pixels(), Lights.ALL_OFF, lo.quad_out, length)

    def flash(self, color, frequency):
        return Flash(self._lights_svc, [
         color] * Lights.NUM_LEDS, frequency)

    def glow_pattern(self, pattern, fade_length, hold_length):
        fade_up = self.animate(self._lights_svc.get_pixels(), pattern, lo.quad_in, fade_length)
        hold_on = LedPattern(self._lights_svc, [pattern], hold_length, hold=True)
        return fade_up + hold_on

    def glow(self, r, g, b, fade_length, hold_length):
        final_pat = [
         (
          r, g, b)] * Lights.NUM_LEDS
        fade_up = self.animate(self._lights_svc.get_pixels(), final_pat, lo.quad_in, fade_length)
        hold_on = LedPattern(self._lights_svc, [final_pat], hold_length, hold=True)
        return fade_up + hold_on

    def white_glow(self, v, fade_length, hold_length):
        return self.glow(v, v, v, fade_length, hold_length)

    def white_pulse(self, length, frequency=0.5):
        return Pulse(self._lights_svc, [
         (255, 255, 255)] * Lights.NUM_LEDS, frequency, length)

    def pulse(self, color, pulse_up_time=0.3, pulse_down_time=0.3, indeces=None, loop=False):
        if not indeces:
            indeces = Lights.LED_MID_RING + Lights.LED_CENTER
        color_pattern = list(Lights.ALL_OFF)
        for index in indeces:
            color_pattern[index] = color

        off_on = self.animate(Lights.ALL_OFF, color_pattern, lo.quad_in, pulse_up_time, loop=loop)
        on_off = self.animate(color_pattern, Lights.ALL_OFF, lo.quad_out, pulse_down_time, loop=loop)
        pulse = off_on + on_off
        pulse.set_length(pulse_up_time + pulse_down_time)
        return pulse

    def animate_patterns(self, patterns, fps=ANIM_FREQUENCY, loop=False, hold=False):
        length = float('inf') if loop else len(patterns) / float(fps)
        return LedPattern(self._lights_svc, patterns, length, fps, hold)

    def animate(self, start_pat, end_pat, func, length, fps=ANIM_FREQUENCY, loop=False, hold=False):
        """
        Given a start pattern, end pattern and a linear interpolation function
        return a series of patterns that interpolates between the two.
        
        Parameters
        ----------
        start_pat (list of n 3 tuples): pattern to start with
        end_pat (list of n 3 tuples): pattern to end at
        func (f(float) -> float): interpolation func (see gizmo.utils.interp)
        length (float): num secs
        fps (int): frame rate
        loop (bool): whether we should generate a looping pattern
        """
        nframes = int(math.ceil(length * fps))
        patterns = lo.interp_pat(start_pat, end_pat, func, nframes)
        return self.animate_patterns(patterns, fps=fps, loop=loop, hold=hold)

    def heartbeat(self, color, fade_off_color, period):
        """
        Heartbeat LED primitive.
        Color intensifies from bottom to middle, random spike at top.
        NOTE: only on p3.
        :param: color The primary heartbeat color.
        :param: period The length of time that a sigle heartbeat should take.
        When fast the animation is 2*period with the top LED bridging the
        two lower heartbeats
        """
        led = Lights
        off = (0, 0, 0)
        bottom_frames = int(math.ceil(period * ANIM_FREQUENCY))
        low_glow_frame = int(math.ceil(0.4 * period * ANIM_FREQUENCY))
        low_deglow_frame = bottom_frames
        low_pat = lo.interp_keyframes([
         lo.KeyFrame(0, [fade_off_color], lo.quad_in),
         lo.KeyFrame(low_glow_frame, [color], lo.quad_in),
         lo.KeyFrame(low_deglow_frame, [fade_off_color], lo.quad_out)])
        mid_lowglow_frame = int(math.ceil(0.267 * period * ANIM_FREQUENCY))
        mid_glow_frame = int(math.ceil(0.534 * period * ANIM_FREQUENCY))
        mid_deglow_frame = int(math.ceil(0.869 * period * ANIM_FREQUENCY))
        mid_end_frame = bottom_frames
        middle_pat = lo.interp_keyframes([
         lo.KeyFrame(0, [off], lo.quad_in),
         lo.KeyFrame(mid_lowglow_frame, [fade_off_color], lo.quad_in),
         lo.KeyFrame(mid_glow_frame, [color], lo.quad_in),
         lo.KeyFrame(mid_deglow_frame, [fade_off_color], lo.quad_out),
         lo.KeyFrame(mid_end_frame, [off], lo.quad_out)])
        top_off_color = off
        top_period = period
        top_frames = bottom_frames
        if period < 2.0:
            top_off_color = fade_off_color
            top_period = 2 * period
            top_frames = 2 * bottom_frames
        top_off_frame = int(math.ceil(0.26 * top_period * ANIM_FREQUENCY))
        top_lowglow_frame = int(math.ceil(0.39 * top_period * ANIM_FREQUENCY))
        top_glow_frame = int(math.ceil(0.603 * top_period * ANIM_FREQUENCY))
        top_deglow_frame = int(math.ceil(0.801 * top_period * ANIM_FREQUENCY))
        top_endoff_frame = int(math.ceil(0.86 * top_period * ANIM_FREQUENCY))
        top_pat = lo.interp_keyframes([
         lo.KeyFrame(0, [top_off_color], lo.quad_in),
         lo.KeyFrame(top_off_frame, [top_off_color], lo.quad_in),
         lo.KeyFrame(top_lowglow_frame, [fade_off_color], lo.quad_in),
         lo.KeyFrame(top_glow_frame, [color], lo.quad_in),
         lo.KeyFrame(top_deglow_frame, [fade_off_color], lo.quad_out),
         lo.KeyFrame(top_endoff_frame, [top_off_color], lo.quad_out),
         lo.KeyFrame(top_frames, [top_off_color], lo.quad_out)])
        top_index = random.choice([
         led.IDX_OUTER_UPPER_MID_RIGHT,
         led.IDX_OUTER_UPPER_TOP_RIGHT,
         led.IDX_OUTER_UPPER_TOP_LEFT,
         led.IDX_OUTER_UPPER_MID_LEFT])
        base_pattern = list(led.ALL_OFF)
        if period < 2.0:
            base_pattern[led.IDX_OUTER_UPPER_MID_RIGHT] = list(fade_off_color)
            base_pattern[led.IDX_OUTER_UPPER_TOP_RIGHT] = list(fade_off_color)
            base_pattern[led.IDX_OUTER_UPPER_TOP_LEFT] = list(fade_off_color)
            base_pattern[led.IDX_OUTER_UPPER_MID_LEFT] = list(fade_off_color)
        heartbeat_pattern = []
        for top_frameno in range(top_frames):
            frame = list(base_pattern)
            frameno = top_frameno % bottom_frames
            frame[led.IDX_OUTER_BOTTOM_LOW_RIGHT] = low_pat[frameno][0]
            frame[led.IDX_OUTER_BOTTOM_LOW_LEFT] = low_pat[frameno][0]
            frame[led.IDX_INNER_BOTTOM_RIGHT] = middle_pat[frameno][0]
            frame[led.IDX_INNER_BOTTOM_LEFT] = middle_pat[frameno][0]
            frame[top_index] = top_pat[top_frameno][0]
            heartbeat_pattern += [frame]

        return LedPattern(self._lights_svc, heartbeat_pattern, period)

    def listening_light(self):
        """
        Reactive listening LED primitive.
        """
        return ListeningPattern(self._lights_svc)

    def dance_light(self):
        """
        Dance LED primitive.
        """
        return MusicLedPattern(self._lights_svc)

    def blink_regular(self, color):
        off = self.animate(Lights.all_leds(color), Lights.ALL_OFF, lo.quad_out, 0.17)
        on = self.animate(Lights.ALL_OFF, Lights.all_leds(color), lo.quad_out, 0.21)
        return off + on

    def night_light(self, length=10.0, period=2.0):
        off_on = self.animate(Lights.ALL_HALF, Lights.ALL_ON, lo.quad_in, period / 2.0, loop=True)
        on_off = self.animate(Lights.ALL_ON, Lights.ALL_HALF, lo.quad_out, period / 2.0, loop=True)
        pulse = off_on + on_off
        pulse.set_length(length)
        return pulse

    def happy_birthday(self, length=10.0, period=2.0):
        half_blue = [(0, 0, 127)] * Lights.NUM_LEDS
        off_on = self.animate(half_blue, Lights.ALL_ON, lo.quad_in, period / 2.0, loop=True)
        on_off = self.animate(Lights.ALL_ON, half_blue, lo.quad_out, period / 2.0, loop=True)
        pulse = off_on + on_off
        pulse.set_length(length)
        return pulse

    def recording(self, length=10.0):
        off_on = self.animate(Lights.ALL_OFF, Lights.all_leds(Lights.RED), lo.quad_in, 1.0, loop=True)
        on_off = self.animate(Lights.all_leds(Lights.RED), Lights.ALL_OFF, lo.quad_out, 1.0, loop=True)
        pulse = off_on + on_off
        pulse.set_length(length)
        return pulse


class MusicLedPattern(track.Content):
    """
    Content mock for Music LED pattern.
    """

    def __init__(self, lights_svc):
        super(MusicLedPattern, self).__init__()
        self._lights_svc = lights_svc

    def play(self):
        p = MusicLedPlayer(self, self._lights_svc)
        p.start()
        return p

    def length(self):
        return 'inf'

    def mux_set(self):
        return set([self._lights_svc])


class LedPlayer(track.Player):

    def __init__(self, content, chestlight, done_cb=None):
        super(LedPlayer, self).__init__(content)
        self._chestlight = chestlight
        self._signal_cancel = tr.Event()
        self._done_cb = done_cb

    def cancel(self):
        self._signal_cancel.set()
        self.join(timeout=0.5)

    def run(self):
        start_time = time.time()
        cur_time = 0.0
        idx = 0
        r = Rate(self._content._frequency)
        with self._chestlight as (light):
            while cur_time <= self._content.length():
                if idx == len(self._content._patterns):
                    if self._content._hold:
                        remaining = self._content.length() - cur_time
                        if self._signal_cancel.wait(remaining):
                            break
                    else:
                        idx = 0
                if r.sleep(self._signal_cancel.wait):
                    break
                try:
                    light.put_pixels(self._content._patterns[idx])
                except rospy.ROSException:
                    return

                idx += 1
                cur_time = time.time() - start_time

            if self._done_cb and not self._signal_cancel.is_set():
                self._done_cb()


class ListeningPattern(track.Content):
    """
    Content mock for Listening LED pattern.
    """

    def __init__(self, lights_svc):
        super(ListeningPattern, self).__init__()
        self._lights_svc = lights_svc

    def play(self):
        p = ListeningLedPlayer(self, self._lights_svc)
        p.start()
        return p

    def length(self):
        return 'inf'

    def mux_set(self):
        return set([self._lights_svc])


class LedPattern(track.Content):
    """
        Parameters
        ----------
    
        patterns (list of list of 3-tuples and boolean): for each led the
            3-tuple specify the rgb value and the boolean specifies whether to
            interpolate
    
        length (float): num secs to play this animation if inf
            then play forever.
    
        frequency (int): frame rate
    
        hold (bool): If true, hold the last frame rather than looping
    """

    def __init__(self, lights_svc, patterns, length, frequency=ANIM_FREQUENCY, hold=False):
        super(LedPattern, self).__init__()
        self._lights_svc = lights_svc
        self._patterns = patterns
        self._length = length
        self._frequency = frequency
        self._hold = hold

    def mux_set(self):
        return set([self._lights_svc])

    def play(self, done_cb=None):
        p = LedPlayer(self, self._lights_svc, done_cb)
        p.start()
        return p

    def set_length(self, l):
        self._length = l

    def length(self):
        return self._length

    def __add__(self, other):
        """
            Adds two animated sequences.
        
            Frequencies have to match and if either has inf length then the
            result will have inf length.
        
            Only the second pattern can have hold true
        """
        joined_pat = self._patterns + other._patterns
        if other._frequency != self._frequency:
            raise TypeError(('Tried to add LedPattern object with different frequencies ({} and {})').format(self._frequency, other._frequency))
        if self._hold:
            raise TypeError('Tried to add to LedPattern which holds')
        if math.isinf(other._length) or math.isinf(self._length):
            return LedPattern(self._lights_svc, joined_pat, float('inf'), self._frequency, other._hold)
        length = len(joined_pat) / float(self._frequency)
        return LedPattern(self._lights_svc, joined_pat, length, self._frequency, other._hold)

    def __str__(self):
        return ('LedPattern len={}s').format(round(self._length, 3))


class Pulse(LedPattern):

    def __init__(self, lights_svc, pattern, pulse_frequency, length):
        super(Pulse, self).__init__(lights_svc, [pattern,
         Lights.ALL_OFF], length=length, frequency=pulse_frequency * 2)
        self._pulse_frequency = pulse_frequency


class Flash(LedPattern):

    def __init__(self, lights_svc, pattern, pulse_frequency):
        super(Flash, self).__init__(lights_svc, [pattern, Lights.ALL_OFF], length=1.0 / pulse_frequency, frequency=pulse_frequency * 2)
        self._pulse_frequency = pulse_frequency