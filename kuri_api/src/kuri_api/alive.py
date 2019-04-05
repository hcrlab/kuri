import random
from threading import Lock, RLock
import time
from kuri_api.utils.timeouts import EventTimeout
from kuri_api.anim import AnimationPlayer
from kuri_api.anim.library.blink_animations import BlinkAnimations

class Alive(object):
    """
    Alive service, which keeps Kuri in character - she's alive.
    Blinks with a frequency rougly based on a model of how humans blink,
    sampling from a gaussian distribution.
    """
    BLINK_MIN = 2.8
    BLINK_MAX = 6.0
    BLINK_MEAN = 3.7
    BLINK_STD_DEV = 0.7
    AWARE_PAUSE_TIME = 6.0
    DOUBLE_BLINK_THRESHOLD = 0.08
    TRIPLE_BLINK_THRESHOLD = 0.01

    def __init__(self, head):
        super(Alive, self).__init__()
        self._head = head
        self._blink_timer = None
        self.blink_player = AnimationPlayer(animations=BlinkAnimations(head=self._head))
        self.blink_anim = None
        self._active = False
        self._aware_start = 0.0
        self._api_lock = Lock()
        self._lock = RLock()
        return

    def shutdown(self):
        with self._lock:
            if self.blink_player:
                self.blink_player.shutdown()
            if self._blink_timer:
                self._blink_timer.shutdown()

    def sleep(self):
        """
        Permanently disables the alive service.
        NOTE: calling this is discouraged. If your state runs an animation
        with eye movements use the @disable_alive decorator. If you are
        capturing from the camera call aware() instead.
        """
        with self._api_lock:
            with self._lock:
                if self._active:
                    self._active = False
                else:
                    return
            self._kill_timer(blocking=True)
            if self.blink_anim:
                self.blink_anim.join()

    def wakeup(self):
        """
        Enables the alive service.
        """
        with self._api_lock:
            if not self._active:
                self._active = True
                self._start_blink_timer()

    def aware(self):
        """
        Temporarily disables the alive service from blinking (for 6 seconds).
        """
        self._aware_start = time.time()

    def is_active(self):
        """
        True after 'wakeup' is called until 'sleep' is called.  Calls to
        'aware' do not affect the value of is_active
        """
        with self._lock:
            return self._active

    def is_aware(self):
        """
        True after 'aware' is called until AWARE_PAUSE_TIME has elapsed
        """
        return time.time() - self._aware_start < self.AWARE_PAUSE_TIME

    def _start_blink_timer(self):
        """
        Start the blink timer.
        """
        with self._lock:
            if self.is_active():
                self._kill_timer()
                next_blink_time = self._get_next_blink_time()
                self._blink_timer = EventTimeout(duration=next_blink_time, timeout_cb=self._blink_timeout)

    def _blink_timeout(self):
        if self._aware_start > time.time():
            self._aware_start = 0.0
        with self._lock:
            if not self.is_active():
                return
            if self.is_aware():
                self._start_blink_timer()
                return
            self._play_blink_anim()

    def _play_blink_anim(self):
        """
        We are going to blink.
        Double blink with a small probability.
        Triple blink with a VERY small probability.
        """
        blink_seed = random.random()
        if blink_seed < self.TRIPLE_BLINK_THRESHOLD:
            self.blink_anim = self.blink_player.triple_blink(done_cb=self._start_blink_timer)
        else:
            if blink_seed < self.DOUBLE_BLINK_THRESHOLD:
                self.blink_anim = self.blink_player.double_blink(done_cb=self._start_blink_timer)
            else:
                self.blink_anim = self.blink_player.blink(done_cb=self._start_blink_timer)

    def _get_next_blink_time(self):
        return max(self.BLINK_MIN, min(self.BLINK_MAX, random.normalvariate(self.BLINK_MEAN, self.BLINK_STD_DEV)))

    def _kill_timer(self, blocking=False):
        if self._blink_timer:
            self._blink_timer.shutdown(blocking=blocking)
            self._blink_timer = None
        return