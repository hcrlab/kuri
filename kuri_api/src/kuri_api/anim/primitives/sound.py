import os
import threading

from assets import config
from kuri_api.anim import track
from kuri_api.sound import WaveFile

sounds_path = config.get_sounds_path()


class Sound(object):
    """
    Top-level container for sound primitives

    Parameters
    ----------

    sounds_svc:   sound service

    Example
    -------
        t = Track()
        s = Sound(sounds_svc=Sound())
        t.add(1.0, s.open('Got_It.wav'))
        t.play()
    """

    def __init__(self, sounds_svc):
        self._sounds_svc = sounds_svc

    def open(self, name):
        return SoundFile(self._sounds_svc, os.path.join(sounds_path, name))


class SoundPlayer(track.Player):

    def __init__(self, sounds_svc, content, done_cb=None):
        super(SoundPlayer, self).__init__(content)
        self._source = sounds_svc
        self._cancel_sig = threading.Event()
        self._done_cb = done_cb

    def run(self):
        self._source.play(self._content.wav)
        self._cancel_sig.wait(self._content.length())
        if self._done_cb and not self._cancel_sig.is_set():
            self._done_cb()

    def cancel(self, timeout=1.0):
        self._source.cancel()
        self._cancel_sig.set()
        self.join(timeout)
        assert not self.is_alive(), ('Failed to cancel SoundPlayer: {}').format(self._content)


class SoundFile(track.Content):
    """
        Class which loads a sound from a filename and plays it.

        Parameters
        ----------

        sounds_svc:   sound service
        filename: str, name of file to play
    """

    def __init__(self, sounds_svc, filename):
        self._sounds_svc = sounds_svc
        if os.path.isfile(filename):
            self.wav = WaveFile(filename)
        else:
            print("{} doesn't exist; couldn't load sound".format(filename))
            self.wav = None

    def play(self, done_cb=None):
        p = SoundPlayer(self._sounds_svc, self, done_cb)
        p.start()
        return p

    def length(self):
        if not self.wav:
            return 0
        else:
            return self.wav.duration

    def __str__(self):
        return str(self.wav)
