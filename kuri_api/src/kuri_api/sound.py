import abc, collections, logging, math, sys, threading, wave, alsaaudio

logger = logging.getLogger(__name__)


class SoundSource(object):
    """
    Plays sounds, e.g.:

    .. code:: python

        sound_src = SoundSource('me')
        sound = sound_src.play('/something/i/would/say.wav')
        sound_src.cancel(sound)

    """

    def __init__(self, name, card='default', delay=0.1, pcm_cls=None):
        self.name = name
        pcm_cls = pcm_cls or alsaaudio.PCM
        try:
            self._pcm = pcm_cls(alsaaudio.PCM_PLAYBACK, card=card)
        except alsaaudio.ALSAAudioError as ex:
            logger.warn(('unable to open pcm device for card="{}", not playing sounds - {}').format(card, ex))
            self._pcm = None

        self._daemon = SoundDaemon(self._pcm, delay=delay)
        self._player = None
        self._lock = threading.RLock()
        return

    @property
    def is_playing(self):
        return self._daemon.is_playing()

    def shutdown(self, timeout=None):
        self.cancel(timeout=timeout)
        self._daemon.shutdown()
        if self._pcm:
            self._pcm.close()
            self._pcm = None
        return

    def play(self, sound):
        player = SoundPlayer(self._daemon, sound)
        player.start()
        self._player = player
        return player

    def cancel(self, timeout=None):
        with self._lock:
            if self._player:
                self._player.cancel(timeout=timeout)
            self._player = None
        return


class SoundSources(collections.Sequence):

    def __init__(self, card='default', delay=0.1, pcm_cls=None):
        self.romoji = SoundSource('romoji', pcm_cls=pcm_cls, card=card, delay=delay)
        self.volume = SoundSource('volume', pcm_cls=pcm_cls, card=card, delay=delay)

    def __getitem__(self, index):
        return [
            self.romoji, self.volume][index]

    def __len__(self):
        return 2

    def shutdown(self):
        self.romoji.shutdown()
        self.volume.shutdown()


class SoundPlayer(object):
    """

    """

    def __init__(self, daemon, sound):
        if isinstance(sound, str):
            sound = WaveFile(sound)
        self._daemon = daemon
        self._sound = sound

    def start(self):
        self._daemon.play(self._sound)

    def length(self):
        return self._sound.length()

    def cancel(self, timeout=None):
        self._daemon.stop(self._sound)
        self.join(timeout)

    def join(self, timeout=None):
        pass

    def is_alive(self):
        return self._daemon.is_playing(self._sound)

    def __str__(self):
        return ('<{}({})>').format(self.__class__.__name__, self._sound)


class Sound(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def length(self):
        pass

    @abc.abstractmethod
    def read(self, num_frames):
        pass

    @abc.abstractproperty
    def frame_rate(self):
        pass

    @abc.abstractproperty
    def num_channels(self):
        pass

    @abc.abstractproperty
    def sample_width(self):
        """
        Number of samples per frame per channel in bytes
        """
        pass

    def time_of_frame(self, frame_number):
        return float(frame_number) / self.frame_rate


class WaveFile(Sound):

    def __init__(self, path):
        self._fname = path
        self._file = wave.open(self._fname, 'rb')

    @property
    def num_frames(self):
        return self._file.getnframes()

    @property
    def duration(self):
        return self.time_of_frame(self.num_frames)

    def length(self):
        return float(self.num_frames) / self.frame_rate

    def rewind(self):
        self._file.rewind()

    def __str__(self):
        return ('{}(num_frames={}, frame_rate={}, num_channels={}, sample_width={}, duration={:.2f})').format(
            type(self).__name__, self.num_frames, self.frame_rate, self.num_channels, self.sample_width, self.duration)

    @property
    def frame_rate(self):
        return self._file.getframerate()

    @property
    def num_channels(self):
        return self._file.getnchannels()

    @property
    def sample_width(self):
        return self._file.getsampwidth()

    def read(self, num_frames):
        return self._file.readframes(num_frames)


class SoundDaemon(threading.Thread):
    """
    Sound playing daemon thread. This can only play one sound
    at a time. After a call to shutdown(), the daemon can
    not be used again.
    """
    daemon = True

    def __init__(self, pcm, delay, poll=5.0):
        super(SoundDaemon, self).__init__(name='SoundDaemon')
        self._pcm = pcm
        self._delay = delay
        self._wake = threading.Event()
        self._sound = None
        self._shutdown = False
        self._poll = poll
        self._buffer_size_frames = None
        return

    def play(self, sound):
        """
        Play the specified sound
        sound: a WaveFile
        """
        if not self.is_alive() and self._pcm:
            self.start()
        self._sound = sound
        self._wake.set()
        return sound

    def stop(self, sound=None):
        """
        Stops the specified sound if it is playing. If sound is None,
        stops any song.
        The sound daemon will sleep and wait for another sound if the
        sound is stopped.
        """
        if not sound or self._sound is sound:
            self._sound = None
        return

    def is_playing(self, sound=None):
        """
        Determines if the sound daemon is playing a sound
        sound:  None to see if the daemon is playing any sound
                The argument passed to play to see if the daemon is playing a
                specific sound
        return: True/False
        """
        if sound:
            return self._sound is sound
        return self._sound is not None

    def shutdown(self):
        if self.is_alive():
            self.stop()
            self._shutdown = True
            self._wake.set()
            self.join(1.0)
            if self.is_alive():
                raise RuntimeError("couldn't cancel sound daemon thread")

    def _config(self, sound):
        buffer_multipler = 2.0
        period_size_frames = int(math.ceil(sound.frame_rate * self._delay / buffer_multipler))
        period_size_bytes = period_size_frames * sound.num_channels * sound.sample_width
        buffer_size_frames = int(period_size_frames * buffer_multipler)
        endian = 'LE' if sys.byteorder == 'little' else 'BE'
        pcm_format = getattr(alsaaudio, ('PCM_FORMAT_S{}_{}').format(str(sound.sample_width * 8), endian))
        self._pcm.setformat(pcm_format)
        self._pcm.setchannels(sound.num_channels)
        self._pcm.setrate(sound.frame_rate)
        self._pcm.setperiodsize(period_size_bytes)
        self._buffer_size_frames = buffer_size_frames

    def run(self):
        while True:
            if self._wake.wait(self._poll):
                self._wake.clear()
            if self._shutdown:
                break
            sound = self._sound
            if not sound:
                continue
            self._config(sound)
            while sound is self._sound:
                samples = sound.read(self._buffer_size_frames)
                if not samples:
                    if self._sound is sound:
                        self._sound = None
                    break
                self._pcm.write(samples)

        return