import abc, threading as tr, time


class Content(object):
    """
        Interface for playable content. Each content object creates and returns
        a thread that plays its content.
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def play(self, done_cb=None):
        """
            Returns instance of Player that plays content
            through a thread.
        """
        pass

    @abc.abstractmethod
    def length(self):
        """
            Total time it'll take to play this content.
        """
        pass

    def mux_set(self):
        """
            A set of muxes that should be held while a Track with this
            content is playing -- this way when multiple content objects
            a played in succession they don't lose the mux in between them.
        """
        return set()


class Player(tr.Thread):

    def __init__(self, content):
        super(Player, self).__init__(name='Player')
        self._content = content

    def length(self):
        if self._content:
            return self._content.length()
        return 0.0

    def __str__(self):
        return str(self._content)

    @abc.abstractmethod
    def cancel(self):
        pass


class Track(Content):
    """
        Represents a time indexed sequence of different playable Content
        objects.

        Parameters
        ----------

        content_list (optional): list of Content objects to play in sequence
    """

    def __init__(self, content_list=None):
        self._schedule = []
        if content_list is not None:
            t = 0
            for obj in content_list:
                self.add(t, obj)
                t += obj.length()

        return

    def add(self, time_start, obj):
        """
            Add object that will start at a specific time
        """
        self._schedule.append((time_start, obj))

    def length(self):
        if len(self._schedule) == 0:
            return 0.0
        finish_times = [t + o.length() for t, o in self._schedule
                        ]
        finish_times.sort()
        return finish_times[-1]

    def play(self, done_cb=None, looping=False):
        ltp = TrackPlayer(self, done_cb=done_cb, looping=looping)
        ltp.start()
        return ltp

    def schedule(self):
        return sorted(list(self._schedule), key=lambda x: x[0])

    def __repr__(self):
        return ('Track: {}').format(self._schedule)

    def mux_set(self):
        s = set()
        for t, o in self._schedule:
            s = s | o.mux_set()

        return s

    def acquire_muxes(self):
        content_muxes = self.mux_set()
        for mux in content_muxes:
            mux.acquire()

    def release_muxes(self):
        content_muxes = self.mux_set()
        for mux in content_muxes:
            mux.release()


class TrackPlayer(Player):
    """
        Player for ContentTrack objects
    """

    def __init__(self, content_obj, done_cb=None, looping=False):
        super(TrackPlayer, self).__init__(content_obj)
        self._content_obj = content_obj
        self._next_content_obj = None
        self._is_last_content = False
        self._players = []
        self._players_lock = tr.RLock()
        self._next_content_lock = tr.RLock()
        self._start_time = None
        self._done_cb = done_cb
        self._signal_cancel = tr.Event()
        self._cv = tr.Condition()
        self._num_active_players = 0
        self._looping = looping
        self._playing = False
        return

    def time(self):
        """
        Gets the time that the TrackPlayer has been playing for, or 0 if
        the track player has not started playing
        """
        if self._start_time:
            return time.time() - self._start_time
        return 0.0

    def run(self):
        self._playing = True
        schedule = self._content_obj.schedule()
        if not len(schedule):
            return
        self._content_obj.acquire_muxes()
        try:
            while self._playing:
                self._schedule_track_player(schedule)
                self._wait_done()
                if self._signal_cancel.is_set() or not self._looping:
                    self._playing = False
                else:
                    with self._next_content_lock:
                        if self._next_content_obj:
                            old_content = self._content_obj
                            self._content_obj = self._next_content_obj
                            self._content_obj.acquire_muxes()
                            old_content.release_muxes()
                            schedule = self._content_obj.schedule()
                            self._next_content_obj = None
                        if self._is_last_content:
                            self._looping = False

        finally:
            self._clear_active()
            self._content_obj.release_muxes()

        return

    def set_next_content(self, content_obj, is_last=False):
        with self._next_content_lock:
            self._next_content_obj = content_obj
            self._is_last_content = is_last

    def _schedule_track_player(self, schedule):
        with self._players_lock:
            self._clear_active()
            if self._looping:
                self._clear_all()
        self._start_time = time.time()
        for sec, obj in schedule:
            cur_time = self.time()
            if self._signal_cancel.wait(sec - cur_time):
                break
            with self._players_lock:
                self._num_active_players += 1
                self._players.append(obj.play(done_cb=self._player_finished))

    def _wait_done(self):
        """ wait for all objects to finish, for a timeout, or for cancel() """
        extratime = 1.0
        expiration = self._start_time + self._content_obj.length() + extratime
        with self._cv:
            while True:
                if self._signal_cancel.is_set():
                    break
                dur = expiration - time.time()
                if dur < 0:
                    break
                self._cv.wait(dur)
                if self._num_active_players == 0:
                    break

        if self._done_cb and not self._signal_cancel.is_set():
            self._done_cb()

    def _player_finished(self):
        with self._cv:
            self._num_active_players -= 1
            self._cv.notify_all()

    def _active_content(self):
        with self._players_lock:
            return [p for p in self._players if p.is_alive()]

    def _clear_active(self):
        for p in self._active_content():
            p.cancel()

        self._num_active_players = 0

    def _clear_all(self):
        with self._players_lock:
            for p in self._players:
                p.cancel()

            self._players = []
        self._num_active_players = 0

    def cancel(self, timeout=1.0):
        self._signal_cancel.set()
        self._clear_active()
        with self._cv:
            self._cv.notify_all()
        self.join(timeout)
        assert not self.is_alive(), ('could not cancel: {}').format(self._content_obj)