import bisect, collections, inspect, threading


class Mux(collections.Sequence):

    def __init__(self):
        self._lock = threading.RLock()
        self._channels = []

    class protect(object):
        """
        Descriptor used to protect methods from being accessed from
        inactive channels.

        Parameters
        ----------

        fail:   return value when channel is *not* active.

        Example
        -------

            class A(object):

                def method(self):
                    pass

            class B(Mux):

                class Channel(A, MuxChannel):

                    method = Mux.protect(fail='nope')

                def __init__(self):
                    self.chan1 = self.Channel(self, 'chan1', 1)
                    self.chan2 = self.Channel(self, 'chan2', 2)

            b = B()
            with b.chan1 as obj:
                obj.method()

        """

        def __init__(self, fail=False):
            self.name = None
            self.func = None
            self.fail = fail
            return

        def __get__(self, obj, type=None):
            if obj is None:
                return self

            def wrap(*args, **kwargs):
                if not obj.is_active:
                    return self.fail
                return self.func(obj, *args, **kwargs)

            return wrap

    def __len__(self):
        return len(self._channels)

    def __getitem__(self, index):
        return self._channels[index]

    @property
    def current(self):
        if not self:
            return None
        return self[-1]

    def on_override(self, channel):
        channel.on_override()

    def acquire(self, channel):
        with self._lock:
            if channel in self._channels:
                return
            i = bisect.bisect_left(self._channels, channel)
            current_active = self.current
            should_notify = current_active and i == len(self._channels)
            self._channels.insert(i, channel)
        if should_notify:
            self.on_override(current_active)

    def release(self, channel):
        with self._lock:
            if channel in self._channels:
                channel.on_release()
                self._channels.remove(channel)


class MuxChannelMeta(type):

    def __new__(cls, name, bases, dct):
        type_ = type.__new__(cls, name, bases, dct)
        protects = inspect.getmembers(type_, lambda t: isinstance(t, Mux.protect))
        for n, protect in protects:
            protect.name = n
            for base in bases:
                if hasattr(base, n):
                    protect.func = getattr(base, n)
                    break
            else:
                raise TypeError(('{}.{} protects nothing').format(name, n))

        return type_


class MuxChannel(object):
    __metaclass__ = MuxChannelMeta

    def __init__(self, mux, name, priority):
        self._mux = mux
        self.name = name
        self.priority = priority
        self.acquire_lock = threading.RLock()
        self.acquire_count = 0

    def acquire(self):
        with self.acquire_lock:
            self.acquire_count += 1
            if self.acquire_count == 1:
                self._mux.acquire(self)

    @property
    def is_acquired(self):
        with self.acquire_lock:
            return self in self._mux

    def release(self):
        with self.acquire_lock:
            if self.acquire_count == 0:
                return
            self.acquire_count -= 1
            if self.acquire_count == 0:
                self._mux.release(self)

    def on_release(self):
        pass

    def on_override(self):
        """
        When the mux status change check for clean up
        if needed
        :return:
        """
        pass

    @property
    def is_active(self):
        with self.acquire_lock:
            return self.is_acquired and self._mux.current is self

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, type, value, tb):
        return self.release()

    def __cmp__(self, other):
        return cmp(self.priority, other.priority)

    def __str__(self):
        return ('{}(name="{}", priority={}, is_acquired={}, is_active={})').format(type(self).__name__, self.name,
                                                                                   self.priority, self.is_acquired,
                                                                                   self.is_active)