import collections, inspect, weakref


class EventField(object):

    def __init__(self, **defaults):
        self.name = None
        self.defaults = defaults
        return

    def __get__(self, obj, type=None):
        if obj is None:
            return self
        if self.name not in obj._event_srcs:
            obj._event_srcs[self.name] = EventSource(self.name, obj, self)
        return obj._event_srcs[self.name]


class _EventCallback(object):

    def __init__(self, func, **kwargs):
        self.func = func
        for k, v in kwargs.iteritems():
            setattr(self, k, v)

    def __call__(self, *args, **kwargs):
        return self.func(*args, **kwargs)

    def __cmp__(self, other):
        if isinstance(other, _EventCallback):
            other = other.func
        return cmp(self.func, other)


class EventSource(collections.Sequence):
    """
    Source of events. These are typically created by `EventField` attached as
    class attributes to `Events`, e.g.:

    .. code:: python

        from kuri_api.utils import Events
        from kuri_api.utils.events import EventField,  EventSource

        class MyEventEmitter(Events):

            event_type_1 = Events.source()

        assert(isinstance(MyEventEmitter.event_type_1, EventField))
        my_em = MyEventEmitter()
        assert(isinstance(my_em.event_type_1, EventSource))

    """

    def __init__(self, name, container, field):
        self.name = name
        self._c = weakref.proxy(container)
        self._f = field
        self._cbs = []

    def connect(self, cb, **kwargs):
        params = self._f.defaults.copy()
        params.update(kwargs)
        if len(self._cbs) == 0:
            self._c._first_event_connect(self._f)
        if params:
            cb = _EventCallback(cb, **params)
        self._cbs.append(cb)

    def is_connected(self, cb):
        return cb in self._cbs

    def disconnect(self, cb):
        if cb not in self._cbs:
            return
        self._cbs.remove(cb)
        if len(self._cbs) == 0:
            self._c._last_event_disconnect(self._f)

    def trigger(self, *args, **kwargs):
        for cb in self:
            cb(*args, **kwargs)

    __call__ = trigger

    def __len__(self):
        return len(self._cbs)

    def __getitem__(self, index):
        return self._cbs[index]


class EventsMeta(type):

    def __new__(cls, name, bases, dct):
        type_ = type.__new__(cls, name, bases, dct)

        def is_event_field(member):
            return isinstance(member, EventField)

        event_fields = inspect.getmembers(type_, is_event_field)
        for name, event_field in event_fields:
            event_field.name = name

        return type_


class Events(object):
    """
    Uses this as the base class for types that emit event, e.g.:

    .. code:: python

        from kuri_api.utils import Events

        class MyEventEmitter(Events):

            event_type_1 = Events.source()

            event_type_2 = Events.source()

            def _first_event_connect(self, event_type):
                print 'first connect for', event_type.name

            def _last_event_disconnect(self, event_type):
                print 'last disconnect for', event_type.name

        my_em = MyEventEmitter()

        def on_event_1(msg)
            print 'you said', msg

        my_em.event_1.connect(on_event_1)
        my_em.event_1('hi')
        my_em.event_1.disconnect(on_event_1)
        my_em.event_1('drop')

    """
    __metaclass__ = EventsMeta

    def __init__(self):
        self._event_srcs = {}

    @classmethod
    def source(cls, **defaults):
        return EventField(**defaults)

    def _first_event_connect(self, event):
        pass

    def _last_event_disconnect(self, event):
        pass