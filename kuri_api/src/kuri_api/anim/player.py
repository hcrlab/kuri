import logging
import threading
from collections import Counter

logger = logging.getLogger(__name__)


def get_animations_in_class(anim):
    """
    Returns the name of all animations in a class.
    """
    anim_funcs = []
    anim_funcs[:] = (x for x in dir(anim) if x not in vars(anim).keys() and callable(getattr(anim, x)))
    return filter(lambda x: not x.startswith('_'), anim_funcs)


def anim_clash_detected(anims):
    """
    Check for clashes in naming of animation tracks.
    """
    all_funcs = [f for anim_class in anims for f in get_animations_in_class(anim_class)]
    clash = len(all_funcs) != len(set(all_funcs))
    if clash:
        clash_anims = [item for item, count in Counter(all_funcs).items() if count > 1]
        logger.error(('Animation clash: {}').format(clash_anims))
    return clash


def build_animation_map(anims):
    """
    Build a map of animation names to track objects.
    """
    return {x: a_cls for a_cls in anims for x in get_animations_in_class(a_cls)}


def parse_animations(anims):
    """
    Parses a list of tracks into a track map {'anim_name': anim_class}.
    """
    if not isinstance(anims, list):
        anims = [
            anims]
    if anim_clash_detected(anims):
        raise KeyError('Animations have clashing names')
    return build_animation_map(anims)


class AnimationPlayer(object):
    """
    Player for canned `Animations`. It just tracks and auto-plays a single
    animation `Track` at a time, e.g.

    .. code:: python

        from gizmo.services import AnimationPlayer

        ap = AnimationPlayer(animations=..., assembler=...)
        ap.i_love_you()
        ap.cancel()

    """

    def __init__(self, animations, assembler=None, animation_sets=None):
        """
        :param animations is for programatic animations
        :param assembler is the assembler for JSON/atom animations
        :param animation_sets groups animations from the previous two so that
            they can easily be switched among
        """
        self._lock = threading.RLock()
        self._assembler = assembler
        self._animation_map = parse_animations(animations)
        self._animation_sets = animation_sets
        self.player = None
        self._active_anim = None
        self._active_anim_name = None
        return

    def shutdown(self):
        self.cancel(timeout=5.0)

    @property
    def is_playing(self):
        with self._lock:
            return self.player and self.player.is_alive()

    def cancel(self, timeout=1.0):
        with self._lock:
            if self.is_playing:
                self.player.cancel(timeout=timeout)
                if self.player.is_alive():
                    raise RuntimeError('player could not be cancelled')
                self.player = None
                self._active_anim = None
                self._active_anim_name = None
        return

    def _get_assembled_animation(self, anim):
        jname = ('{}.json').format(anim)
        track = self._assembler.animation_track_from_container_file(jname)
        return track

    def _loop_autoplay(self, anim, *args, **kwargs):
        force_cancel = kwargs.pop('force_cancel', False)
        loop_cb = kwargs.pop('loop_cb', None)
        if force_cancel:
            self.cancel()
        if self._active_anim_name is not anim:
            func = self._select(anim)
            if func is None:
                return
            self.cancel()
            track = func(*args, **kwargs)
            if track:
                self._active_anim = track
                self._active_anim_name = anim
            with self._lock:
                if track:
                    self.player = self._active_anim.play(done_cb=loop_cb, looping=True)
                return self.player
        else:
            return self.set_next_anim(anim, *args, **kwargs)
        return

    def set_next_anim(self, anim, *args, **kwargs):
        with self._lock:
            is_last = kwargs.pop('is_last', False)
            func = self._select(anim)
            if func is None:
                return
            track = func(*args, **kwargs)
            self._active_anim_name = anim
            self._active_anim = track
            self.player.set_next_content(track, is_last)
            return self.player
        return

    def _autoplay(self, anim, *args, **kwargs):
        done_cb = kwargs.pop('done_cb', None)
        self.cancel()
        func = self._select(anim)
        if func is None:
            return
        track = func(*args, **kwargs)
        with self._lock:
            self.player = track.play(done_cb=done_cb)
            self._active_anim = track
            self._active_anim_name = anim
            return self.player
        return

    def _select(self, name):
        """
        Wrapper to select animations and fucntions from animation sets
        :param name:
        :return:
        """
        anim = name
        if self._animation_sets and name in self._animation_sets:
            anim = self._animation_sets.select_random(name)
        return self._name_to_animation(anim)

    def _name_to_animation(self, anim):
        """
        Given an anim name, return a function that builds the animation.
        This will build 'classic' and JSON animations

        :param anim: The name of the animation

        :returns: A function that when called returns the animation track
        """
        anim_class = self._animation_map.get(anim)
        if not anim_class:
            if self._assembler and self._assembler.check_animation_exists(anim):
                track = self._get_assembled_animation(anim)

                def func(*args, **kwargs):
                    return track

                return func
            return
        func = getattr(anim_class, anim)
        return func

    def __getattr__(self, name):

        def _autoplay(*args, **kwargs):
            should_loop = kwargs.pop('should_loop', False)
            if should_loop:
                return self._loop_autoplay(name, *args, **kwargs)
            return self._autoplay(name, *args, **kwargs)

        return _autoplay


class LoopingAnimationPlayer(object):
    """
    Player for looping `Animations`. It tracks and auto-plays a set of
    animation Tracks and loops them. If you call the same animation that is
    currently playing, it will not start again unless you pass the
    `force_cancel=True` argument.

    done_cb NOT supported. Make sure to cancel the player directly.

    .. code:: python

        from gizmo.services import LoopingAnimationPlayer

        ap = LoopingAnimationPlayer(animations=...)
        ap.anim_1()
        ap.anim_1()  # This animation will not restart
        ap.anim_1(force_cancel=True)  # This animation will restart
        ap.anim_2()  # This will cancel anim_1 and start anim_2
        ap.cancel()

    """

    def __init__(self, animations):
        self._lock = threading.RLock()
        self._animation_map = parse_animations(animations)
        self.player = None
        self.active_track = None
        self._active_track_name = None
        return

    def shutdown(self):
        self.cancel(timeout=5.0)

    @property
    def is_playing(self):
        with self._lock:
            return self.player and self.player.is_alive()

    def cancel(self, timeout=1.0):
        with self._lock:
            if self.is_playing:
                self.player.cancel(timeout=timeout)
                if self.player.is_alive():
                    raise RuntimeError('player could not be cancelled')
                self.player = None
                self.active_track = None
                self._active_track_name = None
        return

    def __getattr__(self, name):
        anim_class = self._animation_map.get(name, None)
        if not anim_class:
            return
        func = getattr(anim_class, name)

        def _autoplay(*args, **kwargs):
            force_cancel = kwargs.pop('force_cancel', False)
            loop_cb = kwargs.pop('loop_cb', None)
            if force_cancel:
                self.cancel()
            if self._active_track_name is not name:
                self.cancel()
                track = func(*args, **kwargs)
                self.active_track = track
                self._active_track_name = name
                with self._lock:
                    self.player = self.active_track.play(done_cb=loop_cb, looping=True)
                    return self.player
            else:
                with self._lock:
                    track = func(*args, **kwargs)
                    self.active_track = track
                    self.player.set_next_content(track)
                    return self.player
            return

        return _autoplay
