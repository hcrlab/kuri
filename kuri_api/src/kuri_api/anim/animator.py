import logging
import threading

from gizmo.utils.animation import AnimationParser
from gizmo.utils.animation import AnimationTrackAssembler
from kuri_api.anim import Track

logger = logging.getLogger(__name__)


class Animator(object):

    def __init__(self, comm_interface, head=None, wheels=None, sound_srcs=None, chest_leds=None):
        self._lock = threading.RLock()
        self.animation_assembler = AnimationTrackAssembler(head=head, wheels=wheels, sound_srcs=sound_srcs,
                                                           chest_leds=chest_leds)
        self.currentTrackPlayer = None
        return

    def play_animation_from_container_file(self, container_filename, alt_path=None, done_cb=None):
        track = self.animation_assembler.animation_track_from_container_file(container_filename, alt_path=alt_path)
        self._play_track(track, done_cb)

    def _play_track(self, track, done_cb=None):
        self.cancel()
        with self._lock:
            self.currentTrackPlayer = track.play(done_cb=done_cb)

    @property
    def is_playing(self):
        with self._lock:
            return self.currentTrackPlayer and self.currentTrackPlayer.isAlive()

    def cancel(self, timeout=1.0):
        with self._lock:
            if self.is_playing:
                self.currentTrackPlayer.cancel(timeout=timeout)
                if self.currentTrackPlayer.is_alive():
                    raise RuntimeError('player could not be cancelled')
                self.currentTrackPlayer = None
        return

    def play_live_animation(self, command):
        animation_command = command
        animation = AnimationParser.parse_animation(animation_command.json)
        if animation is not None and not animation.hasParseErrors:
            track = Track()
            self.animation_assembler.add_animation_to_track(animation, track)
            self._play_track(track)
            return self.currentTrackPlayer
        logger.error('Could not play animation because of parse error.')
        return

    def play_animation_from_container_json(self, container_json, done_cb=None):
        track = self.animation_assembler.animation_track_from_container_json(container_json)
        self._play_track(track, done_cb)

    def shutdown(self):
        self.cancel()
