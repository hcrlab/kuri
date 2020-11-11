import math
import sys
import threading as tr
import time
import traceback as tb

from kuri_api.anim import track
from kuri_api.head import Head
from numpy import clip


class HeadPlayer(track.Player):

    def __init__(self, svc, content, done_cb=None):
        super(HeadPlayer, self).__init__(content)
        self._svc = svc
        self._content_head = content
        self._done_cb = done_cb
        self._cv = tr.Condition()
        self._signal_cancel = tr.Event()
        self._trajectories_remaining = 0
        self._printlock = tr.Lock()

    def run(self):
        numtraj = 0
        with self._svc as (head):
            if self._content_head.pan_traj or self._content_head.tilt_traj:
                numtraj += 1
                head.pan_and_tilt_sequence(self._content_head.pan_traj, self._content_head.tilt_traj,
                                           done_cb=self._head_cb)
            if self._content_head.eyes_traj:
                numtraj += 1
                head.eyes_sequence(self._content_head.eyes_traj, done_cb=self._eye_cb)
            if numtraj > 0:
                self._start_time = time.time()
                self._trajectories_remaining = numtraj
                self._wait_done()

    def _wait_done(self):
        extratime = 0.5
        expiration = self._start_time + self._content_head.length() + extratime
        with self._cv:
            while True:
                if self._signal_cancel.is_set():
                    break
                duration = expiration - time.time()
                if duration < 0:
                    break
                self._cv.wait(duration)
                if self._trajectories_remaining == 0:
                    break

        if self._done_cb and not self._signal_cancel.is_set():
            self._done_cb()

    def cancel(self, timeout=1.0):
        self._signal_cancel.set()
        self._svc.cancel()
        with self._cv:
            self._cv.notify_all()
        self.join(timeout)
        assert self._thread_is_dead(), 'Could not cancel: {}.  HeadPlayer thread ID was `{}`'.format(
            self._content_head, self.ident)

    def _thread_is_dead(self):
        """
        Checks that our thread is not alive.
        If it's still alive, print diagnostic information to help
        debug the problem

        This was added to debug a fuzz test failure.  When this is no longer
        a problem, this method can be removed.  We're mainly interested
        in the side-effect for debugging.  Seeing if the thread is alive or
        not is trivial

        See PM-4010
        """
        alive = self.is_alive()
        if alive:
            with self._printlock:
                print 'Content length was {}'.format(self._content_head.length())
                print '\n*** STACKTRACE - START ***\n'
                code = []
                for thread_id, stack in sys._current_frames().items():
                    code.append('\n# ThreadID: %s' % thread_id)
                    for fname, lineno, name, ln in tb.extract_stack(stack):
                        code.append('File: "%s", line %d, in %s' % (fname,
                                                                    lineno,
                                                                    name))
                        if ln:
                            code.append('  %s' % ln.strip())

                for line in code:
                    print line

                print '\n*** STACKTRACE - END ***\n'
        return not alive

    def _head_cb(self, x, y):
        with self._cv:
            self._trajectories_remaining -= 1
            self._cv.notify()

    def _eye_cb(self, x, y):
        with self._cv:
            self._trajectories_remaining -= 1
            self._cv.notify()


class HeadMotion(track.Content):

    def __init__(self, svc, pan_traj=None, tilt_traj=None, eyes_traj=None):
        self._svc = svc
        self.pan_traj = pan_traj or []
        self.tilt_traj = tilt_traj or []
        self.eyes_traj = eyes_traj or []

    def play(self, done_cb=None):
        p = HeadPlayer(self._svc, self, done_cb)
        p.start()
        return p

    def length(self):
        return max(zip(*(self.pan_traj + self.tilt_traj + self.eyes_traj))[0])

    def __str__(self):
        return 'HeadMotion ({},{},{}) len={}s'.format('pan' if self.pan_traj else '   ',
                                                      'tlt' if self.tilt_traj else '   ',
                                                      'eye' if self.eyes_traj else '   ', round(self.length(), 3))


class HeadMotions(object):
    LOOK_AT_USER = Head.TILT_NEUTRAL - 0.05

    def __init__(self, svc):
        self._svc = svc

    def pantilt(self, pan, tilt, duration=0.5):
        """
        Move the head's pan and tilt to desired angles (radians) with an
        optional duration
        Ensures the pan and tilt angles are in the valid range
        """
        pan = clip(pan, Head.PAN_RIGHT, Head.PAN_LEFT)
        tilt = clip(tilt, Head.TILT_UP, Head.TILT_DOWN)
        return HeadMotion(self._svc, pan_traj=[
            (
                duration, pan)], tilt_traj=[
            (
                duration, tilt)])

    def lookat(self, pan, duration=0.5):
        """
        Move the head's pan to a desired angle (radians) with an optional
        duration
        Ensures the pan angle is in the valid range
        """
        pan = clip(pan, Head.PAN_RIGHT, Head.PAN_LEFT)
        return HeadMotion(self._svc, pan_traj=[
            (
                duration, pan)])

    def neutral(self, duration=0.46):
        """
        Moves the head to the neutral position with an optional time.
        """
        return HeadMotion(self._svc, pan_traj=[
            (
                duration, Head.PAN_NEUTRAL)], tilt_traj=[
            (
                duration, Head.TILT_NEUTRAL)])

    def moveeyes(self, position, time=0.17):
        """
        Moves the eyes to a position with an optional time.
        Ensures the position is in the valid range for the eyes
        """
        position = clip(position, Head.EYES_HAPPY, Head.EYES_CLOSED)
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, position)])

    def openeyes(self, time=0.17, amplitude=1.0):
        amplitude = clip(amplitude, 0.0, 1.0)
        eyes = (1.0 - amplitude) * (Head.EYES_CLOSED - Head.EYES_OPEN) + Head.EYES_OPEN
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, eyes)])

    def closeeyes(self, time=0.17, amplitude=1.0):
        amplitude = clip(amplitude, 0.0, 1.0)
        eyes = amplitude * (Head.EYES_CLOSED - Head.EYES_OPEN) + Head.EYES_OPEN
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, eyes)])

    def blinkeyes(self, open_amplitude=1.0, close_amplitude=1.0, open_time=0.21, close_time=0.12,
                  eyes_closed_blink=Head.EYES_CLOSED_BLINK):
        close_eyes = close_amplitude * (eyes_closed_blink - Head.EYES_OPEN) + Head.EYES_OPEN
        open_eyes = open_amplitude * (Head.EYES_OPEN - eyes_closed_blink) + eyes_closed_blink
        return HeadMotion(self._svc, eyes_traj=[
            (
                close_time, close_eyes),
            (
                open_time, open_eyes)])

    def blink(self, speed=0.0, done_eye_position=None):
        blink_close_time = 0.13
        blink_open_time = 0.26
        speed = clip(speed, -3.0, 3.0)
        speed_add = speed / 100.0
        blink_close_time -= speed_add
        blink_open_time -= speed_add
        done_pos = done_eye_position or Head.EYES_OPEN
        return HeadMotion(self._svc, eyes_traj=[
            (
                blink_close_time, Head.EYES_CLOSED),
            (
                blink_open_time, done_pos)])

    def happyeyes(self, time=0.25):
        """
        Sets the eyes to the happy position with an optional time.
        """
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, Head.EYES_HAPPY)])

    def sadeyes(self, time=0.25):
        """
        Sets the eyes to the sad position with an optional time.
        """
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, Head.EYES_SUPER_SAD)])

    def happyposture(self, time=0.25):
        """
        Sets the eyes and head tilt to the happy posture with an optional time.
        """
        return HeadMotion(self._svc, tilt_traj=[
            (
                time, Head.TILT_NEUTRAL - 0.18)], eyes_traj=[
            (
                time, Head.EYES_HAPPY)])

    def sadposture(self, time=0.25):
        """
        Sets the eyes and head tilt to the sad posture with an optional time.
        """
        return HeadMotion(self._svc, eyes_traj=[
            (
                time, Head.EYES_SUPER_SAD)], tilt_traj=[
            (
                time, Head.TILT_NEUTRAL)])

    def reset(self):
        """
        Reset the pan, tilt, and eyes to neutral positions.
        """
        return HeadMotion(self._svc, pan_traj=[
            (
                0.3, Head.PAN_NEUTRAL)], tilt_traj=[
            (
                0.3, Head.TILT_NEUTRAL)], eyes_traj=[
            (
                0.3, Head.EYES_OPEN)])

    def headwiggle(self, pan_reference, tilt):
        """
        Wiggle the head horizontally around a given pan with a reference tilt.
        """
        gain = 4.0
        return HeadMotion(self._svc, pan_traj=[
            (
                0.33, pan_reference + gain * math.radians(-5)),
            (
                0.71, pan_reference + gain * math.radians(3.4)),
            (
                1.0899999999999999, pan_reference + gain * math.radians(-2.85)),
            (
                1.3399999999999999,
                pan_reference + gain * math.radians(2.15))], tilt_traj=[
            (
                0.33, tilt),
            (
                0.71, tilt),
            (
                1.0899999999999999, tilt),
            (
                1.3399999999999999, tilt)])
