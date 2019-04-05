import itertools as it, threading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from kuri_api.anim import track

class WheelMotions:
    """
    Top-level container for wheel motion primitives
    
    Parameters
    ----------
    wheels:     wheel service
    """
    DEFAULT_TRANS_VEL = 0.08

    def __init__(self, wheels):
        self.wheels = wheels

    def _create(self, cls, *args, **kwargs):
        m = cls(*args, **kwargs)
        m.wheel_svc(self.wheels)
        return m

    def inch(self, *args, **kwargs):
        return self._create(Inch, *args, **kwargs)

    def rotate(self, *args, **kwargs):
        return self._create(Rotate, *args, **kwargs)

    def rotate_by(self, *args, **kwargs):
        return self._create(RotateBy, *args, **kwargs)

    def arc_move(self, *args, **kwargs):
        return self._create(Arc, *args, **kwargs)

    def bodywiggle(self, *args, **kwargs):
        return self._create(BodyWiggle, *args, **kwargs)

    def stop(self, *args, **kwargs):
        return self._create(StopWheelMotion, *args, **kwargs)


def trig_profile(length):
    r"""
        A triangular velocity profile going from 0.0 to 1.0
        and then back to 0.0
    
        \params length: number of points in the profile
    """
    points = []
    points += [ 2.0 * x / (length - 1) for x in range(length / 2) ]
    points += [ 2.0 - 2.0 * x / (length - 1) for x in range(length / 2, length) ]
    return points


class WheelMotion(track.Content):
    """
        A basic motion object.
        Points are instances of Twist
        Interval is fixed (in [s])
    """

    def __init__(self):
        self._twists = []
        self._interval = 0.0
        self._wheel_svc = None
        return

    def wheel_svc(self, service):
        self._wheel_svc = service

    def play(self, done_cb=None):
        p = WheelMotionPlayer(self, self._wheel_svc, done_cb)
        p.start()
        return p

    def length(self):
        return len(self._twists) * self._interval


class WheelMotionPlayer(track.Player):

    def __init__(self, wheel_motion, wheel_svc, done_cb=None):
        super(WheelMotionPlayer, self).__init__(wheel_motion)
        self._wheel_svc = wheel_svc
        self._signal_cancel = threading.Event()
        self._done_cb = done_cb

    def run(self):
        if self._wheel_svc is None:
            return
        with self._wheel_svc as (wheels):
            traj = [ (i * self._content._interval, t.linear.x, t.angular.z) for i, t in enumerate(self._content._twists) ]
            wheels.send_trajectory(traj)
            self._signal_cancel.wait(self._content.length())
            if self._done_cb and not self._signal_cancel.is_set():
                self._done_cb()
            wheels.send_trajectory([(0.0, 0.0, 0.0)])
        return

    def cancel(self):
        self._signal_cancel.set()
        self.join()


class ArcMotion(track.Content):
    """
        A rotation and translation motion object
    """

    def __init__(self):
        self._rotate_radians = None
        self._arc_len = None
        self._arc_move_length = None
        self._wheel_svc = None
        return

    def wheel_svc(self, service):
        self._wheel_svc = service

    def play(self, done_cb=None):
        p = ArcMotionPlayer(self, self._wheel_svc, done_cb)
        p.start()
        return p

    def length(self):
        return self._arc_move_length


class ArcMotionPlayer(track.Player):
    FIXED_ACCEL_TIME = 0.1
    VEL_COMP = 0.2

    def __init__(self, wheel_moiton, wheel_svc, done_cb=None):
        super(ArcMotionPlayer, self).__init__(wheel_moiton)
        self._wheel_svc = wheel_svc
        self._done_cb = done_cb
        self._cancel_requested = False

    def run(self):
        if self._wheel_svc is None:
            return
        with self._wheel_svc as (wheels):
            wheels.arc_event.connect(self._done)
            angle = self._content._rotate_radians
            arc_len = self._content._arc_len
            duration = self._content._arc_move_length
            if duration == 0.0:
                if self._done_cb:
                    self._done_cb()
                return
            angular_velocity = 0
            if duration - self.FIXED_ACCEL_TIME <= 0:
                angular_velocity = angle / duration
            else:
                if angle / duration <= 1.0:
                    angular_velocity = angle / (duration - self.FIXED_ACCEL_TIME)
                    angular_velocity += self.VEL_COMP * angular_velocity
                else:
                    angular_velocity = angle / (duration - self.FIXED_ACCEL_TIME)
                    duration -= self.FIXED_ACCEL_TIME
            linear_velocity = arc_len / duration
            linear_velocity += linear_velocity * self.VEL_COMP
            wheels.arc_move(angle=angle, angular_velocity=angular_velocity, arc_len=arc_len, linear_velocity=linear_velocity, duration=duration)
        return

    def _done(self, msg):
        self._wheel_svc.arc_event.disconnect(self._done)
        if self._done_cb and not self._cancel_requested:
            self._done_cb()

    def cancel(self):
        self._cancel_requested = True
        if self._wheel_svc:
            self._wheel_svc.stop()


class RotateMotion(track.Content):
    """
        A basic rotation motion object.
    """

    def __init__(self):
        self._rotate_radians = None
        self._rotate_length = None
        self._wheel_svc = None
        return

    def wheel_svc(self, service):
        self._wheel_svc = service

    def play(self, done_cb=None):
        p = RotateMotionPlayer(self, self._wheel_svc, done_cb)
        p.start()
        return p

    def length(self):
        return self._rotate_length


class RotateMotionPlayer(track.Player):
    FIXED_ACCEL_TIME = 0.1
    VEL_COMP = 0.2

    def __init__(self, wheel_motion, wheel_svc, done_cb=None):
        super(RotateMotionPlayer, self).__init__(wheel_motion)
        self._wheel_svc = wheel_svc
        self._done_cb = done_cb
        self._cancel_requested = False

    def run(self):
        if self._wheel_svc is None:
            return
        with self._wheel_svc as (wheels):
            wheels.arc_event.connect(self._done)
            angle = self._content._rotate_radians
            duration = self._content._rotate_length
            if duration == 0.0:
                if self._done_cb:
                    self._done_cb()
                return
            velocity = 0
            if duration - self.FIXED_ACCEL_TIME <= 0:
                velocity = angle / duration
            else:
                if angle / duration <= 1.0:
                    velocity = angle / (duration - self.FIXED_ACCEL_TIME)
                    velocity += self.VEL_COMP * velocity
                else:
                    velocity = angle / (duration - self.FIXED_ACCEL_TIME)
                    duration -= self.FIXED_ACCEL_TIME
            wheels.rotate_by(angle=angle, velocity=velocity, duration=duration)
        return

    def _done(self, msg):
        self._wheel_svc.arc_event.disconnect(self._done)
        if self._done_cb and not self._cancel_requested:
            self._done_cb()

    def cancel(self):
        self._cancel_requested = True
        if self._wheel_svc:
            self._wheel_svc.stop()


class Inch(WheelMotion):
    """
        Using a triangular velocity profile, go straight
        with the given speed for the given time
    """

    def __init__(self, speed, length):
        self._interval = 0.01
        self._twists = []
        for p in trig_profile(int(length / self._interval)):
            t = Twist()
            t.linear.x = p * speed
            self._twists.append(t)


class Rotate(WheelMotion):
    """
        Using a triangular velocity profile, turn in place
        with the given speed for the given time
    """

    def __init__(self, speed, length):
        super(Rotate, self).__init__()
        self._interval = 0.01
        self._twists = []
        for p in trig_profile(int(length / self._interval)):
            t = Twist()
            t.angular.z = p * speed
            self._twists.append(t)


class RotateBy(RotateMotion):
    """
    Rotate by a desired angle.
    """

    def __init__(self, angle, duration):
        super(RotateBy, self).__init__()
        self._rotate_radians = angle
        self._rotate_length = duration


class Arc(ArcMotion):

    def __init__(self, angle, arc_len, duration):
        super(Arc, self).__init__()
        self._rotate_radians = angle
        self._arc_len = arc_len
        self._arc_move_length = duration


class BodyWiggle(WheelMotion):
    """
    #Twist (-5) degrees .33 sec,
           (3.4) degrees .38sec,
           (-2.85) .38sec
           (2.15) .25sec
    """

    def __init__(self, direction=1):
        self._direction = direction
        self._interval = 0.05
        dilate = 1.0
        self._times = [ t * dilate for t in [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4] ]
        self._speeds = [2.5, -5.0, 5.0, -4.0, 4.0, -2.0, 2.0, -1.0]
        self._twists = self._wiggle_twists()

    def _wiggle_twists(self):
        gain = self._direction
        trajs = []
        for t, s in zip(self._times, self._speeds):
            trajs.append([ Twist(angular=Vector3(0, 0, gain * p * s)) for p in trig_profile(int(t / self._interval))
                         ])

        return it.chain(*trajs)

    def length(self):
        return sum(self._times)


class StopWheelMotion(track.Content):
    """
        Stops the wheels as a track
    """

    def __init__(self):
        self._wheel_svc = None
        return

    def wheel_svc(self, service):
        self._wheel_svc = service

    def play(self, done_cb=None):
        p = StopWheelPlayer(self, self._wheel_svc, done_cb)
        p.start()
        return p

    def length(self):
        return 0


class StopWheelPlayer(track.Player):

    def __init__(self, wheel_motion, wheel_svc, done_cb=None):
        super(StopWheelPlayer, self).__init__(wheel_motion)
        self._wheel_svc = wheel_svc
        self._done_cb = done_cb

    def run(self):
        if self._wheel_svc is None:
            return
        with self._wheel_svc as (wheels):
            wheels.send_trajectory([(0.0, 0.0, 0.0)])
            if self._done_cb:
                self._done_cb()
        return

    def _done(self, msg):
        if self._done_cb:
            self._done_cb()

    def cancel(self):
        self.join()