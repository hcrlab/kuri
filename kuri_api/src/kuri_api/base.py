#! /usr/bin/env python
import actionlib
import rospy
import math
import copy
import rospy.rostime
import tf.transformations as tft
from geometry_msgs.msg import Twist
import nav_msgs.msg._Odometry
import mobile_base_driver.msg as mbdm
import threading
from kuri_api.utils import Mux, MuxChannel
from kuri_api.utils import Events


def normalize_angle_positive(angle):
    """Normalizes angle to be in [0, 2pi)
    """
    return (angle % (2.0 * math.pi)) % (2.0 * math.pi)


def normalize_angle(angle):
    """Normalizes angle to be in [-pi, pi)
    """
    pos = normalize_angle_positive(angle)
    if pos >= math.pi:
      return pos - 2.0 * math.pi
    return pos


def quat_heading(quat):
    x, y, z, w = (quat.x, quat.y, quat.z, quat.w)
    return math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))


trajectory_topic ='mobile_base/commands/wheel_traj'
controller='mobile_base_controller'
switch_service='controller_manager/switch_controller',


class Base(Events):
    """Base controls the mobile base portion of the Kuri robot.

    Sample usage:
        base = kuri_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    move_event = Events.source()
    arc_event = Events.source()

    def __init__(self):
        super(Base, self).__init__()

        self._odom_sub = rospy.Subscriber('/odom', nav_msgs.msg.Odometry, callback=self._odom_callback)

        # Move_base default
        velocity_topic = "/mobile_base/commands/velocity"
        # Are we not using sim time? Must be real robot
        if not rospy.get_param("use_sim_time", False):
            self.arc_move_client = ArcMove()
        else:
            self.arc_move_client = None
        self._traj_pub = rospy.Publisher(trajectory_topic, mbdm.WheelTraj, queue_size=1)

        self.vel_pub = rospy.Publisher(velocity_topic, Twist, queue_size=10)
        self.latest_odom = None
        self.latest_pose = None
        self.latest_yaw = None
        self.no_odom_received = True

    def _odom_callback(self, msg):
        self.no_odom_received = False
        self.latest_odom = msg
        self.latest_yaw = quat_heading(msg.pose.pose.orientation)
        self.latest_pose = msg.pose.pose.position

    def send_trajectory(self, traj):
        """
            traj -- list of tuples (time, linear, angular)
        """
        if not self._traj_pub:
            rospy.logerr("Tried to send a trajectory in simulation, but this functionality is only present on the real robot.")
            return
        t = mbdm.WheelTraj()
        for p in traj:
            t.points.append(mbdm.WheelTrajPoint(time_from_start=rospy.Duration(p[0]), linear_vel=p[1], angular_vel=p[2]))

        self._traj_pub.publish(t)
    
    def go_forward(self, distance, speed):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        if self.arc_move_client:
            # We don't want duration to be a limit, so set it to a high number
            self.arc_move_client.arc_move(angle=0.0, angular_velocity=0.0, arc_len=distance, linear_velocity=speed,
                                    duration=1000.0, done_cb=lambda : self.arc_event('done'))

            return


        # rospy.sleep until the base has received at least one message on /odom
        while self.no_odom_received and not rospy.is_shutdown():
            rospy.loginfo_throttle(10, "Waiting for odom message before moving...")
            rospy.sleep(0.3)

        # Record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.latest_odom)
        start_pose = start.pose.pose.position

        # Check if the robot has traveled the desired distance
        distance_error = abs(distance)
        direction = -1 if distance < 0 else 1
        rate = rospy.Rate(50)
        while distance_error > 0.01 and not rospy.is_shutdown():
            p = min(speed, speed * distance_error + 0.1)
            self.move(direction * p, 0)
            # We're controlling the distance that the robot travels, trying to make it as close to abs(distance) as possible
            distance_error =  abs(distance) - math.sqrt(((self.latest_pose.x - start_pose.x) ** 2)+((self.latest_pose.y - start_pose.y) ** 2))
            rate.sleep()
        self.stop()
        
    def rotate_by(self, angular_distance, velocity, duration):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """

        if self.arc_move_client:
            self.arc_move_client.arc_move(angle=angular_distance, angular_velocity=velocity, arc_len=0.0, linear_velocity=0.0,
                                    duration=duration, done_cb=lambda : self.arc_event('done'))
            return


        # rospy.sleep until the base has received at least one message on /odom
        while self.no_odom_received and not rospy.is_shutdown():
            rospy.loginfo_throttle(10, "Waiting for odom message before moving...")
            rospy.sleep(0.3)

        start_yaw = normalize_angle_positive(quat_heading(self.latest_odom.pose.pose.orientation)) 
        last_yaw = start_yaw       
        target_yaw = normalize_angle_positive(start_yaw + angular_distance) 

        # How much we have left to go. When we haven't moved, we're either
        # already spot on, or the error is the angular distance provided in the command
        yaw_error = normalize_angle_positive(target_yaw - start_yaw)

        # We'll always turn in the direction of the command, even if it would be
        # shorter to go the other way.
        # This will cause issues if we overshoot, because we'll just go back aroud 
        # the long way.
        direction = -1 if angular_distance < 0 else 1   
        rate = rospy.Rate(50)
        # We'll tolerate an error slightly smaller than a degree
        while yaw_error > 0.01 and not rospy.is_shutdown():
            # Simple proportional control with fixed direction
            # Make control proportional to the error, with min of 0.1 and max of `speed`
            p = min(velocity, velocity * yaw_error + 0.1)
            self.move(0, direction * p)
            last_yaw = normalize_angle_positive(self.latest_yaw)
            yaw_error = normalize_angle_positive(target_yaw - last_yaw)
            rate.sleep()
        self.stop()

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist Message
        msg = Twist()

        # Fill out message
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        # Publish message
        self.vel_pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.vel_pub.publish(msg)

    def get_yaw(self):
        return self.latest_yaw

    def get_pose(self):
        return self.latest_pose


class ArcMove(object):
    """
    Interface to the driver's rotate_by action
    """
    TOPIC = 'mobile_base/arc_move'

    def __init__(self):
        self.ac = actionlib.ActionClient(self.TOPIC, mbdm.ArcMoveAction)
        self.goal = None
        self.gh = None
        self._lock = threading.RLock()
        return

    def arc_move(self, angle, angular_velocity, arc_len, linear_velocity, duration, done_cb=None):
        """ start rotating to the desired angle

            :param angle: [rad] a positive angle means a counter-clockwise
                               rotation. Can rotate full turns and more.
            :param angular_velocity: [float] a velocity to rotate at
            :param arc_len: [float] arc_length of the curve we want
                            the robot to traverse in meters
            :param linear_velocity velocity we want the robot
                   to traverse the arc
            :param duration: [float] a maximum duration to rotate for
            :param done_cb: fun(success) a callback when rotate is done,
                                         `success` will be False if the
                                         roatation could not complete by
                                         the specified duration
        """

        def transition_cb(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if id(gh_goal) != id(self.goal) or gh.get_comm_state() != actionlib.CommState.DONE:
                return
            with self._lock:
                self.goal = None
            if done_cb:
                done_cb()
            return

        self.cancel()
        with self._lock:
            self.goal = mbdm.ArcMoveGoal(angle=angle, angular_velocity=angular_velocity, arc_len=arc_len,
                                         linear_velocity=linear_velocity, duration=duration)
            self.gh = self.ac.send_goal(self.goal, transition_cb)

    def cancel(self):
        gh_to_cancel = None
        with self._lock:
            gh_to_cancel = self.gh
            self.gh = None
            self.goal = None
        if gh_to_cancel:
            gh_to_cancel.cancel()
        return

    def shutdown(self):
        if self.ac:
            self.cancel()
            self.ac.status_sub.unregister()
            self.ac.result_sub.unregister()
            self.ac.feedback_sub.unregister()
            self.ac = None
        return

class BaseMux(Mux):

    class Channel(Base, MuxChannel):

        class __metaclass__(Base.__metaclass__, MuxChannel.__metaclass__):
            pass

        def __init__(self, mux, name, priority, *args, **kwargs):
            Base.__init__(self, *args, **kwargs)
            MuxChannel.__init__(self, mux, name, priority)

        request_event = Events.source()

        def move(self, linear_speed, angular_speed):
            if not self.is_active:
                self.move_event((linear_speed, angular_speed))
                if not self.is_acquired:
                    self.request_event('wheels_request')
                return False
            return super(type(self), self).move(linear_speed, angular_speed)

        def send_trajectory(self, traj):
            if not self.is_active:
                return False
            return super(type(self), self).send_trajectory(traj)

        def go_forward(self, distance, speed):
            if not self.is_active:
                return False

            return super(type(self), self).go_forward(distance, speed)

        def rotate_by(self, angle, velocity, duration):
            if not self.is_active:
                self.arc_event('done')
                return False
            return super(type(self), self).rotate_by(angle, velocity, duration)

        def arc_move(self, arc_len, linear_velocity, angle, angular_velocity, duration):
            if not self.is_active:
                self.arc_event('done')
                return False
            return super(type(self), self).arc_move(arc_len, linear_velocity, angle, angular_velocity, duration)

        def stop(self):
            if not self.is_active:
                return False
            return super(type(self), self).stop()

        def on_override(self):
            if not self.is_active and self.is_acquired:
                super(BaseMux.Channel, self).stop()

        def get_current_pose(self):
            return super(BaseMux.Channel, self).get_pose()

        def get_current_yaw(self):
            return super(BaseMux.Channel, self).get_yaw()

    class __metaclass__(Events.__metaclass__, Mux.__metaclass__):
        pass

    priority = [
     'nav',
     'teleop',
     'romoji',
     'safety']

    def __init__(self, priority=None):
        Mux.__init__(self)
        priority = priority or self.priority
        self.safety = self.Channel(self, 'safety', self.priority.index('safety') or -1)
        self.teleop = self.Channel(self, 'teleop', self.priority.index('teleop') or -1)
        self.nav = self.Channel(self, 'nav', self.priority.index('nav') or -1,)
        self.animations = self.Channel(self, 'animations', self.priority.index('romoji') or -1)

    def shutdown(self):
        for channel in [self.safety, self.teleop, self.nav, self.romoji]:
            channel.shutdown()