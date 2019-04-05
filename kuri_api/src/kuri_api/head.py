import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header
from actionlib.action_client import CommState
import control_msgs.msg, rospy, trajectory_msgs.msg
from kuri_api.utils import Mux, MuxChannel, Events
import math
import logging

def _fill_traj_blanks(pts, values):
    """ filter a trajectory by forward propagating missing values
        Example:
            a = _fill_traj_blanks([
                (0, None, None),
                (1, None, 37),
                (2, 42, None)
            ], [1, 2])  # == [(0, 1, 2), (1, 1, 37), (2, 42, 37)]
    """
    if not pts:
        return pts
    pt = pts[0]
    vals = tuple((x if 1 else y for x, y in zip(pt[1:], values) if x is not None))
    return [
     (
      pt[0],) + vals] + _fill_traj_blanks(pts[1:], vals)


def _merge_traj_points(pts):
    """ merge successive points occurring at the same time
    in that case, use values from last point """

    def filt(lst):
        if not lst or not lst[1:]:
            return lst
        if lst[0][0] == lst[1][0]:
            return filt([mrg(lst[0], lst[1])] + lst[2:])
        return [
         lst[0]] + filt(lst[1:])

    def mrg(fst, sec):
        vals = tuple((x if x is not None else y for x, y in zip(sec[1:], fst[1:])))
        return (fst[0],) + vals

    return filt(pts)

class Head(object):
    JOINT_PAN = 'head_1_joint'
    JOINT_TILT = 'head_2_joint'
    JOINT_EYES = 'eyelids_joint'
    JOINT_HEIGHT = 0.405
    PAN_LEFT = 0.78
    PAN_NEUTRAL = 0
    PAN_RIGHT = -PAN_LEFT
    TILT_UP = -0.92
    TILT_NEUTRAL = 0.0
    TILT_DOWN = 0.29
    EYES_OPEN = 0.0
    EYES_NEUTRAL = 0.1
    EYES_CLOSED = 0.41
    EYES_HAPPY = -0.16
    EYES_SUPER_SAD = 0.15
    EYES_CLOSED_BLINK = 0.35
    HEAD_NS = 'head_controller/follow_joint_trajectory'
    EYES_NS = 'eyelids_controller/follow_joint_trajectory'

    def __init__(self, joint_state, tf, head_ns=None, eyes_ns=None):
        self._head_gh = None
        self._head_goal = None
        self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, control_msgs.msg.FollowJointTrajectoryAction)
        self._eyes_gh = None
        self._eyes_goal = None
        self._eyes_ac = actionlib.ActionClient(eyes_ns or self.EYES_NS, control_msgs.msg.FollowJointTrajectoryAction)
        self._js = joint_state
        self._tf = tf
        return

    @property
    def cur_pan(self):
        return self._js.get_pan_pos()

    @property
    def cur_tilt(self):
        return self._js.get_tilt_pos()

    @property
    def cur_eyes(self):
        return self._js.get_eye_pos()

    def cancel(self):
        head_gh = self._head_gh
        eyes_gh = self._eyes_gh
        if head_gh:
            head_gh.cancel()
        self._head_goal = None
        self._head_gh = None
        if eyes_gh:
            eyes_gh.cancel()
        self._eyes_goal = None
        self._eyes_gh = None
        return

    def eyes_to(self, radians, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's eye lids to the specified location in the duration
        specified
        
        :param radians: The eye position.  Expected to be between
        HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
        
        :param duration: The amount of time to take to get the eyes to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        point = trajectory_msgs.msg.JointTrajectoryPoint([
         radians], [], [], [], duration)
        trajectory = trajectory_msgs.msg.JointTrajectory(joint_names=[
         self.JOINT_EYES], points=[point])
        return self.send_trajectory(trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def eyes_sequence(self, seq, **kwargs):
        """
        Generates and sends a `trajectory_msgs.msg.JointTrajectory` from a
        sequence of `(time_from_start, position)` pairs.
        """
        pts = list(seq)
        pts.sort(key=lambda pt: pt[0])
        pts = _merge_traj_points(pts)
        traj = trajectory_msgs.msg.JointTrajectory(joint_names=[
         self.JOINT_EYES], points=[ trajectory_msgs.msg.JointTrajectoryPoint([pos], [], [], [], time) for time, pos in pts
                                  ])
        return self.send_trajectory(traj, **kwargs)

    def is_done(self):
        active = {
         GoalStatus.PENDING, GoalStatus.RECALLING,
         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self._head_gh:
            if self._head_gh.get_goal_status() in active:
                return False
        if self._eyes_gh:
            if self._eyes_gh.get_goal_status() in active:
                return False
        return True

    def pan_and_tilt(self, pan, tilt, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's head to the point specified in the duration
        specified
        
        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
        and HeadClient.PAN_RIGHT
        
        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
        and HeadClient.TILT_DOWN
        
        :param duration: The amount of time to take to get the head to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        point = trajectory_msgs.msg.JointTrajectoryPoint([
         pan, tilt], [], [], [], duration)
        trajectory = trajectory_msgs.msg.JointTrajectory(joint_names=[
         self.JOINT_PAN, self.JOINT_TILT], points=[
         point])
        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def pan_and_tilt_sequence(self, pans, tilts, **kwargs):
        """
        Generates and sends a `trajectory_msgs.msg.JointTrajectory` from
        sequences of `(time_from_start, position)` pairs.
        """
        pts = []
        pts.extend((t, pan, None) for t, pan in pans or [])
        pts.extend((t, None, tilt) for t, tilt in tilts or [])
        pts.sort(key=lambda pt: pt[0])
        pts = _merge_traj_points(pts)
        pts = _fill_traj_blanks(pts, [self.cur_pan, self.cur_tilt])
        traj = trajectory_msgs.msg.JointTrajectory(joint_names=[
         self.JOINT_PAN, self.JOINT_TILT], points=[ trajectory_msgs.msg.JointTrajectoryPoint([pan_pt, tilt_pt], [], [], [], time) for time, pan_pt, tilt_pt in pts
                                                  ])
        return self.send_trajectory(traj, **kwargs)

    def send_trajectory(self, traj, feedback_cb=None, done_cb=None):
        """
        Sends the specified trajectories to the head and eye controllers
        
        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
        
        :param feedback_cb: A callable that takes one parameter - the feedback
        
        :param done_cb: A callable that takes two parameters - the goal status
        the goal handle result
        """
        for point in traj.points:
            for k in ('velocities', 'accelerations', 'effort'):
                if getattr(point, k) is None:
                    setattr(point, k, [])

            if isinstance(point.time_from_start, (int, float)):
                point.time_from_start = rospy.Duration(point.time_from_start)

        goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=traj)

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if done_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                if gh.get_comm_state() == CommState.DONE:
                    done_cb(gh.get_goal_status(), gh.get_result())
            return

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                feedback_cb(feedback)
            return

        if self.JOINT_EYES in traj.joint_names:
            if not self._eyes_ac:
                return False
            self._eyes_goal = goal
            self._eyes_gh = self._eyes_ac.send_goal(goal, _handle_transition, _handle_feedback)
        else:
            if not self._head_ac:
                return False
            self._head_goal = goal
            self._head_gh = self._head_ac.send_goal(goal, _handle_transition, _handle_feedback)
        return True

    def pose_stamped_to_pan_and_tilt(self, pose_stamped):
        """
        Converts a pose stamped to pan and tilt angles to make the center of
        the head point towards the given pose
        """
        pose_in_base_frame = self._tf.transform_pose_stamped(pose_stamped, 'base_footprint')
        if pose_in_base_frame is None:
            logging.error(("Couldn't transform from frame {} to base_footprint!").format(pose_stamped.header.frame_id))
            return False
        return self.position_to_pan_and_tilt(self.JOINT_HEIGHT, pose_in_base_frame.pose.position)

    @staticmethod
    def position_to_pan_and_tilt(joint_height, position):
        """
        Converts a position in base frame to pan and tilt angles to make the
        center of the head point towards the given pose
        """
        pan_angle = math.atan2(position.y, position.x)
        pose_in_head_1_neutral_z = position.z - joint_height
        r = (position.x ** 2 + position.y ** 2) ** 0.5
        tilt_angle = -math.atan2(pose_in_head_1_neutral_z, r)
        return (
            pan_angle, tilt_angle)

    def look_at_pose_stamped(self, pose_stamped, duration=1.0, **kwargs):
        """
        Send a position-based trajectory with a duration to make the robot
        look at a pose stamped
        """
        result = self.pose_stamped_to_pan_and_tilt(pose_stamped)
        if result is False:
            empty_header = Header()
            pose_stamped.header.stamp = empty_header.stamp
            result = self.pose_stamped_to_pan_and_tilt(pose_stamped)
        pan_angle, tilt_angle = result
        return self.pan_and_tilt(pan_angle, tilt_angle, duration, **kwargs)

    def look_at(self, nav_fb, tilt=0.0, rotation_default=0.7, min_target_dist=0.2):
        """
        Converts nav feedback messages into a head motion.
        """
        if nav_fb.cmd_vel.x == 0.0 and nav_fb.target_pose.x != float('inf'):
            if nav_fb.cmd_vel.theta < 0.0:
                return self.pan_and_tilt(-rotation_default, tilt)
            if nav_fb.cmd_vel.theta > 0.0:
                return self.pan_and_tilt(rotation_default, tilt)
            return self.pan_and_tilt(0.0, tilt)
        else:
            if not math.isnan(nav_fb.target_pose.x) and not math.isnan(
                    nav_fb.target_pose.y) and nav_fb.target_pose.x != float('inf') and abs(
                    nav_fb.target_pose.x) > min_target_dist:
                heading = math.atan2(nav_fb.target_pose.y, nav_fb.target_pose.x)
                return self.pan_and_tilt(heading, tilt)
            return self.pan_and_tilt(0.0, tilt)

    def shutdown(self):
        self.cancel()
        self._head_ac = None
        self._eyes_ac = None
        return

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()

        return False


class HeadMux(Mux):

    class Channel(Head, MuxChannel):

        def __init__(self, mux, name, priority, js, tf, namespace=None):
            Head.__init__(self, js, tf)
            MuxChannel.__init__(self, mux, name, priority)

        send_trajectory = Mux.protect(fail=False)

    priority = [
     'behavior',
     'teleop',
     'animations',
     'safety']

    def __init__(self, js, tf, priority=None):
        super(HeadMux, self).__init__()
        self.priority = priority or self.priority
        self.animations = self.Channel(self, 'animations', self.priority.index('animations'), js, tf)
        self.behavior = self.Channel(self, 'behavior', self.priority.index('behavior'), js, tf)
        self.teleop = self.Channel(self, 'teleop', self.priority.index('teleop'), js, tf)
        self.safety = self.Channel(self, 'safety', self.priority.index('safety'), js, tf)

    def shutdown(self):
        for head in (self.animations, self.behavior, self.teleop):
            head.shutdown()
