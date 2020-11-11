import logging
from random import choice, random, randint, uniform

import rospy
from kuri_api.head import Head
from numpy import clip

from .dance_routines import choreographed_performances, dance_routine_pools, dance_routines, dance_routine_names

params = {}

logger = logging.getLogger(__name__)
BPM_DANCE_RANGE = (
    params.get('dance_bpm_min', 40),
    params.get('dance_bpm_max', 180))
SLOW_DANCE_THRESHOLD = params.get('dance_slow_threshold', 64)
FAST_DANCE_THRESHOLD = params.get('dance_fast_threshold', 138)
SLOW_ROUTINE_RANGE = (
    BPM_DANCE_RANGE[0], SLOW_DANCE_THRESHOLD)
FAST_ROUTINE_RANGE = (FAST_DANCE_THRESHOLD, BPM_DANCE_RANGE[1])
MAX_DANCE_ROT_VEL = params.get('dance_max_rot_vel', 3.0)
MAX_DANCE_TRANS_VEL = params.get('dance_max_trans_vel', 0.5)
DANCE_SPEEDS = [
    'slow', 'regular', 'fast']
DANCE_SPEED_SLOW = 0
DANCE_SPEED_REG = 1
DANCE_SPEED_FAST = 2
DANCE_PROFILES = [
    'natural', 'robot']
DANCE_PROFILE_NATURAL = 0
DANCE_PROFILE_ROBOT = 1
PERFORMANCE_STAGES = [
    'intro', 'warm_up', 'apex', 'cool_down']
PERFORMANCE_STAGE_INTRO = 0
PERFORMANCE_STAGE_WARM_UP = 1
PERFORMANCE_STAGE_APEX = 2
PERFORMANCE_STAGE_COOL_DOWN = 3


def get_bpm_range():
    """
    Get the danceable BPM range.
    """
    return BPM_DANCE_RANGE


def scale_duration(duration, min_duration, max_duration, beat_period):
    """
    Scale a duration within a range based on a beat period.
    """
    while duration < min_duration:
        duration += beat_period

    while duration > max_duration:
        duration -= beat_period

    return duration


class DancePerformance(object):
    """
    Keeps track of the state of a dance performance, which is a set of
    routines that have an appropriate intensity arc for dancing to a song.
    """

    def __init__(self, bpm=None, song=None):
        self._routine_map, self.profile, self._seed = _gen_performance(bpm, song)
        logger.info(('\n\x1b[1;35mGenerated Dance Performance!\n\tSeed: {}\n\tProfile: {}\x1b[1;0m').format(self._seed,
                                                                                                            self.profile))
        self.last_pose = None
        self._time_start = rospy.get_time()
        self._performance_stage = PERFORMANCE_STAGES[PERFORMANCE_STAGE_INTRO]
        self._current_routine = self._routine_map[self._performance_stage][0]
        self._routine_index = 0
        return

    @property
    def routine_name(self):
        return self._current_routine.name

    @property
    def is_emph_step(self):
        return self._current_routine.is_emph_step

    def runtime(self):
        return rospy.get_time() - self._time_start

    def step(self):
        """
        Increments the performance by one step.
        :return: The pose that will be transitioned to.
        """
        needs_init = self._current_routine.relative and not self._current_routine.relative_is_init
        if needs_init:
            if not self.last_pose:
                logger.warn('Relative routine without a last pose')
                self.last_pose = DancePose(pan=0, tilt=0)
            self._current_routine.set_relative_point(self.last_pose.pan, self.last_pose.tilt)
        next_pose = self._current_routine.step()
        final_pose = self._current_routine.update()
        if final_pose:
            self._routine_index += 1
            self._current_routine.reset()
        self.last_pose = next_pose
        return next_pose

    def update(self):
        """
        Updates the performance stage and sets the current routine.
        """
        routines_in_stage = len(self._routine_map[self._performance_stage])
        if self._routine_index == routines_in_stage:
            self._performance_stage = self._next_performance_stage()
            logger.info(('\n\x1b[1;35mNext dance stage: {} \x1b[1;0m').format(self._performance_stage))
            self._routine_index = 0
        self._current_routine = self._routine_map[self._performance_stage][self._routine_index]

    def drop_detected(self):
        """
        TODO(adam): implement drop detection once pulse monitoring is merged.
        """
        pass

    def _next_performance_stage(self):
        """
        Increments the performance stage.
        """
        next_stage = (PERFORMANCE_STAGES.index(self._performance_stage) + 1) % len(PERFORMANCE_STAGES)
        return PERFORMANCE_STAGES[max(1, next_stage)]

    def __str__(self):
        perf_str = 'Dance Performance:'
        perf_str += ('\n\tStage: {}').format(self._performance_stage)
        perf_str += ('\n\t{}').format(self._current_routine)
        return perf_str


class DanceRoutine(object):
    """
    Represents a sequence of poses in a small "routine" of a full performance.
    """

    def __init__(self, name):
        self.name = name
        routine = dance_routines()[self.name]
        routine_poses = routine['poses']
        self.num_repeats = routine.get('repeat', 1)
        self.length = len(routine_poses)
        self.randomness = routine.get('randomness', (False, False))
        self.relative = routine.get('relative', False)
        self.relative_is_init = False
        self._index = 0
        self._times_performed_routine = 0
        self._is_emph_step = False
        self._cur_pose = None
        self._poses = []
        for pose in routine_poses:
            dp = build_dance_pose(pose)
            if self.randomness[0]:
                dp.pan *= uniform(0.5, 1.0)
                dp.tilt *= uniform(0.5, 1.0)
            if self.randomness[1]:
                if dp.wheel_rotate:
                    dp.wheel_rotate *= uniform(0.7, 1.0)
                if dp.wheel_translate:
                    dp.wheel_translate *= uniform(0.7, 1.0)
            self._poses.append(dp)

        return

    def step(self):
        """
        Increments the routine by one step.
        :return: The DancePose that is transitioned to.
        """
        if self._index == 0 and self._times_performed_routine == 0:
            logger.info(('\n\x1b[1;35mNew Dance Routine: {} \x1b[1;0m').format(self.name))
        self._cur_pose = self._poses[self._index]
        self._is_emph_step = self._index % 2 == 0
        self._index = (self._index + 1) % self.length
        if self._index == 0:
            self._times_performed_routine += 1
        return self._cur_pose

    def update(self):
        """
        Updates the dance routine state.
        :return: None if this is not the last pose in the routine.
            If it is the last pose in the routine, return the pose.
        """
        if self._times_performed_routine == self.num_repeats:
            last_pose = self._cur_pose
            return last_pose
        return
        return

    def set_relative_point(self, pan, tilt):
        """
        Sets a relative head point for a dance routine.
        """
        if not self.relative:
            logger.warn('Trying to set a relative point for non-relative routine')
            return
        for pose in self._poses:
            if pose.pan:
                pose.pan = clip(pan + pose.pan, Head.PAN_RIGHT, Head.PAN_LEFT)
            else:
                pose.pan = pan
            if pose.tilt:
                pose.tilt = clip(tilt + pose.tilt, Head.TILT_UP, Head.TILT_DOWN)
            else:
                pose.tilt = tilt

        self.relative_is_init = True

    @property
    def is_emph_step(self):
        """
        Return a bool whether an emphasis step is scheduled for the next pose.
        """
        return self._is_emph_step

    def reset(self):
        """
        Resets a routine so it can be played again in a performance.
        """
        self._cur_pose = None
        self._index = 0
        self._times_performed_routine = 0
        self.relative_is_init = False
        return

    def __str__(self):
        routine_str = 'DanceRoutine:'
        routine_str += ('\n\t\tName: {}').format(self.name)
        if self.relative:
            routine_str += ' (relative),'
        if self.randomness[0]:
            routine_str += ' (random Head),'
        if self.randomness[1]:
            routine_str += ' (random Wheels),'
        routine_str += ('\n\t\tIndex: {} / {}').format(self._index, self.length - 1)
        routine_str += ('\n\t\tRepeat: {} / {}').format(self._times_performed_routine + 1, self.num_repeats)
        if self._cur_pose:
            routine_str += ('\n\t\t{}').format(self._cur_pose)
        return routine_str


class DancePose(object):
    """
    Represents a single pose in a dance routine.
    """

    def __init__(self, pan=None, tilt=None, wheel_rotate=None, wheel_translate=None, wheel_arc=None, eyes=None):
        self.pan = pan
        self.tilt = tilt
        self.wheel_rotate = wheel_rotate
        self.wheel_translate = wheel_translate
        self.wheel_arc = wheel_arc
        self.eyes = eyes

    def __str__(self):
        pose_str = 'DancePose:'
        if self.pan:
            pose_str += ('\n\t\t\tPan: {}').format(self.pan)
        if self.tilt:
            pose_str += ('\n\t\t\tTilt: {}').format(self.tilt)
        if self.eyes:
            pose_str += ('\n\t\t\tEyes: {}').format(self.eyes)
        if self.wheel_rotate:
            pose_str += ('\n\t\t\tWheel Rotate: {}').format(self.wheel_rotate)
        if self.wheel_translate:
            pose_str += ('\n\t\t\tWheel Translate: {}').format(self.wheel_translate)
        if self.wheel_arc:
            pose_str += ('\n\t\t\tWheel Arc: {}').format(self.wheel_arc)
        return pose_str


def build_dance_pose(pose_array):
    """
    Builds a DancePose object from the shorthand performance notation
    """
    NUM_POSE_ELEMENTS = 6
    WHEEL_ROTATE_INDEX = 2
    WHEEL_TRANSLATE_INDEX = 3
    WHEEL_ARC_INDEX = 4
    pose = DancePose(*pose_array[0:NUM_POSE_ELEMENTS])
    pose = _clamp_head(pose)
    if len(pose_array) > WHEEL_ROTATE_INDEX and pose_array[WHEEL_ROTATE_INDEX] == 0:
        pose.wheel_rotate = None
    if len(pose_array) > WHEEL_TRANSLATE_INDEX and pose_array[WHEEL_TRANSLATE_INDEX] == 0:
        pose.wheel_translate = None
    if len(pose_array) > WHEEL_ARC_INDEX:
        arc = pose_array[WHEEL_ARC_INDEX]
        arc_is_ok = isinstance(arc, tuple) or isinstance(arc, list)
        if not arc_is_ok or len(arc) != 2 or arc == (0, 0):
            pose.wheel_arc = None
    pose = _single_wheel_cmd(pose)
    pose = _clamp_wheels(pose)
    return pose


def _clamp_head(pose):
    """
    Ensures we don't send the robot a head position outside of its valid range.
    """
    if pose.pan:
        pose.pan = clip(pose.pan, Head.PAN_RIGHT, Head.PAN_LEFT)
    if pose.tilt:
        pose.tilt = clip(pose.tilt, Head.TILT_UP, Head.TILT_DOWN)
    if pose.eyes:
        pose.eyes = clip(pose.eyes, Head.EYES_HAPPY, Head.EYES_CLOSED)
    return pose


def _clamp_wheels(pose):
    """
    Ensures we don't send the robot a head position outside of its valid range.
    """
    ROTATE_VELOCITY_RANGE = (
        -MAX_DANCE_ROT_VEL, MAX_DANCE_ROT_VEL)
    TRANSLATE_VELOCITY_RANGE = (-MAX_DANCE_TRANS_VEL, MAX_DANCE_TRANS_VEL)
    if pose.wheel_rotate:
        pose.wheel_rotate = clip(pose.wheel_rotate, ROTATE_VELOCITY_RANGE[0], ROTATE_VELOCITY_RANGE[1])
    if pose.wheel_translate:
        pose.wheel_translate = clip(pose.wheel_translate, TRANSLATE_VELOCITY_RANGE[0], TRANSLATE_VELOCITY_RANGE[1])
    return pose


def _single_wheel_cmd(pose):
    """
    Make sure only one wheel command, priority of rotate, translate, arc
    """
    if pose.wheel_rotate:
        pose.wheel_translate = None
        pose.wheel_arc = None
    if pose.wheel_translate:
        pose.wheel_rotate = None
        pose.wheel_arc = None
    if pose.wheel_arc:
        pose.wheel_rotate = None
        pose.wheel_translate = None
    return pose


def _gen_performance(bpm=None, song=None):
    """
    Generates a routine with a given bpm or song name. If song name is
    specified, bpm is ignored.
    """
    if song:
        return _performance_for_song(song)
    if bpm:
        return _performance_for_bpm(bpm)
    return _performance_for_bpm(120)


def _performance_for_song(song):
    """
    Placeholder for product intent. Just the pancake robot defaults.
    """
    choreo = choreographed_performances()
    if song not in choreo.keys():
        logger.warn(('Song not in list of choreographed performances: {}').format(song))
        song = 'pancake_robot'
    stages = [
        _names_to_routines(choreo[song][PERFORMANCE_STAGES[PERFORMANCE_STAGE_INTRO]]),
        _names_to_routines(choreo[song][PERFORMANCE_STAGES[PERFORMANCE_STAGE_WARM_UP]]),
        _names_to_routines(choreo[song][PERFORMANCE_STAGES[PERFORMANCE_STAGE_APEX]]),
        _names_to_routines(choreo[song][PERFORMANCE_STAGES[PERFORMANCE_STAGE_COOL_DOWN]])]
    perf = _build_performance_with_stages(stages)
    return (
        perf, 'natural', song)


def _build_performance_with_stages(stages):
    """
    Builds a performance with specified stages.
    """
    perf = {}
    if len(stages) == len(PERFORMANCE_STAGES):
        perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_INTRO]] = stages[PERFORMANCE_STAGE_INTRO]
        perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_WARM_UP]] = stages[PERFORMANCE_STAGE_WARM_UP]
        perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_APEX]] = stages[PERFORMANCE_STAGE_APEX]
        perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_COOL_DOWN]] = stages[PERFORMANCE_STAGE_COOL_DOWN]
    return perf


def _performance_for_bpm(bpm):
    """
    Generates a routine for a given BPM.
    :param: bpm The beats-per-minute of the song
    :return: (performance map with routines, motion profile)
    """
    assert bpm >= BPM_DANCE_RANGE[0] and bpm <= BPM_DANCE_RANGE[1]
    logger.info(('\n\x1b[1;35mBPM Seed: {}\x1b[1;0m').format(bpm))
    pools = dance_routine_pools()
    motion_profile = DANCE_PROFILES[DANCE_PROFILE_NATURAL]
    routines = None
    num_routines_per_segment = None
    seed = 'regular'
    if _is_slow_bpm(bpm):
        seed = 'slow'
        ROBOT_PROBABILITY = 0.15
        if random() < ROBOT_PROBABILITY:
            motion_profile = DANCE_PROFILES[DANCE_PROFILE_ROBOT]
        pool = pools[DANCE_SPEEDS[DANCE_SPEED_SLOW]][motion_profile]
        num_routines_per_segment = (randint(1, 2),
                                    randint(3, 5),
                                    randint(1, 2),
                                    randint(3, 5))
    else:
        if _is_fast_bpm(bpm):
            seed = 'fast'
            ROBOT_PROBABILITY = 0.625
            if random() < ROBOT_PROBABILITY:
                motion_profile = DANCE_PROFILES[DANCE_PROFILE_ROBOT]
            pool = pools[DANCE_SPEEDS[DANCE_SPEED_FAST]][motion_profile]
            num_routines_per_segment = (1,
                                        randint(3, 5),
                                        randint(3, 5),
                                        randint(4, 5))
        else:
            ROBOT_PROBABILITY = 0.35
            if random() < ROBOT_PROBABILITY:
                motion_profile = DANCE_PROFILES[DANCE_PROFILE_ROBOT]
            pool = pools[DANCE_SPEEDS[DANCE_SPEED_REG]][motion_profile]
            num_routines_per_segment = (1,
                                        randint(3, 5),
                                        randint(2, 3),
                                        randint(3, 6))
    routines = _performance_from_pool(pool, *num_routines_per_segment)
    assert routines is not None
    return (
        routines, motion_profile, seed)


def _is_slow_bpm(bpm):
    return bpm > SLOW_ROUTINE_RANGE[0] and bpm <= SLOW_ROUTINE_RANGE[1]


def _is_fast_bpm(bpm):
    return bpm > FAST_ROUTINE_RANGE[0] and bpm <= FAST_ROUTINE_RANGE[1]


def _performance_from_pool(routine_pools, num_intros, num_warm_up, num_apex, num_cool_down):
    perf = {}
    perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_INTRO]] = _gen_unique_pool(
        routine_pools[PERFORMANCE_STAGES[PERFORMANCE_STAGE_INTRO]], num_intros)
    perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_WARM_UP]] = _gen_unique_pool(
        routine_pools[PERFORMANCE_STAGES[PERFORMANCE_STAGE_WARM_UP]], num_warm_up)
    perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_APEX]] = _gen_unique_pool(
        routine_pools[PERFORMANCE_STAGES[PERFORMANCE_STAGE_APEX]], num_apex)
    perf[PERFORMANCE_STAGES[PERFORMANCE_STAGE_COOL_DOWN]] = _gen_unique_pool(
        routine_pools[PERFORMANCE_STAGES[PERFORMANCE_STAGE_COOL_DOWN]], num_cool_down)
    return perf


def _names_to_routines(routine_names):
    routines = []
    for routine_name in routine_names:
        if routine_name not in dance_routine_names():
            logger.warn(('INVALID ROUTINE: {}').format(routine_name))
        else:
            routines.append(DanceRoutine(routine_name))

    return routines


def _gen_unique_pool(pool, num_routines):
    routine_names = []
    routine_idx = 0
    last_routine = ''
    while routine_idx < num_routines:
        rand_routine = choice(pool)
        if last_routine != rand_routine or len(pool) == 1:
            last_routine = rand_routine
            routine_names.append(rand_routine)
            routine_idx += 1

    return _names_to_routines(routine_names)
