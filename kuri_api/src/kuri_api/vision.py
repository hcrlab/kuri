import rospy
from copy import deepcopy
from collections import Counter
import threading
from vision_msgs.srv import VisionActiveModules, VisionActiveModulesRequest, VisionCmds, VisionCmdsRequest, VisionQuery, \
    VisionQueryRequest
from vision_msgs.msg import VisionCmdMsg, FrameResults, ImageClustering
from std_msgs.msg import Header
from mayfield_msgs.msg import KeyValue
from robot_api.utils.event import Event
from robot_api.utils import ros
from threading import Thread
import threading
import Queue
from collections import deque
from .vision_utils.visual_features import VisualFeatures
from .vision_utils.utils import cast_nan
import numpy as np, logging

logger = logging.getLogger(__name__)


class Vision(object):
    """
    Coordinates activation and deactivation of vision modules through a ROS
    interface to vision_bridge (C++).
    Provides cached data from the vision bridge modules

    Config: {'fps': Int,
             'priority': Int,
             'resolution': (Int, Int)}
    """
    NAMESPACE = 'vision'
    FACE_DETECTOR = 'face_detector'
    IMAGE_QUALITY = 'image_quality'
    OBJECT_DETECTOR = 'object_detector'
    AVAILABLE_MODULES = [
        FACE_DETECTOR,
        IMAGE_QUALITY,
        OBJECT_DETECTOR]
    OBJECTS_OF_INTEREST = [
        'cat', 'dog', 'person']
    RESULTS_BUFFER_SIZE = 100
    face_change = Event()
    object_change = Event()
    PUT_TIMEOUT = 5.0
    STATES_DIRTY_TIMEOUT = 2.5

    def __init__(self):
        super(Vision, self).__init__()
        self._lock = threading.Lock()
        self._init_service_proxies()
        self._start_publishers()
        self._start_subscribers()
        self._short_term_memory = {}
        self._reqd_states = []
        self._states_dirty = threading.Event()
        self._states_dirty_time = 0.0
        self._sp_queue = Queue.Queue()
        self._sp_queue_active = threading.Event()
        self._sp_thread_shutdown = threading.Event()
        self._sp_thread = Thread(target=self._sp_update_loop)
        self._sp_thread.start()
        self.vf = VisualFeatures()
        self._results_buffer = deque(maxlen=self.RESULTS_BUFFER_SIZE)
        self._results_buffer.appendleft(self.vf.calc.featurize_frame(FrameResults()))
        self.last_pre_check = {}
        self.last_post_check = {}
        self.last_cluster = ImageClustering()
        self.last_captured = ImageClustering()

    def shutdown(self):
        for module in self.active_modules():
            self.deactivate(module)

        self.finish_queue(5.0)
        self._sp_thread_shutdown.set()
        self._sp_thread.join()
        self._stop_subscribers()

    def set_feature_config(self, fsps=None, msps=None, features=None):
        """
        Set config, data structures for frame and moment features
        Resets results buffer as it must be consistent

        :param VisualFeaturesConfig parameters
        """
        self.vf = VisualFeatures(fsps, msps, features)
        self.clear_results_buffer()

    def featurized_frame(self, data, msg=FrameResults):
        return {'msg': msg, 'data': data}

    def _start_publishers(self):
        self._capture_pub = rospy.Publisher('vision/captured', ImageClustering, queue_size=1)

    def _start_subscribers(self):
        self._results_sub = rospy.Subscriber('vision/results', FrameResults, self._results_cb)

    def _stop_subscribers(self):
        if self._results_sub:
            self._results_sub.unregister()
            self._results_sub = None
        return

    def captured(self, cluster=None):
        if not cluster:
            header = Header()
            cluster = ImageClustering()
            cluster.header = header
            if 'cluster_id' in self.last_post_check and bool(self.last_post_check['cluster_id']):
                cluster.header.stamp = rospy.Time.from_sec(self.last_post_check['start_time'])
                cluster.cluster = self.last_post_check['cluster_id']
            elif self.last_cluster:
                cluster = self.last_cluster
        self._capture_pub.publish(cluster)
        self.last_captured = cluster

    def _init_service_proxies(self):
        self._proxy = {}
        _proxies = [
            (
                'cmds', VisionCmds),
            (
                'get_config', VisionQuery),
            (
                'get_params', VisionQuery),
            (
                'active_modules', VisionActiveModules)]
        for proxy, msg in _proxies:
            self._proxy[proxy] = rospy.ServiceProxy(self.NAMESPACE + '/' + proxy, msg)

    def wait_until_ready(self, timeout=0):
        return ros.wait_for_servers(svrs=self._proxy.values(), timeout=timeout)

    def _sp_update_loop(self):
        while not self._sp_thread_shutdown.is_set():
            if self._states_dirty.is_set():
                if rospy.get_time() - self._states_dirty_time > self.STATES_DIRTY_TIMEOUT:
                    self.cmds(self._distill_reqd_state())
                    self._states_dirty.clear()
            try:
                sp_action = self._sp_queue.get_nowait()
            except Queue.Empty:
                self._sp_queue_active.clear()
                rospy.sleep(0.1)
            else:
                self._sp_queue_active.set()
                try:
                    self._proxy[sp_action[0]](sp_action[1])
                except (rospy.service.ServiceException, rospy.ServiceException) as e:
                    logger.error(e.message)

    def finish_queue(self, wait=10):
        """
        Block until sp_queue is processed
        :param wait: Float seconds to wait until timeout
        """
        start = rospy.get_time()
        while rospy.get_time() - start < wait and (
                self._sp_queue_active.is_set() or self._sp_queue.qsize() or self._states_dirty.is_set()):
            try:
                rospy.sleep(min(0.1, self._sp_queue.qsize() * 0.1))
            except rospy.ROSInterruptException:
                return

    def clear_results_buffer(self):
        with self._lock:
            self._results_buffer.clear()
            self._results_buffer.appendleft(self.vf.calc.featurize_frame(FrameResults()))

    def results_buffer_size(self):
        with self._lock:
            return len(self._results_buffer)

    def last_result_time(self):
        try:
            return self.results()['msg'].header.stamp.to_sec()
        except IndexError:
            return 0.0

    def _results_cb(self, msg):
        """
        Handle vision FrameResults msg

        Gather and build features we want for each frame
        """
        if 'face_detector' in msg.modules:
            if len(msg.faces.faces) or len(self.results()['msg'].faces.faces):
                self.face_change(msg.faces)
        if 'object_detector' in msg.modules:
            if len(msg.objects.positive_detections.objects) or len(
                    self.results()['msg'].objects.positive_detections.objects):
                self.object_change(msg.objects)
        if len(msg.clustering.cluster):
            self.last_cluster = msg.clustering
        ff = self.vf.calc.featurize_frame(msg)
        with self._lock:
            self._results_buffer.appendleft(ff)

    def req_mods(self, requester, state):
        """
        Requestor structure:
                [action, module, config, params]
            ex:
                [["activate", "face_detector", {"fps": 6}, {}],
                [deactivate", "object_detector", {}, {}],
                ...]
        """
        request_count = len(self._reqd_states)
        new_states = [req for req in self._reqd_states if req['requester'] != requester
                      ]
        if len(state) == 0:
            if len(new_states) == request_count - 1:
                self._reqd_states = deepcopy(new_states)
                self._states_dirty_time = rospy.get_time()
                self._states_dirty.set()
                logger.debug(('Cleared state for {}').format(requester))
            else:
                logger.warn(('Clearing state for {} failed.').format(requester))
                return False
        else:
            new_states.append({'requester': requester, 'state': state,
                               'req_time': rospy.get_time()})
            self._reqd_states = deepcopy(new_states)
            self._states_dirty_time = 0.0
            self._states_dirty.set()
            if len(new_states) == request_count:
                logger.debug(('Updated state for {}').format(requester))
            else:
                logger.debug(('Added state for {}').format(requester))
        return True

    def _distill_reqd_state(self):
        desired_states = {module: ('deactivate', {}, {}) for module in self.AVAILABLE_MODULES}
        for req in deepcopy(self._reqd_states):
            for s in req['state']:
                desired_states[s[1]] = (
                    s[0], s[2], s[3])

        distilled_state = []
        for module, state in desired_states.iteritems():
            distilled_state.append((state[0], module, state[1], state[2]))

        return distilled_state

    def _kv(self, d={}):
        """
        Convert dictionary to list of KeyValue
        """
        return [KeyValue(k, str(v)) for k, v in d.iteritems()]

    def _cc(self, action, module='', config={}, params={}):
        """
        Defaults and conversion to KeyValue
        """
        return [
            action, module, self._kv(config), self._kv(params)]

    def cmds(self, commands):
        """
        Convert raw commands into msgs and enque
        """
        msgs = [VisionCmdMsg(*self._cc(*command)) for command in commands]
        req = VisionCmdsRequest(commands=msgs)
        self._sp_queue.put(('cmds', req), True, self.PUT_TIMEOUT)
        self._sp_queue_active.set()
        return True

    def activate(self, module, config={}, params={}):
        return self.cmds([('activate', module, config, params)])

    def deactivate(self, module):
        return self.cmds([('deactivate', module, {}, {})])

    def trigger(self, module):
        return self.cmds([('trigger', module, {}, {})])

    def active_modules(self):
        try:
            self.finish_queue()
            resp = self._proxy['active_modules'](VisionActiveModulesRequest())
            return resp.modules
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return []

    def is_module_active(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['active_modules'](VisionActiveModulesRequest())
            return module in resp.modules
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return False

    def get_fps(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['get_config'](VisionQueryRequest(module=module))
            for kv_pair in resp.params:
                if kv_pair.k == 'fps':
                    return int(kv_pair.v)

            return
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return

        return

    def set_fps(self, module, fps):
        return self.cmds([('set_config', module, {'fps': fps}, {})])

    def get_priority(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['get_config'](VisionQueryRequest(module=module))
            for kv_pair in resp.params:
                if kv_pair.k == 'priority':
                    return int(kv_pair.v)

            return
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return

        return

    def set_priority(self, module, priority):
        return self.cmds([('set_config', module, {'priority': priority}, {})])

    def get_resolution(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['get_config'](VisionQueryRequest(module=module))
            width = height = None
            for kv_pair in resp.params:
                if kv_pair.k == 'width':
                    width = int(kv_pair.v)
                elif kv_pair.k == 'height':
                    height = int(kv_pair.v)

            if width is None or height is None:
                return
            return (
                width, height)
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return

        return

    def set_resolution(self, module, res):
        return self.cmds([
            ('set_config',
             module,
             {'width': res[0], 'height': res[1]})])

    def get_params(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['get_params'](VisionQueryRequest(module=module))
            return {kv.k: kv.v for kv in resp.params}
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return

        return

    def get_config(self, module):
        try:
            self.finish_queue()
            resp = self._proxy['get_config'](VisionQueryRequest(module=module))
            return {kv.k: kv.v for kv in resp.params}
        except (rospy.service.ServiceException, rospy.ServiceException) as e:
            logger.error(e.message)
            return

        return

    def set_params(self, module, params):
        return self.cmds([('set_config', module, {}, params)])

    def score_m(self, m_data):
        """
        Score a Moment representation

        Time-weighted mean of frame scores
        TODO(Joe): Generate score based on richer set of features
        """
        return m_data[self.vf.config.get_mf_ind(['score', 'mean', 'value'])]

    def moment_pre_check(self, thresholds, good_subject=False, similar_subject=False, target_time=None):
        """
        Moment pre check ported from photo_shoot and updated

        :param thresholds dict of thresholds to compare against
        :param good_subject bool from attention service
        :param target_time float epoch time to gather results up to
        :returns dict check with calculated results
        """
        target_time = target_time or rospy.get_time()
        comp_state = self.composite_state(target_time - 5.0, target_time)
        check = {'stats': comp_state['stats']}
        default_thresholds = {'min_score': 0.0, 'min_brightness': 0.0,
                              'min_sharpness': 0.0,
                              'min_excitement': 0.0,
                              'bad_cluster': '',
                              'max_similar': 1}
        default_thresholds.update(thresholds or {})
        thresholds = default_thresholds
        score = check['stats']['score']['value']
        check['score'] = score
        brightness = check['stats']['brightness']['value']
        sharpness = check['stats']['sharpness']['value']
        excitement = check['stats']['excitement']['value']
        check['pre_check_cluster'] = comp_state['msg'].clustering
        cluster = check['pre_check_cluster'].cluster
        cluster_size = check['pre_check_cluster'].current_cluster_size
        check['score_ok'] = cast_nan(score, 0.0) >= thresholds['min_score']
        check['brightness_ok'] = cast_nan(brightness, 0.0) >= thresholds['min_brightness']
        check['sharpness_ok'] = cast_nan(sharpness, 0.0) >= thresholds['min_sharpness']
        check['excitement_ok'] = cast_nan(excitement, 0.0) >= thresholds['min_excitement']
        check['bad_cluster_ok'] = cluster != thresholds['bad_cluster']
        check['max_similar_ok'] = cluster_size <= thresholds['max_similar']
        check['success'] = check['score_ok'] and check['brightness_ok'] and check['sharpness_ok'] and check[
            'bad_cluster_ok'] and check['max_similar_ok'] and not similar_subject and (
                                       check['excitement_ok'] or good_subject)
        check['log_str'] = ('success,{}|').format(check['success']) + ('score,{},{}|').format(score,
                                                                                              check['score_ok']) + (
                               'brightness,{},{}|').format(brightness, check['brightness_ok']) + (
                               'sharpness,{},{}|').format(sharpness, check['sharpness_ok']) + ('cluster,{},{}|').format(
            cluster, check['bad_cluster_ok']) + ('similar,{},{}|').format(cluster_size, check['max_similar_ok']) + (
                               'excitement,{},{}|').format(excitement, check['excitement_ok']) + (
                               'good_subject,{}|').format(good_subject) + ('similar_subject,{}').format(similar_subject)
        self.last_pre_check = check
        return check

    def moment_post_check(self, thresholds, start_time, stop_time):
        """
        Check whole moment to see whether or not we wish to capture it
        Return full moment representation for logging & metadata

        :param thresholds dict of thresholds to check, impacting success
        :param start_time float seconds timestamp of first moment frame
        :param stop_time float seconds timestamp of last moment frame
        :returns check dictionary with calculated results
        """
        default_thresholds = {'min_score': 0.0, 'min_brightness': 0.0,
                              'min_sharpness': 0.0,
                              'min_excitement': 0.0,
                              'min_det_count': 0.0,
                              'min_det_extent': 0.0,
                              'max_similar': 1}
        default_thresholds.update(thresholds or {})
        thresholds = default_thresholds
        check = self.get_moment_features(start_time, stop_time)
        check['score_ok'] = check['score'] >= thresholds['min_score']
        check['brightness_ok'] = check['brightness'] >= thresholds['min_brightness']
        check['sharpness_ok'] = check['sharpness'] >= thresholds['min_sharpness']
        check['excitement_ok'] = check['excitement'] >= thresholds['min_excitement']
        check['det_count_ok'] = check['det_count'] >= thresholds['min_det_count']
        check['det_extent_ok'] = check['det_extent'] >= thresholds['min_det_extent']
        check['max_similar_ok'] = check['cluster_size'] <= thresholds['max_similar'] and bool(check['cluster_id'])
        check['success'] = check['score_ok'] and check['brightness_ok'] and check['sharpness_ok'] and check[
            'excitement_ok'] and check['det_count_ok'] and check['det_extent_ok'] and check['max_similar_ok']
        check['log_str'] = ('success,{}|').format(check['success']) + ('score,{},{}|').format(check['score'],
                                                                                              check['score_ok']) + (
                               'brightness,{},{}|').format(check['brightness'], check['brightness_ok']) + (
                               'sharpness,{},{}|').format(check['sharpness'], check['sharpness_ok']) + (
                               'excitement,{},{}|').format(check['excitement'], check['excitement_ok']) + (
                               'det_count,{},{}|').format(check['det_count'], check['det_count_ok']) + (
                               'det_extent,{},{}|').format(check['det_extent'], check['det_extent_ok']) + (
                               'similar,{},{},{}').format(check['cluster_id'], check['cluster_size'],
                                                          check['max_similar_ok'])
        self._last_checked_moment = check
        self.last_post_check = check
        return check

    def results_window(self, start_time, stop_time, padding=(0, 0)):
        """
        Filter of self._results_buffer by timestamp

        As self._results_buffer is ordered [newest, ... , oldest] we
        iterate through it, first looking for the stop_time, then appending
        until we pass the start_time.

        :param start_time float seconds timestamp of first result
        :param stop_time float seconds timestamp of last result
        :param padding (int, int) # results to pad output beyond time window
        :returns results [oldest, ..., newest] within window + padding
        """
        results_in_window = []
        with self._lock:
            for i, r in enumerate(self._results_buffer):
                r_time = r['msg'].header.stamp.to_sec()
                if r_time <= stop_time:
                    if r_time >= start_time:
                        if len(results_in_window) == 0:
                            for p in range(padding[1])[::-1]:
                                padding_index = max(i - p, 0)
                                results_in_window.append(self._results_buffer[padding_index])

                        results_in_window.append(self._results_buffer[i])
                    else:
                        for p in range(padding[0])[::-1]:
                            padding_index = min(len(self._results_buffer), i + p)
                            results_in_window.append(self._results_buffer[padding_index])

                        break

        return results_in_window[::-1]

    def composite_ff(self, results):
        """
        Composite multiple featurized frames

        This handes the scenario where we have different detectors on different
        frames, such as quality running at 6fps and object detections running
        at 2fps and we want our best representation (most recent from each).

        :param results: List of results to composite. (oldest to newest)
        """
        m = FrameResults()
        for r in results:
            if 'face_detector' in r['msg'].modules:
                if 'face_detector' not in m.modules:
                    m.modules.append('face_detector')
                m.faces = r['msg'].faces
            if 'object_detector' in r['msg'].modules:
                if 'object_detector' not in m.modules:
                    m.modules.append('object_detector')
                m.objects = r['msg'].objects
            if len(r['msg'].features.features):
                m.features = r['msg'].features
            if len(r['msg'].clustering.cluster):
                m.clustering = r['msg'].clustering
            if 'image_quality' in r['msg'].modules:
                if 'image_quality' not in m.modules:
                    m.modules.append('image_quality')
                m.quality = r['msg'].quality

        if len(results):
            m.header = results[-1]['msg'].header
        return self.vf.calc.featurize_frame(m)

    def clusters_over_proportion(self, clusters, proportion=0.25):
        """
        Returns clusters from the moment over the proportion parameter
        in descending by frames count order, minimum of 1.
        """
        clusters_counter = Counter(clusters)
        total_size = sum(clusters_counter.values())
        output = []
        for index, ((cluster, cluster_size), count) in enumerate(clusters_counter.most_common()):
            if index == 0 or 1.0 * count / total_size > proportion:
                output.append((cluster, cluster_size))

        return output

    def get_moment_features(self, m_start, m_stop, cast_nan_to_none=True):
        """
        Gather moment level features.

        :param m_start: (float) epoch timestamp of first moment frame
        :param m_stop: (float) epoch timestamp of last moment frame
        :param cast_nan_to_none: (bool) ROBOT-1130 Cast np.nan to None or 0.0
        :returns (dict) Moment level features
        """
        output = {'start_time': m_start,
                  'stop_time': m_stop}
        frames = self.results_window(m_start, m_stop, padding=(1, 1))
        moment_data = self.vf.calc.gen_moment_data(frames, m_start, m_stop)
        output['brightness'] = moment_data[self.vf.config.get_mf_ind([
            'brightness', 'mean', 'value'])]
        output['sharpness'] = moment_data[self.vf.config.get_mf_ind([
            'sharpness', 'mean', 'value'])]
        output['brightness_delta'] = moment_data[self.vf.config.get_mf_ind([
            'brightness', 'var', 'value'])]
        output['sharpness_delta'] = moment_data[self.vf.config.get_mf_ind([
            'sharpness', 'var', 'value'])]
        output['excitement'] = moment_data[self.vf.config.get_mf_ind([
            'excitement', 'mean', 'value'])]
        output['clusters'] = [(ff['msg'].clustering.cluster, ff['msg'].clustering.current_cluster_size) for ff in frames
                              if ff['msg'].clustering.cluster
                              ]
        output['significant_clusters'] = self.clusters_over_proportion(output['clusters'])
        if not output['significant_clusters']:
            output['cluster_id'], output['cluster_size'] = ('', 0)
        else:
            output['cluster_id'], output['cluster_size'] = output['significant_clusters'][0]
        output['faces'] = []
        faces_count = moment_data[self.vf.config.get_mf_ind([
            'det', 'face', 'extent', 'mean', 'count'])]
        if faces_count > 0.5:
            faces = []
            for f in frames:
                faces.extend(f['msg'].faces.faces)

            output['faces'] = faces
        output['detections'] = []
        ooi_count = np.nansum(
            [moment_data[self.vf.config.get_mf_ind(['det', ooi, 'extent', 'mean', 'count'])] for ooi in [
                'cat', 'dog']
             ])
        if ooi_count > 0.5:
            obj = []
            for f in frames:
                obj.extend(f['msg'].objects.positive_detections.objects)

            output['detections'] = obj
        det_classes = [
            'face', 'cat', 'dog']
        output['det_count'] = np.nansum(
            [moment_data[self.vf.config.get_mf_ind(['det', det, 'extent', 'mean', 'count'])] for det in det_classes
             ])
        output['det_extent'] = np.nansum(
            [moment_data[self.vf.config.get_mf_ind(['det', det, 'extent', 'mean', 'mean'])] for det in det_classes
             ])
        output['score'] = self.score_m(moment_data)
        if cast_nan_to_none:
            nan_to_none_keys = [
                'score', 'det_count', 'det_extent']
            for k in nan_to_none_keys:
                output[k] = cast_nan(output[k], None)

            metrics = ['score',
                       'excitement',
                       'brightness',
                       'sharpness',
                       'brightness_delta',
                       'sharpness_delta',
                       'det_count',
                       'det_extent']
            for k in metrics:
                output[k] = cast_nan(output[k], 0.0)

        output['stats'] = self.vf.render_mf(moment_data, cast_nan_to_none)
        return output

    def results(self):
        with self._lock:
            return self.vf.render_f_result(self._results_buffer[0])

    def composite_state(self, start_time, stop_time, padding=(0, 0)):
        """
        Data accessor backed by self._latest_results buffer

        :param start_time: start_time for results_window
        :param stop_time: stop_time param from results_window
        """
        results_of_interest = self.results_window(start_time, stop_time, padding)
        comp = self.composite_ff(results_of_interest)
        return self.vf.render_f_result(comp)

    def last_checked_moment(self):
        return deepcopy(self._last_checked_moment)

    def faces(self):
        """ :return: the faces that the robot sees right now """
        return self.results()['msg'].faces.faces

    def objects(self):
        """ :return: the objects that the robot thinks it sees
            This is the list of positive detections
        """
        return self.results()['msg'].objects.positive_detections.objects

    def brightness(self):
        """ :return: (float) how bright the image is, from 0.0 to 1.0 """
        return self.results()['msg'].quality.brightness

    def sharpness(self):
        """ :return: (float) how sharp the image is, from 0.0 to 1.0 """
        return self.results()['msg'].quality.sharpness

    def brightness_delta(self):
        """ :return: (float) change in brightness from -1.0 to 1.0 """
        return self.results()['msg'].quality.brightness_delta

    def sharpness_delta(self):
        """ :return: (float) change in sharpness, from -1.0 to 1.0 """
        return self.results()['msg'].quality.sharpness_delta

    def cluster_id(self):
        """ :return: (str) the current cluster ID """
        return self.results()['msg'].clustering.cluster

    def cluster_size(self):
        """ :return: (int) the number of similar captured moments"""
        return self.results()['msg'].clustering.current_cluster_size

    def features(self):
        """ :return: ([int]) raw features from the object classifier """
        return self.results()['msg'].features

    def excitement(self):
        """ :return: (float) excitement score from object classifier """
        ex = self.vf.calc.get_f_val(['excitement', 'value'], self.results()['data'])
        if np.isnan(ex):
            ex = 0
        return ex

    def score(self):
        """ :return: (float) placeholder synthetic moment ranking score """
        sc = self.vf.calc.get_f_val(['score', 'value'], self.results()['data'])
        if np.isnan(sc):
            sc = 0
        return sc