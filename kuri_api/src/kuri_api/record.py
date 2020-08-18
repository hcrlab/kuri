from __future__ import print_function
import collections, functools, logging, madmux, os
from Queue import Queue
import subprocess, tempfile, threading, time

logger = logging.getLogger(__name__)


class _MadmuxSource(object):
    CHANNEL = 1

    def __init__(self, channel=None):
        self.cbs = []
        if channel is None:
            channel = self.CHANNEL
        self.str = madmux.Stream(('/var/run/madmux/ch{}.sock').format(channel))
        self.str.register_cb(self._mdx_cb)
        return

    def register_cb(self, cb):
        self.cbs.append(cb)

    def unregister_cb(self, cb):
        if cb in self.cbs:
            self.cbs.remove(cb)

    def ask_keyframe(self):
        self.str.force_iframe()

    def snap(self):
        self.str.snap()

    @staticmethod
    def is_keyframe(data):
        hdr = '\x00\x00\x00\x01\t\x10'
        return data[:len(hdr)] == hdr

    def _mdx_cb(self, data):
        for cb in self.cbs:
            cb(data)

    def shutdown(self):
        self.cbs = []
        self.str.close()


class _SnapCamera(object):

    def __init__(self, source):
        self.src = source
        self.cbs = []
        self.waiting_cbs = []
        self.lock = threading.Lock()
        self.data = None
        return

    def _src_cb(self, data):
        with self.lock:
            cbs = self.cbs
            self.cbs = []
            self.waiting_cbs = cbs[:]
            self.data = data
            self.src.unregister_cb(self._src_cb)

    def snap(self, cb):
        with self.lock:
            self.data = None
            self.cbs = []
            self.waiting_cbs = []
            self.src.register_cb(self._src_cb)
            self.cbs.append(cb)
        self.src.snap()
        return

    def upload(self):
        with self.lock:
            if not self.data:
                logger.warn('Attempting to upload without data')
                return False
            cbs = self.waiting_cbs
            self.waiting_cbs = []
            for cb in cbs:
                cb(self.data)

            self.data = None
        return True

    def scrap(self):
        with self.lock:
            self.cbs = []
            self.waiting_cbs = []
            self.data = None
        return

    def shutdown(self):
        self.cbs = []
        self.waiting_cbs = []
        self.data = None
        self.src.unregister_cb(self._src_cb)
        return


class _BufferCamera(object):
    """
    A camera that accumulates a video buffer on demand
    :param source: the video source
    :param buf_len: the length of the buffer [s]
    """
    KEYFRAME_PERIOD = 1.0

    def __init__(self, source, buf_len=None):
        self.source = source
        self.images = collections.deque()
        self.timers = []
        self.last_keyframe = 0

    def _clear_timers(self):
        self.timers = [x for x in self.timers if x.is_alive()]

    def flush(self, trigger=True):
        """ flush the buffer, cancel (and optionally trigger) all timers """
        self.source.unregister_cb(self._video_cb)
        for t in self.timers:
            t.cancel()
            if trigger:
                t.function()

        self.timers = []
        self.images.clear()

    def get_buffer(self):
        self.source.unregister_cb(self._video_cb)
        return list(self.images)

    def capture(self, cb, duration):
        t = threading.Timer(duration, lambda: cb(self.get_buffer()))
        self._clear_timers()
        self.timers.append(t)
        t.setDaemon(True)
        t.start()
        self.last_keyframe = 0
        self.images.clear()
        self.source.register_cb(self._video_cb)

    def _video_cb(self, data):
        keyframe = self.source.is_keyframe(data)
        if keyframe:
            self.last_keyframe = time.time()
        if time.time() - self.last_keyframe > self.KEYFRAME_PERIOD:
            self.source.ask_keyframe()
        if self.last_keyframe == 0:
            return
        self.images.append((time.time(), data, keyframe))

    def shutdown(self):
        self.flush(trigger=False)


class _WorkQueue(threading.Thread):
    """
    An asynchronous task queue.
    Each item will be processed one by one by the given function.
    
    :param fun: the given function
    """
    daemon = True
    DONE = '__done__'

    def __init__(self, fun):
        super(_WorkQueue, self).__init__(name='_WorkQueue')
        self.fun = fun
        self.q = Queue()
        self.start()

    def put(self, item):
        self.q.put(item)

    def run(self):
        while True:
            item = self.q.get()
            if item is self.DONE:
                break
            self.fun(item)
            self.q.task_done()

    def shutdown(self):
        self.q.put(self.DONE)
        self.join(timeout=1.0)
        assert not self.is_alive()


def _clean_value(d, key, instance, default, comp=lambda x: True):
    if key in d and isinstance(d[key], instance) and comp(d[key]):
        return d[key]
    return default


def _metadata(vision, tf_srv=None, map_srv=None, image_wp_srv=None):
    pose_se2 = [0, 0, 0]
    if tf_srv:
        pose_se2 = tf_srv.robot_pose_se2() or pose_se2
    if map_srv:
        waypoint['map_uuid'] = map_srv.current_map_uuid()
    m_check = vision.last_post_check
    return {'version': '1.0',
            'timestamp': _clean_value(m_check, 'start_time', float, 0.0),
            'end_timestamp': _clean_value(m_check, 'stop_time', float, 0.0),
            'cluster_id': _clean_value(m_check, 'cluster_id', basestring, None, len),
            'detections': _clean_value(m_check, 'detections', list, []),
            'faces': _clean_value(m_check, 'faces', list, []),
            'joint_states': None,
            'brightness': _clean_value(m_check, 'brightness', float, 0.0),
            'sharpness': _clean_value(m_check, 'sharpness', float, 0.0),
            'brightness_delta': _clean_value(m_check, 'brightness_delta', float, 0.0),
            'sharpness_delta': _clean_value(m_check, 'sharpness_delta', float, 0.0),
            'excitement': _clean_value(m_check, 'excitement', float, 0.0),
            'score': _clean_value(m_check, 'score', float, 0.0),
            'stats': _clean_value(m_check, 'stats', dict, {})}


def call(cmd):
    """ like check_call, but logs the stderr and returns it in the
    exception e.output """
    with open(os.devnull, 'w') as (devnull):
        p = subprocess.Popen(cmd, stdout=devnull, stderr=subprocess.PIPE)
        err = p.communicate()
        ret = p.returncode
        if ret != 0:
            raise subprocess.CalledProcessError(ret, cmd, output=err)


class LivePhoto(object):
    """
    like a photo, but like, many of them, you know
    
    'Live' photos(tm) are actually short videos (default of 5 s).
    A single thumbnail is snapped at the beginning of the video
    Metadata is also uploaded, and contains vision and robot state data.
    
    :param vision: an instance of the vision service
    :param mqtt: an instance of the MQTT client
    :param uploader: an instance of the uploader service
    :param tf_srv: an instance of the Transforms service
    :param map_srv: an instance of the Mapping service
    :param image_wp_srv: an instance of the PlaceService service
    :param memory_srv: an instance of the LongTermMemory service
    :param fake_source: a Bool indicating to use a fake MadMux source (tests)
    
    .. code:: python
    
        import time
        from mayvision.services import Vision
        from gizmo.services import Mqtt, uploader
    
        # Trigger a 3 second capture
        ph = LivePhoto(...)
        def decision():
            if PREDICATE:
                ph.save()
            else:
                ph.scrap()
        ph.capture(3, cb=decision)
    
        # Once the callback is executed, the data and metadata are stored in
        # memory. The LivePhoto can then be uploaded to S3 with a call to
        # upload(). Any subsequent calls to capture() will clear out
        # the previous data. If you decide not to upload the LivePhoto, call
        # scrap() to save some memory. If upload is called, the video buffer
        # will be dumped to the disk, converted to an h264 MP4 file, and the
        # thumbnail and video will be uploaded to S3 and associate the
        # metadata to MQTT
    
        # Trigger a 5 second capture with a predicate function
    
        # To stop a LivePhoto while it is being captured, call cut(). If you
        # pass capture=True, the user callbacks will run and the LivePhoto
        # will be truncated from the original duration.
    """
    DEFAULT_DURATION = 5.0

    def __init__(self, vision, tf_srv, map_srv, image_wp_srv):
        self.vision = vision
        self._tf_srv = tf_srv
        self._map_srv = map_srv
        self._image_wp_srv = image_wp_srv
        self.lock = threading.Lock()
        self.data = None
        self.metadata = None
        self.moment_post_check = False
        self.encoder = _WorkQueue(self._encode)
        self.video_src = _MadmuxSource(channel=1)
        self.snap_src = _MadmuxSource(channel=3)
        self.bufcam = _BufferCamera(self.video_src)
        self.snapcam = _SnapCamera(self.snap_src)
        self.post_check_thresholds = {}
        return

    def _encode(self, data):
        """ this method is called with a list of
            (time, frame, keyframe[bool]) tuples
            We need to mux these raw h264 frames into a container.
            That's what MP4Box is for.
            Then we send that to the uploader.
        """
        buf, video_metadata = data
        h264 = tempfile.mktemp(suffix='.h264')
        mp4 = tempfile.mktemp(suffix='.mp4')
        with open(h264, 'wb') as (f):
            for _, frame, _ in buf:
                f.write(frame)

        try:
            length = buf[-1][0] - buf[0][0]
            fps = len(buf) / length
        except (IndexError, ZeroDivisionError) as e:
            logger.error(('Got weird buffer in _encode: {}').format(e))
            return

        try:
            call([
                'MP4Box',
                '-add', ('{}:fps={}').format(h264, fps),
                '-new', mp4])
        except subprocess.CalledProcessError as e:
            logger.error(('{}\n{}').format(e, e.output))
            if os.path.exists(mp4):
                os.unlink(mp4)
            return
        finally:
            os.unlink(h264)

        video_uuid = video_metadata['media_uuid']
        print(mp4)

    def cut(self, capture=True):
        """ clear the rolling buffer, trigger any video
            capture callbacks early if needed
        
            :param capture: whether to trigger the callbacks or not
        """
        with self.lock:
            if capture:
                self.snapcam.upload()
            else:
                self.snapcam.scrap()
        self.bufcam.flush(trigger=capture)

    def capture(self, duration=DEFAULT_DURATION, cb=None, source='', trigger=None):
        """ launch a new video capture
            this will use the default duration
            when done, the video will be encoded
            and everything will magically go to the cloud
        """
        with self.lock:
            self.moment_post_check = False
            self.data = None
            self.metadata = None
        thumb_uuid = "xt"
        video_uuid = "xv"

        def capture_cb(buf):
            with self.lock:
                moment_check = {}
                try:
                    moment_check = self.vision.moment_post_check(self.post_check_thresholds, buf[0][0], buf[-1][0])
                except IndexError as e:
                    logger.error(('Weird buffer in capture_cb: {}').format(e))
                    if cb:
                        cb()
                    return

                metadata = _metadata(self.vision, tf_srv=self._tf_srv, map_srv=self._map_srv,
                                     image_wp_srv=self._image_wp_srv)
                metadata['media_uuid'] = video_uuid
                metadata['media_type'] = 'video'
                metadata['thumbnail'] = thumb_uuid
                metadata['source'] = source
                metadata['trigger'] = trigger
                self.moment_post_check = 'success' in moment_check and moment_check['success']
                self.data = buf
                self.metadata = metadata
            if cb:
                cb()

        self.snapcam.snap(lambda data: print("Got snap data"))
        self.bufcam.capture(capture_cb, duration)
        return

    def save(self):
        """
        Uploads the last captured LivePhoto
        :return: metadata for the moment if upload is successful, else None
        """
        with self.lock:
            if not self.data or not self.metadata:
                logger.warn('Attempted to upload without data or metadata')
                return
            data = self.data
            metadata = self.metadata
            self.data = None
            self.metadata = None
        # Snap data is available here too
        self.encoder.put((data, metadata))
        return metadata

    def scrap(self):
        """
        Scraps a buffered LivePhoto.
        Call this to free up some memory if you call capture() and decide not
        to surface the LivePhoto.
        """
        self.snapcam.scrap()
        with self.lock:
            self.data = None
            self.metadata = None
        return

    def shutdown(self):
        self.encoder.shutdown()
        self.bufcam.shutdown()
        self.snapcam.shutdown()
        self.video_src.shutdown()
        self.snap_src.shutdown()
