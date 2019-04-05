"""
Helpers for dealing with ROS.
"""
import logging, genpy, rospy, timer, traceback
from threading import Lock
logger = logging.getLogger(__name__)
DEFAULT_SERVICE_WAIT_RATE = 0.1

def async_wait_for_servers(svrs, done_cb=None):
    """
    Wait for servers asynchronously.
    Timeout has to be manually added
    :param svrs: list of services
    :param done_cb: callback when done.
    :return: A timer object that allows you to
    """

    def _runner():
        if wait_for_servers(svrs=svrs, timeout=DEFAULT_SERVICE_WAIT_RATE):
            done_cb()
            return True
        return False

    t = timer.Timer(rate=DEFAULT_SERVICE_WAIT_RATE, end_condition_callback=_runner)
    return t


def wait_for_servers(svrs, timeout=None):
    if not isinstance(svrs, list):
        svrs = [
         svrs]
    for svr in svrs:
        try:
            svr.wait_for_service(timeout=timeout)
        except rospy.ROSException as ex:
            logger.info(ex)
            return False

    return True


def wait_for_topics(topics, timeout=None, poll=0.1):
    if not isinstance(topics, list):
        topics = [
         topics]
    expires_at = rospy.get_time() + timeout if timeout else None
    for topics in topics:
        while topics.get_num_connections() == 0 and (expires_at is None or rospy.get_time() < expires_at):
            rospy.sleep(poll)

        if topics.get_num_connections() == 0:
            return False

    return True


def ros_obj_to_dict(msg):
    d = {}
    for s in msg.__slots__:
        val = getattr(msg, s)
        if issubclass(val.__class__, genpy.Time):
            d[s] = val.to_time()
        elif issubclass(val.__class__, genpy.message.Message):
            d[s] = ros_obj_to_dict(val)
        else:
            d[s] = val

    return d


def dict_to_ros_obj(msg_class, d):
    msg = msg_class()
    for s in msg.__slots__:
        val = getattr(msg, s)
        if issubclass(val.__class__, genpy.Time):
            setattr(msg, s, genpy.Time(d[s]))
        elif issubclass(val.__class__, genpy.message.Message):
            setattr(msg, s, dict_to_ros_obj(val.__class__, d[s]))
        else:
            setattr(msg, s, d[s])

    return msg


class SafeServiceProxy(object):
    RETRIES = 3

    def __init__(self, *args, **kwargs):
        self._args = args
        self._kwargs = kwargs
        self._create_service()
        self._mute_failures = False
        self._lock = Lock()

    def set_mute_failures(self, v):
        """
        Whether to mute logger.warn for ServiceException's.
        """
        self._mute_failures = v

    def _create_service(self):
        self._service_proxy = rospy.ServiceProxy(*self._args, **self._kwargs)

    def wait_for_service(self, *args, **kwargs):
        self._service_proxy.wait_for_service(*args, **kwargs)

    def __call__(self, *args, **kwargs):
        with self._lock:
            for i in range(self.RETRIES):
                try:
                    return self._service_proxy(*args, **kwargs)
                except rospy.service.ServiceException as rospy.ServiceException:
                    name = ''
                    if hasattr(self._service_proxy, 'resolved_name'):
                        name = self._service_proxy.resolved_name
                    if not self._mute_failures:
                        logger.warn(('Failed to call {}').format(name))
                    return
                except Exception:
                    logger.error(traceback.format_exc())
                    self._create_service()

        return