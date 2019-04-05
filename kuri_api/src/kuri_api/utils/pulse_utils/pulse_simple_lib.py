from ctypes import byref, c_int, c_uint32, c_uint8, Structure
import ctypes, logging
logger = logging.getLogger(__name__)
pa_simple = ctypes.cdll.LoadLibrary('libpulse-simple.so.0')

class struct_pa_sample_spec(Structure):
    __slots__ = [
     'format',
     'rate',
     'channels']


struct_pa_sample_spec._fields_ = [
 (
  'format', c_int),
 (
  'rate', c_uint32),
 (
  'channels', c_uint8)]
pa_sample_spec = struct_pa_sample_spec
PA_SAMPLE_S16LE = 3
DEFAULT_CHANNELS = 2
DEFAULT_SAMPLE_RATE = 48000
PA_STREAM_PLAYBACK = 1

def get_sample_spec(rate=None, channels=None, sample_format=None):
    spec_format = struct_pa_sample_spec()
    spec_format.rate = rate or DEFAULT_SAMPLE_RATE
    spec_format.channels = channels or DEFAULT_CHANNELS
    spec_format.format = sample_format or c_int(PA_SAMPLE_S16LE)
    return spec_format


def make_playback_stream(name, stream_format):
    error = c_int(0)
    stream = pa_simple.pa_simple_new(None, name, PA_STREAM_PLAYBACK, None, 'playback', byref(stream_format), None, None, byref(error))
    if not stream:
        logger.error(('Could not create pulse audio stream: {0}!').format(pa_simple.strerror(byref(error))))
    return stream


def check_latency(stream):
    error = c_int(0)
    latency = pa_simple.pa_simple_get_latency(stream, error)
    return latency >= 0


def write(stream, buf):
    error = c_int(0)
    status = pa_simple.pa_simple_write(stream, buf, len(buf), error)
    if status < 0:
        logger.error(('Could not write to the stream: {0}!').format(pa_simple.strerror(byref(error))))
    if status < 0:
        return False
    return True


def drain(stream):
    error = c_int(0)
    status = pa_simple.pa_simple_drain(stream, error)
    if status < 0:
        logger.error(('Could not drain the stream: {0}!').format(pa_simple.strerror(byref(error))))
    if status < 0:
        return False
    return True


def teardown_playback_stream(stream):
    pa_simple.pa_simple_free(stream)
