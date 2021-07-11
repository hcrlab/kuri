import sys
import time
from ctypes import POINTER, c_void_p, c_ulong, cast, c_short
from threading import RLock

import logging
import pulse_lib

logger = logging.getLogger(__name__)
DEFAULT_SOURCE = 'alsa_output.default.monitor'


class PeakMonitor(object):
    DEFAULT_WINDOW_RATE = 40
    DEFAULT_CHANNELS = 5

    def __init__(self, source_name=None, source_chans=None, window_hz=None):
        self.SAMPLE_FORMAT = pulse_lib.PA_SAMPLE_S16LE
        self.source_name = source_name or DEFAULT_SOURCE
        self._source_chans = source_chans or self.DEFAULT_CHANNELS
        self._window_hz = window_hz or self.DEFAULT_WINDOW_RATE
        self._context_notify_cb = pulse_lib.pa_context_notify_cb_t(self.context_notify_cb)
        self._source_info_cb = pulse_lib.pa_source_info_cb_t(self.source_info_cb)
        self._stream_read_cb = pulse_lib.pa_stream_request_cb_t(self.stream_read_cb)
        self._level = 0
        self._api_lock = RLock()
        self._start_stop_lock = RLock()
        self._mainloop = None
        self._context = None
        self._stream = None
        return

    def start(self):
        with self._start_stop_lock:
            self._mainloop = pulse_lib.pa_threaded_mainloop_new()
            _mainloop_api = pulse_lib.pa_threaded_mainloop_get_api(self._mainloop)
            self._context = pulse_lib.pa_context_new(_mainloop_api, 'peak_detect')
            pulse_lib.pa_context_set_state_callback(self._context, self._context_notify_cb, None)
            pulse_lib.pa_context_connect(self._context, None, 0, None)
            pulse_lib.pa_threaded_mainloop_start(self._mainloop)
        return

    def shutdown(self):
        with self._start_stop_lock:
            if self._mainloop:
                pulse_lib.pa_threaded_mainloop_stop(self._mainloop)
                if self._stream:
                    pulse_lib.pa_stream_disconnect(self._stream)
                    pulse_lib.pa_stream_unref(self._stream)
                    self._stream = None
                if self._context:
                    pulse_lib.pa_context_disconnect(self._context)
                    pulse_lib.pa_context_unref(self._context)
                    self._context = None
                pulse_lib.pa_threaded_mainloop_free(self._mainloop)
                self._mainloop = None
                logger.info('Shutdown successfully')
        return

    def level(self):
        lev = 0.0
        with self._api_lock:
            lev = float(self._level)
        return lev

    def context_notify_cb(self, context, _):
        state = pulse_lib.pa_context_get_state(context)
        if state == pulse_lib.PA_CONTEXT_READY:
            logger.info('Pulseaudio monitor connection ready...')
            o = pulse_lib.pa_context_get_source_info_list(context, self._source_info_cb, None)
            pulse_lib.pa_operation_unref(o)
        else:
            if state == pulse_lib.PA_CONTEXT_FAILED:
                logger.warn('Pulseaudio monitor connection failed')
            else:
                if state == pulse_lib.PA_CONTEXT_TERMINATED:
                    logger.info('Pulseaudio monitor connection terminated')
        return

    def source_info_cb(self, context, source_info_p, _, __):
        if not source_info_p:
            return
        source_info = source_info_p.contents
        if source_info.name == self.source_name:
            logger.info(('Setting up peak detection on: {}\n').format(source_info.name))
            samplespec = pulse_lib.pa_sample_spec()
            samplespec.channels = self._source_chans
            samplespec.format = self.SAMPLE_FORMAT
            samplespec.rate = self._window_hz
            self._stream = pulse_lib.pa_stream_new(context, 'peak detect', samplespec, None)
            pulse_lib.pa_stream_set_read_callback(self._stream, self._stream_read_cb, source_info.index)
            pulse_lib.pa_stream_connect_record(self._stream, source_info.name, None, pulse_lib.PA_STREAM_PEAK_DETECT)
        return

    def stream_read_cb(self, stream, length, index_incr):
        data = self._get_data(stream, length)
        if length <= 0:
            return
        avg = 0
        data_end = length / 2
        for i in range(data_end):
            avg += data[i]

        avg /= float(data_end)
        level = float(avg) / 32767
        with self._api_lock:
            self._level = level
        pulse_lib.pa_stream_drop(stream)

    def _get_data(self, stream, length):
        data = c_void_p()
        pulse_lib.pa_stream_peek(stream, data, c_ulong(length))
        data = cast(data, POINTER(c_short))
        return data


def main(args):
    source = None
    if len(args) == 2:
        source = args[1]
    monitor = PeakMonitor(source)
    monitor.start()
    while True:
        try:
            time.sleep(1.0 / 40.0)
            level = monitor.level()
            if level > 0.025:
                print ' %3f\r' % level
                sys.stdout.flush()
        except KeyboardInterrupt:
            monitor.shutdown()
            sys.exit()

    return


if __name__ == '__main__':
    def my_logger(x):
        print x

    logger.info = my_logger
    main(sys.argv)
