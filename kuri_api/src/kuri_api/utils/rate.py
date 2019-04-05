import time

class Rate(object):
    """
    Convenience class for sleeping in a loop at a specified rate.
    
    Copied over from rospy, modified to not use rostime as that requires master
    to be running, and init_node to have been called. -Hai Nguyen
    """

    def __init__(self, hz):
        """
        Constructor.
        @param hz: hz rate to determine sleeping
        @type  hz: int
        """
        self.last_time = time.time()
        self.sleep_dur = 1.0 / hz

    def _remaining(self, curr_time):
        """
        Calculate the time remaining for rate to sleep.
        @param curr_time: current time
        @type  curr_time: L{Time}
        @return: time remaining
        @rtype: L{Time}
        """
        if self.last_time > curr_time:
            self.last_time = curr_time
        elapsed = curr_time - self.last_time
        r = self.sleep_dur - elapsed
        if r < 0:
            return 0
        return r

    def remaining(self):
        """
        Return the time remaining for rate to sleep.
        @return: time remaining
        @rtype: L{Time}
        """
        curr_time = time.time()
        return self._remaining(curr_time)

    def sleep(self, f=None):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().
        
        @param f: float -> val, function that does the actual
            sleeping. If None, use time.sleep.
        """
        curr_time = time.time()
        r = None
        if f:
            r = f(self._remaining(curr_time))
        else:
            time.sleep(self._remaining(curr_time))
        self.last_time = self.last_time + self.sleep_dur
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time
        return r
