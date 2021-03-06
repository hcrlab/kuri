import rospy
from mobile_base_driver.msg import ChestLeds
from mobile_base_driver.msg import Led
from kuri_api.utils.mux import Mux, MuxChannel

class Lights(object):
    """
    Controls state (on/off + color) of chest LEDs.
    
    LED indices are as follows (if looking at the robot straight on):
             [12]       [13]
        [11]                 [14]
                [4]   [5]
             [3]   [0]   [6]
                [2]   [1]
        [10]                 [7]
              [9]       [8]
    
    
    .. code::python
    
        import robot_api
    
        chest_leds = robot_api.Lights()
        chest_leds.put_pixels(
            [Lights.RED] * Lights.NUM_LEDS
        )
    
    """
    OFF = (0, 0, 0)
    ON = (255, 255, 255)
    HALF = (127, 127, 127)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    ORANGE = (245, 184, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 51)
    NUM_LEDS = 15
    IDX_CENTER = 0
    IDX_INNER_BOTTOM_LEFT = 1
    IDX_INNER_BOTTOM_RIGHT = 2
    IDX_INNER_RIGHT = 3
    IDX_INNER_UPPER_RIGHT = 4
    IDX_INNER_UPPER_LEFT = 5
    IDX_INNER_LEFT = 6
    IDX_OUTER_BOTTOM_MID_LEFT = 7
    IDX_OUTER_BOTTOM_LOW_LEFT = 8
    IDX_OUTER_BOTTOM_LOW_RIGHT = 9
    IDX_OUTER_BOTTOM_MID_RIGHT = 10
    IDX_OUTER_UPPER_MID_RIGHT = 11
    IDX_OUTER_UPPER_TOP_RIGHT = 12
    IDX_OUTER_UPPER_TOP_LEFT = 13
    IDX_OUTER_UPPER_MID_LEFT = 14
    LED_ALL = range(NUM_LEDS)
    LED_CENTER = range(IDX_CENTER + 1)
    LED_MID_RING = range(IDX_INNER_BOTTOM_LEFT, IDX_INNER_LEFT + 1)
    LED_OUTER_RING = range(IDX_OUTER_BOTTOM_MID_LEFT, IDX_OUTER_UPPER_MID_LEFT + 1)
    ALL_OFF = [
     OFF] * NUM_LEDS
    ALL_ON = [ON] * NUM_LEDS
    ALL_HALF = [HALF] * NUM_LEDS

    @classmethod
    def all_leds(cls, color):
        return [color] * cls.NUM_LEDS

    def __init__(self):
        self._light_pub = rospy.Publisher('mobile_base/commands/chest_leds', ChestLeds, queue_size=1, latch=True)
        self._last_pixels = self.ALL_OFF

    def shutdown(self):
        self._light_pub.unregister()

    def put_pixels(self, pixels):
        """ set the LEDs to the values in pixels. 
            :param pixels: an array of 3-ary tuples
        """
        self._last_pixels = pixels
        chest_msg = ChestLeds()
        for i in range(min(len(pixels), Lights.NUM_LEDS)):
            chest_msg.leds[i] = Led(pixels[i][0], pixels[i][1], pixels[i][2])

        self._light_pub.publish(chest_msg)

    def get_pixels(self):
        return list(self._last_pixels)

    def off(self):
        self.put_pixels(Lights.ALL_OFF)


class LightsMux(Mux):

    class Channel(Lights, MuxChannel):

        def __init__(self, mux, name, priority):
            Lights.__init__(self)
            MuxChannel.__init__(self, mux, name, priority)

        def on_release(self):
            self.off()

        put_pixels = Mux.protect(fail=False)
        off = Mux.protect(fail=False)

    priority = [
     'emotion',
     'power_status',
     'performance',
     'teleop',
     'listening',
     'romoji']

    def __init__(self, priority=None):
        super(LightsMux, self).__init__()
        self.priority = priority or self.priority
        self.emotion = self.Channel(self, 'emotion', self.priority.index('emotion'))
        self.power = self.Channel(self, 'power_status', self.priority.index('power_status'))
        self.performance = self.Channel(self, 'performance', self.priority.index('performance'))
        self.teleop = self.Channel(self, 'teleop', self.priority.index('teleop'))
        self.listening = self.Channel(self, 'listening', self.priority.index('listening'))
        self.animations = self.Channel(self, 'animations', self.priority.index('romoji'))

    def shutdown(self):
        self.emotion = None
        self.power = None
        self.performance = None
        self.teleop = None
        self.listening = None
        self.romoji = None
        del self._channels[:]
        return