import logging, threading, json, rospy
from std_msgs.msg import Empty, String
from audio_msgs.msg import Awake, Exchange
from audio_msgs.srv import Stat, WakeUp, Snooze
from kuri_api.utils import Events
from kuri_api.utils.ros import wait_for_servers, wait_for_topics

logger = logging.getLogger(__name__)


class VoiceCommand(object):
    """
    Message type representing voice commands.
    """

    def __init__(self, name=None, params=None):
        self.name = name
        self.params = params

    def __repr__(self):
        return ('Command: {}, Params: {}').format(self.name, self.params)


class Listener(Events):
    """
    Used to listen for:

    - a wake word (e.g. the name of the robot) while asleep
    - commands to execute while awake

    .. code:: python

        from kuri_api import Voice

        def on_voice_wake():
            print "i'm up, i'm up!"

        def on_voice_command(msg):
            print 'sorry, too sleepy ...'
            voice.snooze()

        listener = Listener()
        listener.wake_event.connect(on_voice_command)
        listener.voice_command_event.connect(on_voice_command)

    """
    NAMESPACE = 'audio'
    PROCESS_DELAY = 4
    wake_event = Events.source()
    voice_command_event = Events.source()
    voice_trigger_event = Events.source()

    def __init__(self, namespace=None):
        super(Listener, self).__init__()
        namespace = namespace or self.NAMESPACE
        self._awake_sub = rospy.Subscriber(namespace + '/voice_delegate/awake', Awake, self._on_awake)
        self._exchange_sub = rospy.Subscriber(namespace + '/voice_delegate/exchange', Exchange, self._on_exchange)
        self._stat = rospy.ServiceProxy(namespace + '/voice_delegate/stat', Stat)
        self._wake_up = rospy.ServiceProxy(namespace + '/voice_delegate/wake_up', WakeUp)
        self._snooze = rospy.ServiceProxy(namespace + '/voice_delegate/snooze', Snooze)
        self._lock = threading.RLock()
        self._program_initiated = False
        self._awake_timeout = rospy.get_param('/audio_voice_delegate/awake_timeout', 10) + self.PROCESS_DELAY

    def shutdown(self):
        self._exchange_sub.unregister()
        self._awake_sub.unregister()
        self.snooze()

    def wait_until_ready(self, timeout=0):
        return wait_for_servers([
            self._stat, self._wake_up, self._snooze], timeout=timeout) and wait_for_topics([
            self._awake_sub, self._exchange_sub], timeout=timeout, poll=0.1)

    @property
    def awake_timeout(self):
        return self._awake_timeout

    @property
    def is_awake(self):
        return self._stat().state == 'awake'

    def wake_up(self):
        self._program_initiated = True
        try:
            self._wake_up()
        except rospy.ServiceException as e:
            logger.error(e.message)

    def snooze(self):
        self._program_initiated = False
        try:
            self._snooze()
        except rospy.ServiceException as e:
            logger.error(e.message)

    def _on_awake(self, wake):
        if not self._program_initiated:
            self.wake_event(wake)

    def _on_exchange(self, exchange):
        """
        Stop transcription, generate a voice_command event
        """
        self.snooze()
        if hasattr(exchange, 'error') and exchange.error:
            self.voice_trigger_event({'trigger': 'voice error',
                                      'params': {'input': exchange.error}})
            return
        if len(exchange.commands) > 0 and len(exchange.commands[0].params) == 0 and exchange.transcription != "":
            name = 'custom'
            params = {"transcript": exchange.transcription}
        elif len(exchange.commands) > 0 and len(exchange.commands[0].params) > 0 and len(exchange.commands[0].params[0].k) > 0 and len(exchange.commands[0].params[0].v) > 0:
            command = exchange.commands[0]
            name = command.name.replace('Command', '').replace('Kuri', '').lower()
            params = {param.k: param.v for param in command.params}
            params["transcript"] = exchange.transcription
        else:
            #print("empty exchange")
            return
        command = VoiceCommand(name=name, params=params)
        self.voice_command_event(command)