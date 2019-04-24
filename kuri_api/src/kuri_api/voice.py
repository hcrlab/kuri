import pyttsx

class Voice(object):
    def __init__(self):
        self.engine = pyttsx.init()
        # To see what other voices are available
        """
        voices = engine.getProperty('voices')
        for voice in voices:
            if "english" in voice.id:
                print(voice)
        """
        # "english" may be a little more understandable
        self.engine.setProperty('voice', "english-us+f2")
        self.engine.setProperty('volume', 1.0)
        self.engine.setProperty('rate', 160)

    def say(self, whattosay):
        self.engine.say(whattosay)
        self.engine.runAndWait()
