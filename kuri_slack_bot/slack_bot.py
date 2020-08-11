import os

class SlackBot:
    """Constructs help-request message"""

    WELCOME_BLOCK = {
        "type": "section",
        "text": {
            "type": "mrkdwn",
            "text": (
                "Hi, I'm kuribot! I let your Kuri communicate with you through Slack :robot_face:"
            ),
        },
    }

    TELEOP_BLOCK = {
        "type": "section",
        "text": {
            "type": "mrkdwn",
            "text": (
                "Kuri needs your help! Click <http://" + os.environ['TELEOP_URL'] + "|here> to view a teleop interface..."
            ),
        },
    }

    def __init__(self, channel=None):
        if channel:
            self.channel = channel
        else:
            self.channel = "test"
        self.username = "kuribot"
        self.icon_emoji = ":robot_face:"
        self.timestamp = ""
    
    def get_welcome_payload(self):
        return {
            "ts": self.timestamp,
            "channel": self.channel,
            "username": self.username,
            "icon_emoji": self.icon_emoji,
            "blocks": [
                self.WELCOME_BLOCK
            ]
        }

    def get_teleop_payload(self):
        return {
            "ts": self.timestamp,
            "channel": self.channel,
            "username": self.username,
            "icon_emoji": self.icon_emoji,
            "blocks": [
                self.TELEOP_BLOCK
            ]
        }
