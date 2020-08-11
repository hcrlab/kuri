import os
import logging
LOGGER = logging.getLogger("default logger")
LOGGER.setLevel(logging.DEBUG)
LOGGER.addHandler(logging.StreamHandler())

from flask import Flask
from slack import WebClient
from slackeventsapi import SlackEventAdapter
from slack.errors import SlackApiError
from slack_bot import SlackBot

import time
import threading
import rospy
from std_msgs.msg import Float64

# Based in part on Slack's onboarding tutorial https://github.com/slackapi/python-slackclient/blob/master/tutorial/

# Initialize a Flask app to host the events adapter
app = Flask(__name__)
app.debug = False
app.use_reloader = False
slack_events_adapter = SlackEventAdapter(os.environ['SLACK_SIGNING_SECRET'], "/slack/events", app)

# Initialize a Web API client
slack_web_client = WebClient(token=os.environ['SLACK_BOT_TOKEN'])



def welcome_message(channel: str, user_id: str = None):
    slack_bot = SlackBot(channel)

    # Get the onboarding message payload
    message = slack_bot.get_welcome_payload()

    # Post the onboarding message in Slack
    response = slack_web_client.chat_postMessage(**message)

    # Capture the timestamp of the message we've just posted so
    # we can use it to update the message after a user
    # has completed an onboarding task.

    # _____not used but left in for the time being_____
    # slack_bot.timestamp = response["ts"]

def teleop_message(channel: str, user_id: str=None):
    slack_bot = SlackBot(channel)
    
    # Get the onboarding message payload
    message = slack_bot.get_teleop_payload()

    # Post the onboarding message in Slack
    response = slack_web_client.chat_postMessage(**message)


def dummy_cb(msg):
    print("got message")
    if msg.data > 0.5: 
        response = slack_web_client.conversations_list(types="public_channel")
        for channel in response.get("channels"):
            try:
                teleop_message(channel=channel["id"])
            except SlackApiError as e:
                LOGGER.warning("Failed to post to channel '{}'".format(channel["name"]))
                LOGGER.debug(e)


#____________________________ EVENT CALLBACKS ____________________________

# ================ Team Join Event =============== #
# When the user first joins a team, the type of the event will be 'team_join'.
# Here we'll link the onboarding_message callback to the 'team_join' event.
@slack_events_adapter.on("team_join")
def onboarding_message(payload):
    """Create and send an onboarding welcome message to new users. Save the
    time stamp of this message so we can update this message in the future.
    """
    event = payload.get("event", {})

    # Get the id of the Slack user associated with the incoming event
    user_id = event.get("user", {}).get("id")

    # Open a DM with the new user.
    response = slack_web_client.im_open(user_id)
    channel = response["channel"]["id"]

    # Post the onboarding message.
    welcome_message(user_id, channel)


# ============== Message Events ============= #
# When a user sends a DM, the event type will be 'message'.
# Here we'll link the message callback to the 'message' event.
@slack_events_adapter.on("message")
def message(payload):
    """Display the onboarding welcome message after receiving a message
    that contains "start".
    """
    event = payload.get("event", {})

    channel_id = event.get("channel")
    user_id = event.get("user")
    text = event.get("text")


    if text:
        if text.lower() == "start":
            return welcome_message(user_id, channel_id)
        elif text.lower() == "teleop":
            return teleop_message(user_id, channel_id)



if __name__ == "__main__":
    # Dummy subscriber
    print("Starting subscriber")
    rospy.init_node('flask_app_sub')

    def tmp():
        rospy.Subscriber('dummy_topic', Float64, dummy_cb)
    threading.Thread(target=tmp, daemon=True).start()
    
    threading.Thread(target=app.run, kwargs={"port": 3000}, daemon=True).start()
    
    rospy.spin()
    
