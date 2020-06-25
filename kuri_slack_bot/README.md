# Kuri Slack Bot

## Usage 

Build with catkin after cloning. Then, you can run `roslaunch kuri_slack_bot slack_bot.launch` to start a dummy publisher for testing purposes if you want to.

Run `pip install -r requirements.txt` using the given requirements.txt to install the required libraries.

To start the server which responds to slack events simply run app.py after installing the dependencies. This is only a local server, though. You can use ngrok to get a publicly facing url and then pass that into your Slack bot's events configuration setting (done Slack's online app interface).

`app.py` and `slack_bot.py` are configured to respond to `start` and `teleop` sent to channels in which the slack bot is installed. Additionally, if the dummy publisher is running, the slack bot will periodically send requests for help to these same installed channels.

> Maintained by Nikita Filippov
