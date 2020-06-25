# Kuri Web-Teleop Interface

## Local Usage 

Build with catkin after cloning and installing the dependencies described in the wiki. Then you can run `roslaunch web_teleop web_teleop.launch` to start `rosbridge` and `web_video_server`. 

Then, navigate to the `frontend/` directory and run a local http server (i.e. using something like npm's `http-server` or `python -m http.server`).

## More Setup Instructions

### Install `rosbridge` and `web_video_server`

```bash
sudo apt install ros-melodic-rosbridge-suite
sudo apt install ros-melodic-web-video-server
```

### Configure your environment variables
Set the ROS_IP for Kuri to whatever IP you can ping Kuri with from your VM and, likewise, set ROS_IP for your VM to whatever you can ping your VM with from Kuri (or from your host computer, for that matter).

Set the ROS_MASTER_URI for both Kuri and your VM to `http://*kuri_ip*:11311`, with *kuri_ip* being the IP used for Kuri's ROS_IP. This value tells both machines where to find the machine running roscore (and 11311 is just the standard port used by ROS).

You can streamline this by adding the following to a bash file on Kuri:

```bash
export ROS_IP=xxx.xxx.xxx.xxx;
export ROS_MASTER_URI=http://xxx.xxx.xxx.xxx:11311;
```

and on your VM (or whatever the host computer is):

```bash
export ROS_IP=yyy.yyy.yyy.yyy;
export ROS_MASTER_URI=http://xxx.xxx.xxx.xxx:11311;
```

### Download the heatmapjs folder and put `heatmap.js` in src/

Link to downloads/docs: [here](https://www.patrick-wied.at/static/heatmapjs/)

### Starting up Kuri and rosbridge/web_video_server
SSH into Kuri and run `roslaunch kuri_launch kuri.launch`. On your VM, run `roslaunch web_teleop web_teleop.launch`. 


### Running the web interface
You need some sort of way to quickly set up a local http server. If you have python3 installed, change to the frontend dir and then start a server (the command differs slightly for python2):

```bash
cd ... frontend/
python -m http.server
```

This will launch a local server in the frontend/ directory. Alternatively, you can use `http-server`:

```bash
npm install --global http-server 
cd ... frontend/
http-server
```

This is a little more convenient and better than the python solution but it's not important either way.

### Controlling Kuri
In a browser, enter `localhost:8080` (or whatever port your server's on) and you'll be met with a blank white web page. Open the browser's debugger (f12 in chrome) and navigate to the console. You'll see some log output.

Now, click on the white page to make sure it's selected. Your keyboard now controls Kuri.

### Controls

`wasd` - to move around. 

`arrowkeys` - to move Kuri's head around

### Making your own modifications

Currently, the only way to add your own keyboard bindings is to clone this repo and edit the keyboard interface class in the keyboard.js file. 

> Maintained by Nikita Filippov
