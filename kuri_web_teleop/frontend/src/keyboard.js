import { publishTopic } from "./utils.js";

// ToDo: Remove commented out double-keypress code

const HARD_STOP = ' ';

const movementMap = {
  'w': [1, 0],
  'a': [0, 1],
  's': [-1, 0],
  'd': [0, -1],
  HARD_STOP: [0, 0]  // hard stop (mapped-to value not actually used)
};

// r/v : tilt head up/down
// d/g : pan head left/right

const HEAD_MAP = {
  'ArrowUp': [0, -1],
  'ArrowDown': [0, 1],
  'ArrowLeft': [1, 0],
  'ArrowRight': [-1, 0]
}

// Change if you want bindings for eye controls
const EYE_MAP = {
  // 'c': ['yeet']
};

const CENTER_HEAD = 'c';

// Topics
const JOINTS_TOPIC = '/joint_states';
const VELOCITY_TOPIC = '/mobile_base/commands/velocity';
const EYELIDS_TOPIC = '/eyelids_controller/command';
const HEAD_TOPIC = '/head_controller/command';

// Message types
const TWIST_TYPE = 'geometry_msgs/Twist';
const JOINT_STATE_TYPE = 'sensor_msgs/JointState';
const JOINT_TRAJECTORY_TYPE = 'trajectory_msgs/JointTrajectory';
const JOINT_TRAJECTORY_POINT_TYPE = 'trajectory_msgs/JointTrajectoryPoint'

const LIN_SPEED = 0.2;
const ANG_SPEED = 0.4;

const HEAD_PAN_SPEED = 0.2;
const HEAD_TILT_SPEED = 0.1;


// Key is pressed
// key_pressed = true (interval: if ... then send command)
// keyDownSet is updated to contain the pressed key and this is used in the above callback

// so key press has to trigger both an update of the set and creation of interval with a set-dependent callback
// So the callback has to be defined in the scope of the set


// A keyboard interface for users to send commands to Kuri
export class Interface {
  constructor(element, ros) {
    console.log("constructed keyboard Interface");
    this.element = element;
    this.ros = ros;
    this.keyDownSet = new Set();

    this.delta = 500;

    // Records the latest known position of each of the robots' joints
    this.latestPositions = {};
    
    // Listener(s)/Publisher(s)
    this.statesSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: JOINTS_TOPIC,
      messageType: JOINT_STATE_TYPE
    })
    this.velocityPublisher = new ROSLIB.Topic({
      ros: ros,
      name: VELOCITY_TOPIC,
      messageType: TWIST_TYPE
    });
    this.eyesPublisher = new ROSLIB.Topic({
      ros: ros, 
      name: EYELIDS_TOPIC,
      messageType: JOINT_TRAJECTORY_TYPE
    });
    this.headPublisher = new ROSLIB.Topic({
      ros: ros, 
      name: HEAD_TOPIC,
      messageType: JOINT_TRAJECTORY_TYPE
    });

    // configure the joint states subscription
    this.statesSubscriber.subscribe(this.stateUpdaterCallback.bind(this));

    // tracks intervals used by this class
    this.intervals = {};
  }

  // Updates the latest positions for Kuri's various joints
  stateUpdaterCallback(msg) {
    let positions = {};
    msg.name.forEach((x, i) => {
      positions[x] = msg.position[i];
    });
    this.latestPositions = positions;
  }

  kDownEvent(event) {
    this.keyDownSet.add(event.key);
    event.preventDefault();

    return;
  }
  
  kUpEvent(event) {
    this.keyDownSet.delete(event.key);
    event.preventDefault();

    // If this is the last key then stop movement
    if (this.keyDownSet.size == 0) this.eStop();
    return;
  }

  // Handle key presses
  handleKeyDownSet() {
    // Do nothing if no keys are pressed
    if (this.keyDownSet.size == 0) return;

    let lin_x = 0;
    let ang_z = 0;
    let goingBackwards = false;

    let movement = false;

    this.keyDownSet.forEach(function(key) {
      if (key == HARD_STOP) {
        this.eStop();
      } else if (key in movementMap) { 
        // // Check for double keypress
        let doublePressMod = 1.0;
        
        // Update movement variables (before publishing them)
        lin_x += doublePressMod * LIN_SPEED * movementMap[key][0];
        ang_z += doublePressMod * ANG_SPEED * movementMap[key][1];
        
        // track whether the robot is moving and/or reversing
        movement = true;
        if (key == 's') goingBackwards = true;
      } else if (key in HEAD_MAP) {
        console.log("head stuff");
        let pan = HEAD_PAN_SPEED * HEAD_MAP[key][0] + this.latestPositions["head_1_joint"];
        let tilt = HEAD_TILT_SPEED * HEAD_MAP[key][1] + this.latestPositions["head_2_joint"];
        console.log(pan, tilt);
        this.publishHeadCmd(pan, tilt);
      } else if (key in EYE_MAP) {
        console.log("eye stuff");
      } else if (key == CENTER_HEAD) {
        this.centerHead();
      }
    }.bind(this));

    if (movement) {
      // Generate and send the appropriate message
      this.publishVelocityCmd.bind(this)(lin_x, ang_z, goingBackwards);
    }
  }

  eStop() {
    const twist = new ROSLIB.Message({
      linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      },
      angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      }
    });
    publishTopic(this.velocityPublisher, twist);
  }

  publishVelocityCmd(lin_x, ang_z, backwards=false) {
    // If reversing, rotational inputs should be inverted
    const twist = new ROSLIB.Message({
      linear : {
        x : lin_x,
        y : 0.0,
        z : 0.0
      },
      angular : {
        x : 0.0,
        y : 0.0,
        z : backwards ? -ang_z : ang_z
      }
    });
    publishTopic(this.velocityPublisher, twist);
  }

  publishHeadCmd(pan, tilt) {
    const traj = new ROSLIB.Message({
      joint_names: ["head_1_joint", "head_2_joint"],
      points: [{
        positions: [pan, tilt],
        velocities: [0, 0],
        effort: [],
        time_from_start: {
          secs: 1 // might need tweaking (rospy.Time(1))
        }
      }]
    });
    publishTopic(this.headPublisher, traj);
  }

  // pan = 0.4 * head_bindings[key][0] + latest_positions["head_1_joint"]
  // tilt = 0.2 * head_bindings[key][1] + latest_positions["head_2_joint"]

  publichEyesCmd(position) {
    const traj = new ROSLIB.Message({
      joint_names: ["eyelids_joint"],
      points: [{
        positions: [position],
        velocities: [],
        effort: [],
        time_from_start: {
          secs: 1 // might need tweaking (rospy.Time(1))
        }
      }]
    });
    publishTopic(this.eyesPublisher, traj);
  }

  centerHead() {
    this.publishHeadCmd(0, 0)
  }

  openEyes() {
    this.publichEyesCmd(0.04);
  }

  closeEyes() {
    this.publishEyesCmd(1.0);
  }
  
  // takes arbitrary number of key name arguments in addition to name and callback
  // i.e. watch('name', callback, 'key1, 'key2', ...);
  watch(name) {
    console.log("Watching...");
    this.openEyes()
    this.attach();    
    this.handleKeyDownSet.bind(this)();
    this.intervals[name] = setInterval(this.handleKeyDownSet.bind(this), 200);
  }

  unwatch(name) {
    this.detach();
    clearInterval(this.intervals[name]);
    delete this.intervals[name];
  }
  
  // Add the event listener
  attach() {
    this.element.addEventListener("keydown", this.kDownEvent.bind(this));
    this.element.addEventListener("keyup", this.kUpEvent.bind(this));
  }
  
  detach() {
    this.element.removeEventListener("keydown", this.kDownEvent);
    this.element.removeEventListener("keyup", this.kUpEvent);
  }
}
