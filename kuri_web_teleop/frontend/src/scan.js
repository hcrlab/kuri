const scanTopic = "/scan";
const scanMsgType = "sensor_msgs/LaserScan";

const JOINTS_TOPIC = '/joint_states';
const JOINT_STATE_TYPE = 'sensor_msgs/JointState';

export class ScanInterface {
  constructor(element, ros) {
    this.ros = ros;
    this.element = element;
    this.heatmapInstance = h337.create({
      container: element,
      maxOpacity: 1.0,
      minOpacity: 0.0
    });

    this.latestPositions = {};

    this.scanSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: scanTopic,
      messageType: scanMsgType
    });
    this.scanSubscriber.subscribe(this.scanUpdaterCallback.bind(this));

    this.statesSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: JOINTS_TOPIC,
      messageType: JOINT_STATE_TYPE
    })
    this.statesSubscriber.subscribe(this.jointsUpdaterCallback.bind(this));
  }

  scanUpdaterCallback(msg) {
    const radScale = 1.5;
    // const tooClose = 1;

    const width = this.element.getBoundingClientRect().width;
    const height = this.element.getBoundingClientRect().height;
    const maxRange = msg.range_max;

    // filter weird values
    const rangesFiltered = msg.ranges.filter(function(val) {
      return !!val;
    });

    const minDetected = Math.min(...rangesFiltered);
    const maxDetected = Math.max(...rangesFiltered);
    
    const points = msg.ranges.map(function(val, i) {
      return {
        x: Math.floor(i / msg.ranges.length * width),
        y: Math.floor(height / 2),
        value: (maxDetected - val) / (maxDetected - minDetected) * radScale
      };
    });

    const data = {
      max: maxRange,
      data: points
    };

    this.heatmapInstance.setData(data);

    let canvas = this.element.children[0];
    
    if (canvas.getContext) 
    {
      var context = canvas.getContext('2d');
      // Reset the current path
      context.beginPath(); 
      context.moveTo(width / 2, 0);
      context.lineTo(width / 2, height);
      context.lineWidth = 3;
      context.stroke();

      // calculate where the line should be drawn
      const currPos = this.latestPositions["head_1_joint"];
      const pxPos = (1 - (currPos + 0.75) / 1.5) * width;
      context.beginPath()
      context.moveTo(pxPos, 0);
      context.lineTo(pxPos, height);
      context.lineWidth = 7;
      context.stroke();
    }
  }

  jointsUpdaterCallback(msg) {
    let positions = {};
    msg.name.forEach((x, i) => {
      positions[x] = msg.position[i];
    });
    this.latestPositions = positions;
  }
}
