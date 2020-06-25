// Connect to ROS

// Connect to ROS bridge using websocket
export function connectToROS(ip) {
	var ros = new ROSLIB.Ros({
		url: 'ws://' + ip + ':9090'
	});
	ros.on('connection', function () {
		console.log('Connected to websocket server.');
	});
	ros.on('error', function (error) {
		console.log('Error connecting to websocket server: ', error);
	});
	ros.on('close', function () {
		console.log('Connection to websocket server closed.');
	});

	return ros 
}

// Echo a given topic a single time
export function echoTopic(ros, topic, messageType) {
	var listener = new ROSLIB.Topic({
		ros : ros,
		name : topic,
		messageType : messageType
	});

	listener.subscribe(function(message) {
		console.log('Received message on ' + listener.name + ':\n');
		printJointStates(message);
		listener.unsubscribe();
	});
}

// Publish a given command to the given topic
export function publishTopic(publisher, cmd) {
	console.log('To ' + publisher.name);
	console.log('Publishing command of type ' + publisher.messageType);
	console.log(cmd)
	publisher.publish(cmd)
}

// Print formatted JointState message
function printJointStates(message) {
	var names = message.name;
	var pos = message.position;
	var vel = message.velocity;
	names.forEach((name, i) => {
		console.log('%s:\n\t pos: %f vel: %f', name, 
			pos[i], 
			vel[i]);
	});
}
