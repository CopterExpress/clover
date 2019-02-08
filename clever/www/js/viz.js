var ros = new ROSLIB.Ros({
	url : 'ws://' + location.host + ':9090'
});

var titleEl = document.querySelector('title');

ros.on('error', function(error) {
	titleEl.innerText = 'Disconnected';
	err = error;
	alert('Connection error: please enable \'rosbridge\' in clever.launch!');
});

ros.on('connection', function() {
	console.log('connected');
	titleEl.innerText = 'Connected';
});

ros.on('close', function() {
	console.log('disconnected');
	titleEl.innerText = 'Disconnected';
});

var viewer = new ROS3D.Viewer({
	divID: 'viz',
	width: 1000,
	height: 600,
	antialias: true
});

var tfClient = new ROSLIB.TFClient({
	ros: ros,
	angularThres: 0.01,
	transThres: 0.01,
	rate: 10.0,
	fixedFrame : 'map'
});

var vehicleMarkers = new ROS3D.MarkerArrayClient({
	ros: ros,
	tfClient: tfClient,
	topic: '/vehicle_marker',
	rootObject: viewer.scene
});

var cameraMarkers = new ROS3D.MarkerArrayClient({
	ros: ros,
	tfClient: tfClient,
	topic: '/main_camera/camera_markers',
	rootObject: viewer.scene
});

var map = new ROS3D.Grid({
	ros: ros,
	tfClient: tfClient,
	// frameID: 'map',
	rootObject: viewer.scene
});

viewer.scene.add(map);
