var url = 'ws://192.168.11.1:9090';
var modeEl = document.querySelector('.telemetry .mode');
var batteryEl = document.querySelector('.battery');

var ros = new ROSLIB.Ros({ url: url });

ros.on('connection', function () {
	body.classList.add('connected');
});

ros.on('close', function () {
	body.classList.remove('connected');
	modeEl.classList.remove('armed');
	modeEl.innerHTML = 'DISCONNECTED';
	batteryEl.innerHTML = '';
	setTimeout(function() {
		modeEl.innerHTML = 'RECONNECTING';
		ros.connect(url);
	}, 2000);
});

var fcuState;

new ROSLIB.Topic({
	ros: ros,
	name: '/state_latched',
	messageType: 'mavros_msgs/State'
}).subscribe(function(message) {
	body.classList.toggle('fcu-disconnected', !message.connected);
	body.classList.toggle('armed', message.armed);
	fcuState = message;
	modeEl.classList.toggle('armed', fcuState.armed);
	modeEl.innerHTML = message.connected ? fcuState.mode : 'DISCONNECTED FROM FCU';
	console.log('state', message);
});

function notifyLowBattery() {
	callNativeApp('lowBattery');
}

notifyLowBatteryThrottled = throttle(notifyLowBattery, 10000);

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/battery',
	messageType: 'sensor_msgs/BatteryState',
	throttle_rate: 5000
}).subscribe(function(message) {
	var LOW_BATTERY = 3.8;
	batteryEl.innerHTML = (message.cell_voltage[0].toFixed(2) + ' V') || '';

	if (message.cell_voltage[0] < LOW_BATTERY) {
		console.log('low battery');
		callNativeApp('lowBattery');
		body.classList.remove('low-battery');
		void body.offsetWidth; // trick for repeating animation
		body.classList.add('low-battery');
	} else {
		body.classList.remove('low-battery');
	}
});

new ROSLIB.Topic({
	ros: ros,
	name: '/rosout_agg',
	messageType: 'rosgraph_msgs/Log'
}).subscribe(function(message) {
	var BLACKLIST = ['CMD: ', 'PR: ', 'DROPPED', 'Clock skew detected', 'MANUAL CONTROL LOST'];
	if (message.level >= 4) {
		if (BLACKLIST.some(function(e) {
				return message.msg.indexOf(e) != -1;
			})) {
			console.log('Filtered out message ' + message.msg);
			return;
		}
		callNativeApp('notification', message);
	}
});

var setMode = new ROSLIB.Service({
	ros: ros,
	name : '/mavros/set_mode',
	serviceType : 'mavros_msgs/SetMode'
});

function setControlMode() {
	var CONTROL_MODE = 'STABILIZED';
	setMode.callService(new ROSLIB.ServiceRequest({ custom_mode: CONTROL_MODE }));
}
