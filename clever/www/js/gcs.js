var body = document.querySelector('body');
var titleEl = document.querySelector('title');
var modeEl = document.querySelector('.mode');
var batteryEl = document.querySelector('.battery');

var url = 'ws://' + location.hostname + ':9090';
var ros = new ROSLIB.Ros({ url: url });

function speak(txt) {
	var utterance = new SpeechSynthesisUtterance(txt);
	window.speechSynthesis.speak(utterance);
}

ros.on('connection', function () {
	body.classList.add('connected');
	titleEl.innerText = 'Connected';
});

ros.on('close', function () {
	titleEl.innerText = 'Disconnected';
	modeEl.innerHTML = '';
	body.classList.remove('connected');
	setTimeout(function() {
		titleEl.innerText = 'Reconnecting';
		ros.connect(url);
	}, 2000);
});

var fcuState = {};

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/state',
	messageType: 'mavros_msgs/State'
}).subscribe(function(msg) {
	modeEl.innerHTML = msg.mode;
	if (fcuState.mode != msg.mode) {
		// mode changed
		speak(msg.mode + ' flight mode');
	}
	fcuState = msg;
});

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/statustext/recv',
	messageType: 'mavros_msgs/StatusText'
}).subscribe(function(message) {
	var BLACKLIST = [];
	if (message.severity <= 4) {
		if (BLACKLIST.some(function(e) {
				return message.text.indexOf(e) != -1;
			})) {
			console.log('Filtered out message ' + message.text);
			return;
		}
		speak(message.text);
	}
});

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/battery',
	messageType: 'sensor_msgs/BatteryState',
	throttle_rate: 5000
}).subscribe(function(message) {
	var LOW_BATTERY = 3.8;
	batteryEl.innerHTML = (message.cell_voltage[0].toFixed(2) + ' V') || '';
});
