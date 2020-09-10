import * as ros from './ros.js';
import './blocks.js';
import {generateCode, generateUserCode} from './python.js';

// Tabs
document.getElementById('tabs').addEventListener('click', function(e) {
	var tab = e.target.getAttribute('data-tab');
	if (tab) {
		for (let elem of e.target.parentElement.querySelectorAll('[data-tab]')) {
			elem.classList.remove('selected');
		}
		e.target.classList.add('selected');
		document.body.setAttribute('data-tab', tab);
	}
});

var workspace = Blockly.inject('blockly', {
	toolbox: document.getElementById('toolbox'),
	grid: {
		spacing: 25,
		length: 3,
		colour: '#ccc',
		snap: true
	},
	zoom: { controls: true, wheel: true },
	media: 'blockly/media/',
});

});

var pythonArea = document.getElementById('python');

// update Python code
workspace.addChangeListener(function(e) {
	pythonArea.innerHTML = generateUserCode(workspace);
	hljs.highlightBlock(pythonArea);
});

workspace.addChangeListener(function(e) {
	var xmlDom = Blockly.Xml.workspaceToDom(Blockly.mainWorkspace);
	var xmlText = Blockly.Xml.domToPrettyText(xmlDom);
	localStorage.setItem("blockly.xml", xmlText);
});

function loadWorkspace() {
	var xmlText = localStorage.getItem("blockly.xml");
	if (xmlText) {
		Blockly.mainWorkspace.clear();
		var xmlDom = Blockly.Xml.textToDom(xmlText);
		Blockly.Xml.domToWorkspace(xmlDom, Blockly.mainWorkspace);
	}
}

loadWorkspace();

var running = false;
var runRequest = false;

new ROSLIB.Topic({ ros: ros.ros, name: ros.priv + 'block', messageType: 'std_msgs/String' }).subscribe(function(msg) {
	workspace.highlightBlock(msg.data);
	runRequest = false;
	update();
});

new ROSLIB.Topic({ ros: ros.ros, name: ros.priv + 'running' }).subscribe(function(msg) {
	running = msg.data;
	runRequest = false;
	if (!running) {
		workspace.highlightBlock('');
	}
	update();
});

var notifElem = document.getElementById('notifications');

function z(n) { return (n < 10 ? '0' : '') + n; } // add leading zero

new ROSLIB.Topic({ ros: ros.ros, name: ros.priv + 'print', messageType: 'std_msgs/String'}).subscribe(function(msg) {
	var d = new Date(); // TODO: use StringStamped?
	var timestamp = `${z(d.getHours())}:${z(d.getMinutes())}:${z(d.getSeconds())}`;
	notifElem.innerHTML += `${timestamp}: ${msg.data}\n`;
	notifElem.scrollTop = notifElem.scrollHeight;
});

new ROSLIB.Topic({ ros: ros.ros, name: ros.priv + 'error', messageType: 'std_msgs/String'}).subscribe(function(msg) {
	alert('Error: ' + msg.data);
});

var runButton = document.getElementById('run');

function update() {
	document.body.classList.toggle('running', running);
	runButton.disabled = !ros.ros.isConnected || runRequest || running;
}

var shownPrompts = new Set();

new ROSLIB.Topic({ ros: ros.ros, name: ros.priv + 'prompt', messageType: 'clover_blocks/Prompt'}).subscribe(function(msg) {
	if (shownPrompts.has(msg.id)) return;
	shownPrompts.add(msg.id);

	var response = prompt(msg.message);
	new ROSLIB.Topic({
		ros: ros.ros,
		name: ros.priv + 'input/' + msg.id,
		messageType: 'std_msgs/String',
		latch: true
	}).publish(new ROSLIB.Message({ data: response }));
});

window.stopProgram = function() {
	ros.stopService.callService(new ROSLIB.ServiceRequest(), function(res) {
		if (!res.success) alert(res.message);
	}, err => alert(err))
}

ros.ros.on('connection', update);

ros.ros.on('close', update);

window.runProgram = function() {
	if (!confirm('Run program?')) return;

	runRequest = true;
	update();
	var code = generateCode(workspace);
	console.log(code);
	ros.runService.callService(new ROSLIB.ServiceRequest({ code: code } ), function(res) {
		if (!res.success) {
			runRequest = false;
			update();
			alert(res.message);
		}
	}, function(err) {
		runRequest = false;
		update();
		alert(err);
	})
}

window.land = function() {
	window.stopProgram();
	ros.landService.callService(new ROSLIB.ServiceRequest(), function(result) {
	}, function(err) {
		alert('Unable to land: ' + err);
	});
}

