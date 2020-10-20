/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

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

function readParams() {
	return Promise.all([
		ros.readParam('navigate_tolerance', true, 0.2),
		ros.readParam('yaw_tolerance', true, 20),
		ros.readParam('sleep_time', true, 0.2),
		ros.readParam('confirm_run', true, true),
	]);
}

var ready = readParams(); // initialization complete promise

var pythonArea = document.getElementById('python');

// update Python code
workspace.addChangeListener(function(e) {
	ready.then(function() {
		pythonArea.innerHTML = generateUserCode(workspace);
		hljs.highlightBlock(pythonArea);
	});
});

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
	var d = new Date(); // TODO: use rosgraph_msgs/Log?
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
	}).publish(new ROSLIB.Message({ data: response || '' }));
});

window.stopProgram = function() {
	ros.stopService.callService(new ROSLIB.ServiceRequest(), function(res) {
		if (!res.success) alert(res.message);
	}, err => alert(err))
}

ros.ros.on('connection', update);

ros.ros.on('close', update);

ready.then(() => runButton.disabled = false);

window.runProgram = function() {
	if (ros.params.confirm_run && !confirm('Run program?')) return;

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

function getProgramXml() {
	var xmlDom = Blockly.Xml.workspaceToDom(workspace);
	return Blockly.Xml.domToPrettyText(xmlDom);
}

function setProgramXml(xml) {
	workspace.clear();
	if (xml) {
		let xmlDom = Blockly.Xml.textToDom(xml);
		Blockly.Xml.domToWorkspace(xmlDom, workspace);
	}
}

workspace.addChangeListener(function(e) {
	localStorage.setItem('xml', getProgramXml());
});

var programSelect = document.querySelector('#program-name');
var userPrograms = programSelect.querySelector('optgroup[data-type=user]');
var examplePrograms = programSelect.querySelector('optgroup[data-type=example]');

var programs = {};
var program = '';

function loadWorkspace() {
	var xml = localStorage.getItem('xml');
	if (xml) {
		setProgramXml(xml);
	}
	program = localStorage.getItem('program') || '';
}

loadWorkspace();

function loadPrograms() {
	ros.loadService.callService(new ROSLIB.ServiceRequest(), function(res) {
		if (!res.success) alert(res.message);

		for (let i = 0; i < res.names.length; i++) {
			let name = res.names[i];
			let program = res.programs[i];
			let option = document.createElement('option');
			option.innerHTML = res.names[i];
			if (name.startsWith('examples/')) {
				examplePrograms.appendChild(option);
			} else {
				userPrograms.appendChild(option);
			}

			programs[name] = program;
		}

		if (program) {
			programSelect.value = program;
		}
		updateChanged();
	}, function(err) {
		document.querySelector('.backend-fail').style.display = 'inline';
		alert(`Error loading programs list.\n\nHave you enabled clover_blocks in clover.launch?`);
		runButton.disabled = true;
	})
}

loadPrograms();

function getProgramName() {
	if (programSelect.value.startsWith('@')) {
		return ''
	}
	return programSelect.value;
}

function updateChanged() {
	var name = program;
	document.body.classList.toggle('changed', name in programs && (programs[name].trim() != getProgramXml().trim()));
}

workspace.addChangeListener(function(e) {
	if (e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create || e instanceof Blockly.Events.Delete) {
		updateChanged();
	}
});

function saveProgram() {
	var name = getProgramName();

	if (!name) {
		name = prompt('Enter new program name:');
		if (!name) {
			programSelect.value = program;
			return;
		}
		if (!name.endsWith('.xml')) {
			name += '.xml';
		}
		let option = document.createElement('option');
		option.innerHTML = name;
		userPrograms.appendChild(option);
	}

	let xml = getProgramXml();
	ros.storeService.callService(new ROSLIB.ServiceRequest({
		name: name,
		program: xml
	}), function(result) {
		if (!result.success) {
			alert(result.message);
			return;
		}
		console.log(result.message);
		programSelect.blur();
		program = name;
		localStorage.setItem('program', name);
		programs[name] = xml;
		programSelect.value = program;
		updateChanged();
	}, function(err) {
		// TODO: restore previous state correctly
		alert('Unable to store: ' + err);
		programSelect.blur();
		programSelect.value = program;
	});
}

window.addEventListener('keydown', function(e) {
	if ((e.metaKey || e.ctrlKey) && e.key == 's') { // ctrl+s/cmd+s
		e.preventDefault();
		if (!document.body.classList.contains('changed')) { // if not changed, ignore
			return;
		}
		saveProgram();
	}
});

programSelect.addEventListener('change', function(e) {
	if (programSelect.value == '@clear') {
		if (!confirm('Clear workspace?')) {
			programSelect.value = program;
			return;
		}
		localStorage.removeItem('program');
		program = '';
		setProgramXml('');
		programSelect.value = program;
		programSelect.blur();
	} else if (programSelect.value == '@save') {
		saveProgram();
	} else {
		// load program
		if (program == '' || document.body.classList.contains('changed')) {
			if (!confirm('Discard changes?')) {
				programSelect.value = program;
				return;
			}
		}
		let name = programSelect.value;
		let lastProgram = getProgramXml();
		programSelect.blur();
		try {
			setProgramXml(programs[name]);
			program = name;
			localStorage.setItem('program', name);
		} catch(e) {
			alert(e);
			setProgramXml(lastProgram);
			program = ''
			programSelect.value = program;
		}
		updateChanged();
	}
});
