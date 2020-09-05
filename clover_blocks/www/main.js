import {ros, runService, stopService, landService} from './ros.js';
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

// TODO: use mutators. bug on restore
workspace.addChangeListener(function(e) {
	if (e instanceof Blockly.Events.Change) {
		let block = workspace.getBlockById(e.blockId);
		if (e.name == 'FRAME_ID') {
			if (block.getInput('X')) { // block has x-y-z fields
				if (e.newValue == 'BODY' || e.newValue == 'NAVIGATE_TARGET' || e.newValue == 'BASE_LINK') {
					block.getInput('X').fieldRow[0].setValue('forward');
					block.getInput('Y').fieldRow[0].setValue('left');
					block.getInput('Z').fieldRow[0].setValue('up');
				} else {
					block.getInput('X').fieldRow[0].setValue('x');
					block.getInput('Y').fieldRow[0].setValue('y');
					block.getInput('Z').fieldRow[0].setValue('z');
				}
			}
			if (block.getInput('ID')) { // block has marker id field
				block.getInput('ID').setVisible(e.newValue == 'ARUCO'); // toggle id field
			}
			block.render();
		}

		if (block.type == 'set_effect') {
			if (e.name == 'EFFECT') {
				let hideColor = e.newValue == 'RAINBOW' || e.newValue == 'RAINBOW_FILL';
				block.inputList[1].setVisible(!hideColor);
				block.render();
			}
		}
	}
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

new ROSLIB.Topic({ ros: ros, name: '/clover_blocks/block', messageType: 'std_msgs/String' }).subscribe(function(msg) {
	console.log(msg);
	workspace.highlightBlock(msg.data);
	runButton.disabled = Boolean(msg.data);
});

new ROSLIB.Topic({ ros: ros, name: '/clover_blocks/print', messageType: 'std_msgs/String'}).subscribe(function(msg) {
	alert(msg.data);
});

new ROSLIB.Topic({ ros: ros, name: '/clover_blocks/error', messageType: 'std_msgs/String'}).subscribe(function(msg) {
	alert('Error: ' + msg.data);
});

new ROSLIB.Topic({ ros: ros, name: '/clover_blocks/prompt', messageType: 'clover_blocks/Prompt'}).subscribe(function(msg) {
	var response = prompt(msg.message);
	new ROSLIB.Topic({
		ros: ros,
		name: '/clover_blocks/input/' + msg.id,
		messageType: 'std_msgs/String',
		latch: true
	}).publish(new ROSLIB.Message({ data: response }));
});

window.stopProgram = function() {
	stopService.callService(new ROSLIB.ServiceRequest(), function(res) {
		if (!res.success) alert(res.message);
	}, err => alert(err))
}

var runButton = document.getElementById('run');

ros.on('connection', function () {
	runButton.disabled = false;
});

ros.on('close', function () {
	runButton.disabled = true;
});

window.runProgram = function() {
	if (!confirm('Run program?')) return;

	runButton.disabled = true;
	var code = generateCode(workspace);
	console.log(code);
	runService.callService(new ROSLIB.ServiceRequest({ code: code } ), function(res) {
		if (!res.success) alert(res.message);
		runButton.disabled = false;
	}, function(err) {
		runButton.disabled = false;
		alert(err);
	})
}

window.land = function() {
	window.stopProgram();
	landService.callService(new ROSLIB.ServiceRequest(), function(result) {
	}, function(err) {
		alert('Unable to land: ' + err);
	});
}

