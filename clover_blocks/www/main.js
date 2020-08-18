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
workspace.addChangeListener(function (e) {
	pythonArea.innerHTML = generateUserCode();
	hljs.highlightBlock(pythonArea);
});
