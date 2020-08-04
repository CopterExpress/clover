var workspace = Blockly.inject('blockly', { toolbox: document.getElementById('toolbox') });

workspace.addChangeListener(function (e) {
	if (e instanceof Blockly.Events.Change) {
		let block = workspace.getBlockById(e.blockId);
		if (e.name == 'FRAME_ID') {
			if (block.getInput('X')) { // block has x-y-z fields
				if (e.newValue == 'body' || e.newValue == 'navigate_target' || e.newValue == 'base_link') {
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
				block.getInput('ID').setVisible(e.newValue == 'aruco'); // toggle id field
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

const COLOR_FLIGHT = 293;
const COLOR_STATE = 36;
const COLOR_LED = 143;

var frameIds = [["body", "body"], ["markers map", "aruco_map"], ["marker", "aruco"], ["last navigate target", "navigate_target"]];

Blockly.Blocks['navigate'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("navigate to point");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("forward");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("left");
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("up");
		this.appendDummyInput()
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.appendValueInput("SPEED")
			.setCheck("Number")
			.appendField("with speed");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['set_velocity'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set velocity");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("forward");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("left");
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("up");
		this.appendDummyInput()
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['rangefinder_distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current rangefinder distance");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['get_position'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["x", "X"], ["y", "Y"], ["z", "Z"], ["yaw", "YAW"], ["vx", "VX"], ["vy", "VY"], ["vz", "VZ"]]), "FIELD")
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['get_attitude'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["pitch", "PITCH"], ["roll", "ROLL"], ["pitch rate", "PITCH_RATE"], ["roll rate", "ROLL_RATE"], ["yaw rate", "YAW_RATE"]]), "FIELD");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['voltage'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["total", "TOTAL"], ["cell", "CELL"]]), "NAME")
			.appendField("voltage");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};


Blockly.Blocks['armed'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("armed?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};


Blockly.Blocks['mode'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current flight mode");
		this.setOutput(true, "String");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};


Blockly.Blocks['wait_arrival'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("wait arrival");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};


Blockly.Blocks['arrived'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("arrived?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['set_led'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set LED");
		this.appendValueInput("INDEX")
			.setCheck("Number");
		this.appendValueInput("COLOR")
			.setCheck("Colour")
			.appendField("to color");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_LED);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['set_effect'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set LED effect to")
			.appendField(new Blockly.FieldDropdown([["fill", "FILL"], ["blink", "BLINK"], ["fast blink", "BLINK_FAST"], ["fade", "FADE"], ["wipe", "WIPE"], ["flash", "FLASH"], ["rainbow", "RAINBOW"], ["rainbow fill", "RAINBOW_FILL"]]), "EFFECT");
		this.appendValueInput("COLOR")
			.setCheck("Colour")
			.appendField("with color");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_LED);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['take_off'] = {
	init: function () {
		this.appendValueInput("ALT")
			.setCheck("Number")
			.appendField("take off to");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['land'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("land");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['global_position'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["latitude", "LATITUDE"], ["longitude", "LONGITUDE"], ["altitude", "ALTITUDE"]]), "NAME");
		this.setOutput(true, "Number");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['on_take_off'] = {
	init: function () {
		this.appendStatementInput("TAKE_OFF")
			.setCheck(null)
			.appendField("When took off");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['on_landing'] = {
	init: function () {
		this.appendStatementInput("LAND")
			.setCheck(null)
			.appendField("When landed");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['on_armed'] = {
	init: function () {
		this.appendStatementInput("ARMED")
			.setCheck(null)
			.appendField("when armed");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['angle'] = {
	init: function () {
		this.appendDummyInput()
			.appendField(new Blockly.FieldAngle(90), "NAME");
		this.setOutput(true, "Number");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['set_yaw'] = {
	init: function () {
		this.appendValueInput("YAW")
			.setCheck("Number")
			.appendField("rotate on");
		this.appendDummyInput()
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown([["body", "body"], ["markers map", "aruco_map"], ["last navigate target", "navigate_target"]]), "FRAME_ID");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("distance to");
		this.appendValueInput("X")
			.setCheck("Number")
			.appendField("x");
		this.appendValueInput("Y")
			.setCheck("Number")
			.appendField("y");
		this.appendValueInput("Z")
			.setCheck("Number")
			.appendField("z");
		this.appendDummyInput()
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown([["markers map", "aruco_map"], ["marker", "aruco"], ["last navigate target", "navigate_target"]]), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false);
		this.setInputsInline(false);
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

Blockly.Blocks['wait'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("wait");
		this.appendValueInput("TIME")
			.setCheck("Number");
		this.appendDummyInput()
			.appendField("seconds");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};

var keys = [['up', 'UP'], ['down', 'DOWN'], ['left', 'LEFT'], ['right', 'RIGHT'], ['space', 'SPACE']];

Blockly.Blocks['key_pressed'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("key")
			.appendField(new Blockly.FieldDropdown(keys, "NAME"))
			.appendField("pressed");
		this.appendStatementInput("PRESSED")
			.setCheck(null);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl("");
	}
};
