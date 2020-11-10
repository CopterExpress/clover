/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

const COLOR_FLIGHT = 293;
const COLOR_STATE = 36;
const COLOR_LED = 143;
const COLOR_GPIO = 200;
const DOCS_URL = 'https://clover.coex.tech/en/blocks.html';

var frameIds = [["body", "BODY"], ["markers map", "ARUCO_MAP"], ["marker", "ARUCO"], ["last navigate target", "NAVIGATE_TARGET"], ["map", "MAP"]];

function considerFrameId(e) {
	if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

	var frameId = this.getFieldValue('FRAME_ID');
	// set appropriate coordinates labels
	if (this.getInput('X')) { // block has x-y-z fields
		if (frameId == 'BODY' || frameId == 'NAVIGATE_TARGET' || frameId == 'BASE_LINK') {
			this.getInput('X').fieldRow[0].setValue('forward');
			this.getInput('Y').fieldRow[0].setValue('left');
			this.getInput('Z').fieldRow[0].setValue('up');
		} else {
			this.getInput('X').fieldRow[0].setValue('x');
			this.getInput('Y').fieldRow[0].setValue('y');
			this.getInput('Z').fieldRow[0].setValue('z');
		}
	}
	if (this.getInput('ID')) { // block has marker id field
		this.getInput('ID').setVisible(frameId == 'ARUCO'); // toggle id field
	}
	this.render();
}

function updateSetpointBlock(e) {
	if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

	considerFrameId.apply(this, arguments);

	var type = this.getFieldValue('TYPE');
	var velocity = type == 'VELOCITY';
	var attitude = type == 'ATTITUDE' || type == 'RATES';

	this.getInput('VX').setVisible(velocity);
	this.getInput('VY').setVisible(velocity);
	this.getInput('VZ').setVisible(velocity);
	this.getInput('YAW').setVisible(attitude);
	this.getInput('PITCH').setVisible(attitude);
	this.getInput('ROLL').setVisible(attitude);
	this.getInput('THRUST').setVisible(attitude);
	this.getInput('RELATIVE_TO').setVisible(type != 'RATES');

	if (type == 'RATES') {
		this.getInput('ID').setVisible(false);
	}

	this.render();
}

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
		this.setTooltip("Navigate to the specified point, coordinates are in meters.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
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
		this.setTooltip("Set the drone velocity in meters per second (cancels navigation requests).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['setpoint'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set");
		this.appendDummyInput()
			.appendField(new Blockly.FieldDropdown([["velocity", "VELOCITY"], ["attitude", "ATTITUDE"], ["angular rates", "RATES"]]), "TYPE");
		this.appendValueInput("VX")
			.setCheck("Number")
			.appendField("vx");
		this.appendValueInput("VY")
			.setCheck("Number")
			.appendField("vy");
		this.appendValueInput("VZ")
			.setCheck("Number")
			.appendField("vz");
		this.appendValueInput("PITCH")
			.setCheck("Number")
			.appendField("pitch")
			.setVisible(false);
		this.appendValueInput("ROLL")
			.setCheck("Number")
			.appendField("roll")
			.setVisible(false);
		this.appendValueInput("YAW")
			.setCheck("Number")
			.appendField("yaw")
			.setVisible(false);
		this.appendValueInput("THRUST")
			.setCheck("Number")
			.appendField("thrust")
			.setVisible(false);
		this.appendDummyInput('RELATIVE_TO')
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false);
		this.setInputsInline(false);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Set the drone's setpoints of different types (cancels navigation requests).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(updateSetpointBlock);
	}
};

Blockly.Blocks['rangefinder_distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current rangefinder distance");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['get_position'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["x", "X"], ["y", "Y"], ["z", "Z"], ["vx", "VX"], ["vy", "VY"], ["vz", "VZ"]]), "FIELD")
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns current position or velocity in meters or meters per second.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['get_yaw'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current yaw relative to")
			.appendField(new Blockly.FieldDropdown(frameIds), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false)
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns current yaw in degree (not radian).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
	}
};

Blockly.Blocks['get_attitude'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["pitch", "PITCH"], ["roll", "ROLL"], ["pitch rate", "PITCH_RATE"], ["roll rate", "ROLL_RATE"], ["yaw rate", "YAW_RATE"]]), "FIELD");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns current orientation or angle rates in degree or degree per second (not radian).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['voltage'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current")
			.appendField(new Blockly.FieldDropdown([["total", "VOLTAGE"], ["cell", "CELL_VOLTAGE"]]), "TYPE")
			.appendField("voltage");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns current battery voltage in volts.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};


Blockly.Blocks['armed'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("armed?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns if the drone armed.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};


Blockly.Blocks['mode'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("current flight mode");
		this.setOutput(true, "String");
		this.setColour(COLOR_STATE);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};


Blockly.Blocks['wait_arrival'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("wait arrival");
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Wait until the drone arrives to the navigation target.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['get_time'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("time");
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns current timestamp in seconds.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['arrived'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("arrived?");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns if the drone arrived to the navigation target.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
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
		this.setTooltip("Set an individual LED to specified color.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['set_effect'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("set LED effect")
			.appendField(new Blockly.FieldDropdown([["fill", "FILL"], ["blink", "BLINK"], ["fast blink", "BLINK_FAST"], ["fade", "FADE"], ["wipe", "WIPE"], ["flash", "FLASH"], ["rainbow", "RAINBOW"], ["rainbow fill", "RAINBOW_FILL"]]), "EFFECT");
		this.appendValueInput("COLOR")
			.setCheck("Colour")
			.appendField("with color");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_LED);
		this.setTooltip("Set desired LED strip effect.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);

		this.setOnChange(function(e) {
			if (!(e instanceof Blockly.Events.Change || e instanceof Blockly.Events.Create)) return;

			// Hide color field on some effects
			var effect = this.getFieldValue('EFFECT');
			var hideColor = effect == 'RAINBOW' || effect == 'RAINBOW_FILL';
			this.inputList[1].setVisible(!hideColor);
			this.render();
		});
	}
};

Blockly.Blocks['led_count'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("LED count");
		this.setOutput(true, "Number");
		this.setColour(COLOR_LED);
		this.setTooltip("Returns the number of LEDs (configured in led.launch).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
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
		this.setTooltip("Take off on desired altitude in meters");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
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
		this.setHelpUrl(DOCS_URL + '#' + this.type);
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
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_take_off'] = {
	init: function () {
		this.appendStatementInput("TAKE_OFF")
			.setCheck(null)
			.appendField("When took off");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_landing'] = {
	init: function () {
		this.appendStatementInput("LAND")
			.setCheck(null)
			.appendField("When landed");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['on_armed'] = {
	init: function () {
		this.appendStatementInput("ARMED")
			.setCheck(null)
			.appendField("when armed");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.FieldAngle.WRAP = 180;
Blockly.FieldAngle.ROUND = 10;

Blockly.Blocks['angle'] = {
	init: function () {
		this.appendDummyInput()
			.appendField(new Blockly.FieldAngle(90), "ANGLE");
		this.setOutput(true, "Number");
		this.setColour(230);
		this.setTooltip("");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['set_yaw'] = {
	init: function () {
		this.appendValueInput("YAW")
			.setCheck("Number")
			.appendField("rotate by");
		this.appendDummyInput()
			.appendField("relative to")
			.appendField(new Blockly.FieldDropdown([["body", "body"], ["markers map", "aruco_map"], ["last navigate target", "navigate_target"]]), "FRAME_ID");
		this.appendDummyInput()
			.appendField("wait")
			.appendField(new Blockly.FieldCheckbox("TRUE"), "WAIT");
		this.setInputsInline(true);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setColour(COLOR_FLIGHT);
		this.setTooltip("Rotate the drone to the specified angle in degree (not radian).");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['distance'] = {
	init: function () {
		this.appendDummyInput()
			.appendField("distance to point");
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
			.appendField(new Blockly.FieldDropdown([["markers map", "ARUCO_MAP"], ["marker", "ARUCO"], ["last navigate target", "NAVIGATE_TARGET"]]), "FRAME_ID");
		this.appendValueInput("ID")
			.setCheck("Number")
			.appendField("with ID")
			.setVisible(false);
		this.setInputsInline(false);
		this.setOutput(true, "Number");
		this.setColour(COLOR_STATE);
		this.setTooltip("Returns the distance to the given point in meters.");
		this.setHelpUrl(DOCS_URL + '#' + this.type);
		this.setOnChange(considerFrameId);
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
		this.setHelpUrl(DOCS_URL + '#' + this.type);
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
		this.setHelpUrl(DOCS_URL + '#' + this.type);
	}
};

Blockly.Blocks['gpio_read'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("read GPIO pin");
		this.setOutput(true, "Boolean");
		this.setColour(COLOR_GPIO);
		this.setTooltip("Returns if there is voltage on a GPIO pin.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['gpio_write'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("set GPIO pin");
		this.appendValueInput("LEVEL")
			.setCheck("Boolean")
			.appendField("to");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Set GPIO pin level.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['set_servo'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("set GPIO pin");
		this.appendValueInput("PWM")
			.setCheck("Number")
			.appendField("to PWM");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Set PWM on a GPIO pin to control servo. PWM is specified in range of 500–2500 μs.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};

Blockly.Blocks['set_duty_cycle'] = {
	init: function () {
		this.appendValueInput("PIN")
			.setCheck("Number")
			.appendField("set GPIO pin");
		this.appendValueInput("DUTY_CYCLE")
			.setCheck("Number")
			.appendField("to duty cycle");
		this.setInputsInline(true);
		this.setColour(COLOR_GPIO);
		this.setPreviousStatement(true, null);
		this.setNextStatement(true, null);
		this.setTooltip("Set PWM duty cycle on a GPIO pin (better to control LEDs, etc). Duty cycle is set in range of 0–1.");
		this.setHelpUrl(DOCS_URL + '#GPIO');
	}
};
