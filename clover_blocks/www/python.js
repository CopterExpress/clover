// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('rospy,srv,Trigger,get_telemetry,navigate,set_velocity,land');
Blockly.Python.addReservedWords('block_pub,print_pub,prompt_pub,error_pub,_except_hook');
Blockly.Python.addReservedWords('prompt,navigate_wait,land_wait,wait_arrival,wait_yaw,get_distance');
Blockly.Python.addReservedWords('SetLEDEffect,set_effect');
Blockly.Python.addReservedWords('SetLEDs,LEDState,set_leds');

var userCode = true; // global flag indicating whether the code for GUI is generating

const EXCEPT_HOOK = `\ndef _except_hook(exctype, value, traceback):
    print(value)
    error_pub.publish(str(value))
    block_pub.publish('')

sys.excepthook = _except_hook\n`;

const IMPORT_SRV = `from clover import srv
from std_srvs.srv import Trigger`;

const OFFBOARD = `get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)\n`;

// TODO: tolerance to parameters
const NAVIGATE_WAIT = `\ndef navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)\n`;

const LAND_WAIT = `\ndef land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)\n`;

// TODO: tolerance to parameters
const WAIT_YAW = `\ndef wait_yaw():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.yaw) < math.radians(20):
            return
        rospy.sleep(0.2)\n`;

// TODO: tolerance to parameters
const WAIT_ARRIVAL = `\ndef wait_arrival():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
            return
        rospy.sleep(0.2)\n`;

// TODO: tolerance to parameters
const ARRIVED = `\ndef arrived():
    telem = get_telemetry(frame_id='navigate_target')
    return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2\n`

const GET_DISTANCE = `\ndef get_distance(x, y, z, frame_id):
    telem = get_telemetry(frame_id)
    return math.sqrt((x - telem.x) ** 2 + (y - telem.y) ** 2 + (z - telem.z) ** 2)\n`;

const PROMPT = `\ndef prompt(message):
    prompt_id = str(uuid.uuid4()).replace('-', '')
    prompt_pub.publish(message=message, id=prompt_id)
    return rospy.wait_for_message('/clover_blocks/input/' + prompt_id, String, timeout=30).data\n`;

var rosDefinitions = {};

function generateROSDefinitions() {
	// order for ROS definitions is significant, so generate all ROS definitions as one
	var code = `rospy.init_node('flight', anonymous=True)\n\n`;
	if (!userCode) {
		Blockly.Python.definitions_['import_string'] = 'from std_msgs.msg import String';
		Blockly.Python.definitions_['import_sys'] = 'import sys';
		code += `block_pub = rospy.Publisher('/clover_blocks/block', String, queue_size=10, latch=True)\n`;
		code += `error_pub = rospy.Publisher('/clover_blocks/error', String, queue_size=10, latch=True)\n`;
		code += EXCEPT_HOOK + '\n'; // handle all global exceptions
	}
	if (rosDefinitions.print) {
		Blockly.Python.definitions_['import_string'] = `from std_msgs.msg import String`;
		code += `print_pub = rospy.Publisher('/clover_blocks/print', String, queue_size=10)\n`
	}
	if (rosDefinitions.prompt) {
		Blockly.Python.definitions_['import_prompt'] = 'from clover_blocks.msg import Prompt';
		Blockly.Python.definitions_['import_uuid'] = 'import uuid';
		code += `prompt_pub = rospy.Publisher('/clover_blocks/prompt', Prompt, queue_size=10, latch=True)\n`;
	}
	if (rosDefinitions.offboard) {
		code += OFFBOARD;
	}
	if (rosDefinitions.setEffect) {
		Blockly.Python.definitions_['import_led_effect'] = 'from clover.srv import SetLEDEffect';
		code += `set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)\n`;
	}
	if (rosDefinitions.setLeds) {
		Blockly.Python.definitions_['import_set_led'] = 'from led_msgs.srv import SetLEDs\nfrom led_msgs.msg import LEDState';
		code += `set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs)\n`;
	}
	if (rosDefinitions.prompt) {
		Blockly.Python.definitions_['import_string'] = 'from std_msgs.msg import String';
		code += PROMPT;
	}
	if (rosDefinitions.navigateWait) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += NAVIGATE_WAIT;
	}
	if (rosDefinitions.landWait) {
		code += LAND_WAIT;
	}
	if (rosDefinitions.waitArrival) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_ARRIVAL;
	}
	if (rosDefinitions.arrived) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += ARRIVED;
	}
	if (rosDefinitions.waitYaw) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += WAIT_YAW;
	}
	if (rosDefinitions.distance) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += GET_DISTANCE;
	}
	Blockly.Python.definitions_['ros'] = code;
}

function initNode() {
	Blockly.Python.definitions_['import_rospy'] = 'import rospy';
	generateROSDefinitions();
}

function simpleOffboard() {
	rosDefinitions.offboard = true;
	Blockly.Python.definitions_['import_srv'] = IMPORT_SRV;
	initNode();
}


var autoArm = true;

// Adjust indentation
Blockly.Python.INDENT = '    ';

export function generateUserCode(workspace) {
	userCode = true;
	autoArm = true;
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	userCode = false;
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = 'block_pub.publish(%1)\n';
	var code = Blockly.Python.workspaceToCode(workspace);
	code += "block_pub.publish('')\n"; // end of program
	return code;
}

function getAutoArm() {
	if (autoArm) {
		autoArm = false;
		return true;
	}
	return false;
}

function buildFrameId(block) {
	let frame = block.getFieldValue('FRAME_ID').toLowerCase();
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	// TODO: check empty aruco id
	return frame + (frame == 'aruco' ? '_' + id : '');
}

Blockly.Python.navigate = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);

	let params = [`x=${x}`, `y=${y}`, `z=${z}`, `frame_id='${frameId}'`, `speed=${speed}`];

	if (getAutoArm()) {
		params.push('auto_arm=True');
	}

	simpleOffboard();

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard();

		return `navigate_wait(${params.join(', ')})\n`;

	} else {
		if (frameId != 'body') {
			params.push(`yaw=float('nan')`);
		}
		return `navigate(${params.join(', ')})\n`;
	}
}

Blockly.Python.set_velocity = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	simpleOffboard();

	if (frameId == 'body') {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, frame_id='${frameId}')\n`;
	} else {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, yaw=float('nan'), frame_id='${frameId}')\n`;
	}
}

Blockly.Python.take_off = function(block) {
	autoArm = false; // lower auto_arm flag after take off
	simpleOffboard();

	let z = Blockly.Python.valueToCode(block, 'ALT', Blockly.Python.ORDER_NONE);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		simpleOffboard();

		return `navigate_wait(z=${z}, frame_id='body', auto_arm=True)\n`;
	} else {
		return `navigate(z=${z}, frame_id='body', auto_arm=True)\n`;
	}
}

Blockly.Python.land = function(block) {
	simpleOffboard();

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.landWait = true;
		simpleOffboard();

		return `land_wait()\n`;
	} else {
		return 'land()\n';
	}
}

Blockly.Python.angle = function(block) {
	// return [block.getFieldValue('ANGLE'), Blockly.Python.ORDER_UNARY_SIGN];
	Blockly.Python.definitions_['import_math'] = 'import math';
	return [`math.radians(${block.getFieldValue('ANGLE')})`, Blockly.Python.ORDER_FUNCTION_CALL];

}

Blockly.Python.set_yaw = function(block) {
	simpleOffboard();
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let code = `navigate(x=float('nan'), y=float('nan'), z=float('nan'), yaw=${yaw}, frame_id='body')\n`;
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.waitYaw = true;
		simpleOffboard();
		code += 'wait_yaw()\n';
	}
	return code;
}

Blockly.Python.wait_arrival = function(block) {
	rosDefinitions.waitArrival = true;
	simpleOffboard();
	return 'wait_arrival()\n';
}

Blockly.Python.arrived = function(block) {
	rosDefinitions.arrived = true;
	simpleOffboard();
	return ['arrived()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.wait = function(block) {
	initNode();
	return `rospy.sleep(${Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_NONE)})\n`;
}

Blockly.Python.get_position = function(block) {
	simpleOffboard();
	let frameId = buildFrameId(block);
	var code = `get_telemetry('${frameId}').${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_attitude = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.distance = function(block) {
	rosDefinitions.distance = true;
	simpleOffboard();

	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	return [`get_distance(${x}, ${y}, ${z}, '${frameId}')`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.rangefinder_distance = function(block) {
	initNode();
	Blockly.Python.definitions_['import_range'] = 'from sensor_msgs.msg import Range';
	return [`rospy.wait_for_message('rangefinder/range', Range).range`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.mode = function(block) {
	simpleOffboard();
	return [`get_telemetry().mode`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.armed = function(block) {
	simpleOffboard();
	return [`get_telemetry().armed`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.voltage = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('TYPE').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

function parseColor(color) {
	return {
		r: parseInt(color.substr(2, 2), 16),
		g: parseInt(color.substr(4, 2), 16),
		b: parseInt(color.substr(6, 2), 16)
	}
}

const PARSE_COLOR = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}(color):
  return {'r': int(color[1:3], 16), 'g': int(color[3:5], 16), 'b': int(color[5:7], 16)}`;

// TODO: weird code with colour_rgb block
Blockly.Python.set_effect = function(block) {
	rosDefinitions.setEffect = true;
	initNode();

	var effect = block.getFieldValue('EFFECT').toLowerCase();

	if (effect == 'rainbow' || effect == 'rainbow_fill') {
		return `set_effect(effect='${effect}')`;
	} else {
		let colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

		if (/^'(.*)'$/.test(colorCode)) { // is simple string
			let color = parseColor(colorCode);
			return `set_effect(effect='${effect}', r=${color.r}, g=${color.g}, b=${color.b})\n`;
		} else {
			let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
			return `set_effect(effect='${effect}', **${parseColor}(${colorCode}))\n`;
		}
	}
}

Blockly.Python.set_led = function(block) {
	rosDefinitions.setLeds = true;
	initNode();

	var index = Blockly.Python.valueToCode(block, 'INDEX', Blockly.Python.ORDER_NONE);
	var colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

	if (/^'(.*)'$/.test(colorCode)) { // is simple string
		let color = parseColor(colorCode);
		return `set_leds([LEDState(index=${index}, r=${color.r}, g=${color.g}, b=${color.b})])\n`;
	} else {
		let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
		return `set_leds([LEDState(index=${index}, **${parseColor}(${colorCode})])\n`;
	}
}

function pigpio() {
	Blockly.Python.definitions_['import_pigpio'] = 'import pigpio';
	Blockly.Python.definitions_['init_pigpio'] = 'pi = pigpio.pi()';
}

const GPIO_READ = `\ndef gpio_read(pin):
	pi.set_mode(pin, pigpio.INPUT)
	return pi.read(pin)\n`;

const GPIO_WRITE = `\ndef gpio_write(pin, level):
	pi.set_mode(pin, pigpio.OUTPUT)
	pi.write(pin, level)\n`;

const SET_SERVO = `\ndef gpio_write(pin, pwm):
	pi.set_mode(pin, pigpio.OUTPUT)
	pi.set_servo_pulsewidth(pin, pwm)\n`;

Blockly.Python.gpio_read = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_read'] = GPIO_READ;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	return [`gpio_read(${pin}))`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.gpio_write = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_write'] = GPIO_WRITE;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var level = Blockly.Python.valueToCode(block, 'LEVEL', Blockly.Python.ORDER_NONE);
	return `gpio_write(${pin}, ${level})\n`;
}

Blockly.Python.set_servo = function(block) {
	pigpio();
	Blockly.Python.definitions_['set_servo'] = SET_SERVO;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var pwm = Blockly.Python.valueToCode(block, 'PWM', Blockly.Python.ORDER_NONE);
	return `set_servo(${pin}, ${pwm})\n`;
}

var origPrint = Blockly.Python.text_print; // original print

Blockly.Python['text_print'] = function (block) {
	if (userCode) {
		return origPrint.apply(this, arguments);
	}

	rosDefinitions.print = true;
	initNode();

	var msg = Blockly.Python.valueToCode(block, 'TEXT', Blockly.Python.ORDER_NONE) || '\'\'';

	return `print_pub.publish(str(${msg}))\n`;
};

var origPrompt = Blockly.Python.text_prompt;

Blockly.Python['text_prompt_ext'] = function(block) {
	if (userCode) {
		return origPrompt.apply(this, arguments);
	}

	rosDefinitions.prompt = true;
	initNode();

	var msg = Blockly.Python.valueToCode(block, 'TEXT', Blockly.Python.ORDER_NONE) || '\'\'';

	if (block.getFieldValue('TYPE') == 'NUMBER') {
		return [`float(prompt(${msg}))`, Blockly.Python.ORDER_FUNCTION_CALL];
	} else {
		return [`prompt(${msg})`, Blockly.Python.ORDER_FUNCTION_CALL];
	}
};
