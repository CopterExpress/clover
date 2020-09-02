// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('rospy,srv,Trigger,get_telemetry,navigate,set_velocity,land');
Blockly.Python.addReservedWords('srv,Trigger,get_telemetry,navigate,set_velocity,land');
Blockly.Python.addReservedWords('navigate_wait,land_wait');
Blockly.Python.addReservedWords('SetLEDEffect,set_effect');
Blockly.Python.addReservedWords('SetLEDs,LEDState,set_leds');

var userCode = true; // global flag indicating whether the code for GUI is generating

const IMPORT_SRV = `from clover import srv
from std_srvs.srv import Trigger`;

const OFFBOARD = `get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)\n`;

// TODO: tolerance to parameters
const NAVIGATE_WAIT = `def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
  res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

  if not res.success:
    raise Exception(res.message)

  while not rospy.is_shutdown():
    telem = get_telemetry(frame_id='navigate_target')
    if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
      return
    else:
      print(telem)
      print(math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2))
    rospy.sleep(0.2)\n`;

const LAND_WAIT = `def land_wait():
  land()
  while get_telemetry().armed:
    rospy.sleep(0.2)\n`;

var rosDefinitions = {};

function generateROSDefinitions() {
	// order for ROS definitions is significant, so generate all ROS definitions as one
	var code = `rospy.init_node('flight', anonymous=True)\n\n`;
	if (!userCode) {
		code += `block_pub = rospy.Publisher('/clover_blocks/block', String, queue_size=10, latch=True)\n`;
	}
	if (rosDefinitions.print) {
		code += `print_pub = rospy.Publisher('/clover_blocks/print', String, queue_size=10, latch=True)\n`
	}
	if (rosDefinitions.offboard) {
		code += OFFBOARD;
	}
	if (rosDefinitions.setEffect) {
		code += `set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)\n`;
	}
	if (rosDefinitions.setLeds) {
		code += `set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs)\n`;
	}
	if (rosDefinitions.navigateWait) {
		Blockly.Python.definitions_['import_math'] = 'import math';
		code += NAVIGATE_WAIT;
	}
	if (rosDefinitions.landWait) {
		code += LAND_WAIT;
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

var origInit = Blockly.Python.init;

// Blockly.Python.init = function() {
// 	origInit.apply(this, arguments);
// 	if (!userCode) {
// 		Blockly.Python.definitions_['block_pub'] = `block_pub = rospy.Publisher('clover_blocks/block', String, queue_size=10, latch=True)`;
// 	}
// }

export function generateUserCode(workspace) {
	userCode = true;
	autoArm = true;
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	userCode = false;
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
	return `navigate(x=float('nan'), y=float('nan'), z=float('nan'), yaw=${yaw}, frame_id='body')\n`;
}

const WAIT_ARRIVAL = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}():
  while not rospy.is_shutdown():
    telem = get_telemetry(frame_id='navigate_target')
    if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
      return
    rospy.sleep(0.2)`;

Blockly.Python.wait_arrival = function(block) {
	simpleOffboard();
	var waitArrival = Blockly.Python.provideFunction_('wait_arrival', [WAIT_ARRIVAL]);
	return `${waitArrival}()\n`;
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
	Blockly.Python.definitions_['import_led_effect'] = 'from clover.srv import SetLEDEffect';

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
	Blockly.Python.definitions_['import_set_led'] = 'from led_msgs.srv import SetLEDs\nfrom led_msgs.msg import LEDState';

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
