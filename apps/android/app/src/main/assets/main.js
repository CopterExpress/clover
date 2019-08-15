function throttle(func, ms) {
	var isThrottled = false,
		savedArgs,
		savedThis;

	function wrapper() {
		if (isThrottled) {
			savedArgs = arguments;
			savedThis = this;
			return;
		}
		func.apply(this, arguments);
		isThrottled = true;
		setTimeout(function() {
			isThrottled = false;
			if (savedArgs) {
				wrapper.apply(savedThis, savedArgs);
				savedArgs = savedThis = null;
			}
		}, ms);
	}
	return wrapper;
}

function postAppMessage(msg) {
	if (window.webkit != undefined) {
		if (window.webkit.messageHandlers.appInterface != undefined) {
			window.webkit.messageHandlers.appInterface.postMessage(JSON.stringify(msg));
		}
	} else if (window.appInterface != undefined) {
		window.appInterface.postMessage(JSON.stringify(msg));
	}
}

function callNativeApp(name, msg) {
	try {
		postAppMessage(msg);
		return true;
	} catch(err) {
		console.warn('The native context does not exist yet');
		return false;
	}
}
var rcLastPublish = null;

function rcPublish() {
	callNativeApp('control', controlMessage);
	rcLastPublish = new Date();
}

rcPublishThrottled = throttle(rcPublish, 30);

setInterval(function() {
	if (rcLastPublish !== null && new Date() - rcLastPublish > 800) {
		rcPublishThrottled();
	}
}, 50);

var body = document.querySelector('body');
var stickLeft = document.querySelector('.stick-left');
var stickRight = document.querySelector('.stick-right');

var controlMessage = { x: 0, y: 0, z: 0, r: 0 };

function onStickTouchMove(touch) {
	var target = touch.target;
	var targetRect = target.getBoundingClientRect();
	var stickPointer = target.querySelector('.stick-pointer');

	var offsetX = touch.clientX - targetRect.left;
	var offsetY = touch.clientY - targetRect.top;

	var x = 2 * offsetX / targetRect.width;
	var y = 2 * offsetY / targetRect.height;

	x = Math.max(0, x);
	x = Math.min(2, x);
	y = Math.max(0, y);
	y = Math.min(2, y);

	stickPointer.style.left = (x * 50) + '%';
	stickPointer.style.top = (y * 50) + '%';

	x -= 1;
	y = 1 - y;

	if (target.matches('.stick-left')) {
		controlMessage.z = Math.round((y + 1) * 500);
		controlMessage.r = Math.round(x * 1000);
	} else if (target.matches('.stick-right')) {
		controlMessage.x = Math.round(y * 1000);
		controlMessage.y = Math.round(x * 1000);
	}
}

body.addEventListener('touchmove', function (e) {
	e.preventDefault();
});

function stickTouchStart(e) {
	setControlMode();
	callNativeApp('controlStart');
	onStickTouchMove(e.changedTouches[0]);
	rcPublishThrottled();
	e.stopPropagation();
	e.preventDefault();
}

function stickTouchMove(e) {
	for (touch of e.changedTouches) {
		onStickTouchMove(touch);
	}
	//onStickTouchMove(e.changedTouches[0]);
	rcPublishThrottled();
	e.stopPropagation();
	e.preventDefault();
}

function stickTouchEnd(e) {
	var pointer = e.target.querySelector('.stick-pointer');
	if (e.target.matches('.stick-left')) {
		controlMessage.r = 0;
		pointer.style.left = '50%';
	} else if (e.target.matches('.stick-right')) {
		controlMessage.x = 0;
		controlMessage.y = 0;
		pointer.style.left = '50%';
		pointer.style.top = '50%';
	}
	rcPublishThrottled();
}

stickLeft.addEventListener('touchmove', stickTouchMove);
stickRight.addEventListener('touchmove', stickTouchMove);
stickLeft.addEventListener('touchstart', stickTouchStart);
stickRight.addEventListener('touchstart', stickTouchStart);
stickLeft.addEventListener('touchend', stickTouchEnd);
stickRight.addEventListener('touchend', stickTouchEnd);
